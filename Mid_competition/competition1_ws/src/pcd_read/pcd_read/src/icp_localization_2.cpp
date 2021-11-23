#include <iostream>
#include <fstream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

using namespace std;
string result_save_path, map_path;

class icp_localization
{
  private:
    // Publisher and Subscriber
    ros::Subscriber sub_lidar_scan;
    ros::Subscriber sub_wheel_odom;
    ros::Publisher pub_matched_lidar_scan;
    ros::Publisher pub_map;
    ros::Publisher pub_odem;
    // ros initialize node
    ros::NodeHandle nh;
    // TF Transform
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    // Variable
    Eigen::Matrix4f initial_guess;
    pcl::PointCloud<pcl::PointXYZI>::Ptr map;
    sensor_msgs::PointCloud2 Final_map;
    sensor_msgs::PointCloud2 Final_cloud;
    sensor_msgs::Imu imu;
    double diff_x, diff_y, diff_z;
    nav_msgs::Odometry wheel_odom_old;
    
  public:
    // initializer
    icp_localization();
    // Callback functions
    void cb_matching(const sensor_msgs::PointCloud2 &msg);
    void cb_gps(const geometry_msgs::PointStamped &msg);
    void get_diff(const nav_msgs::Odometry &wheel_odom_current);
    // functions
    Eigen::Matrix4f get_gps();
    Eigen::Matrix4f get_transfrom(const std::string link_name);
    pcl::PointCloud<pcl::PointXYZI> downSample(const sensor_msgs::PointCloud2 &msg);
    // vector to save result
    double pose_array[400][7] = {0};
    int num = 1;

};

// ========== initializer ==========
icp_localization::icp_localization()
{
  // Get param from launch file
  nh.getParam("/map_path", map_path);
  nh.getParam("/ICP_localization_2/result_save_path", result_save_path);

  printf("Hi, this is icp_localization's initialization\n");
  // load in the downsampled map
  // pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
  map = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
  // if (pcl::io::loadPCDFile<pcl::PointXYZI> ("src/data/noetic/nuscenes_map_downsample.pcd", *map) == -1) //* load the file
  if (pcl::io::loadPCDFile<pcl::PointXYZI> (map_path, *map) == -1) //* load the file
  // if (pcl::io::loadPCDFile<pcl::PointXYZI> ("~/SDC_lecture/mid_competition/competition1_ws/src/data/noetic/nuscenes_map_downsample.pcd", *map) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map_downsample.pcd \n");
    exit(0);
  }
  std::cout << "Loaded "
            << map->width * map->height
            << " data points from nuscenes_map_downsample.pcd with the following fields: "
            << std::endl;

  ROS_INFO("Subscribe and publish");
  // Subscribe and publish
  sub_lidar_scan = nh.subscribe("lidar_points", 20, &icp_localization::cb_matching, this);
  sub_wheel_odom = nh.subscribe("wheel_odometry", 20, &icp_localization::get_diff, this);
  pub_matched_lidar_scan = nh.advertise<sensor_msgs::PointCloud2>("matched_lidar_scan", 20);
  pub_odem = nh.advertise<nav_msgs::Odometry>("result_odom", 20);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("load_map", 10);

  //wait for gps
  std::cout << "waiting for gps" << std::endl;
  initial_guess = get_gps();
  std::cout << "get initial guess" << std::endl;
  std::cout << initial_guess << std::endl;

  // initialize diff
  wheel_odom_old.pose.pose.position.x = 0;
  wheel_odom_old.pose.pose.position.y = 0;
  wheel_odom_old.pose.pose.position.z = 0;
  ros::spinOnce();
  // printf("init done \n");
}

// ========== Get initial guess from gps ==========
Eigen::Matrix4f icp_localization::get_gps()
{
  Eigen::Matrix4f trans_gps;
  geometry_msgs::PointStampedConstPtr gps_point;
  // Subscribe a topic for once
  gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);
  // Given initial angle and guess
  double pi = 3.1415926;
  double yaw = 0 ; // rad
  double init_x = 1773.433472;
  double init_y = 866.344177;
  double init_z = 0.0;
  // trans_gps << cos(yaw), -sin(yaw),    0,  (*gps_point).point.x,
  //              sin(yaw),  cos(yaw),    0,  (*gps_point).point.y,
  //                 0,         0,        1,  (*gps_point).point.z,
  //                 0,         0,        0,       1;
  trans_gps << cos(yaw), -sin(yaw),    0,     init_x,
               sin(yaw),  cos(yaw),    0,     init_y,
                  0,         0,        1,     init_z,
                  0,         0,        0,       1;
  return trans_gps;
}

void icp_localization::get_diff(const nav_msgs::Odometry &wheel_odom_current)
{
  diff_x = (wheel_odom_current.pose.pose.position.x - wheel_odom_old.pose.pose.position.x) / 1.67; // hz diff
  diff_y = (wheel_odom_current.pose.pose.position.y - wheel_odom_old.pose.pose.position.y) / 1.67;
  diff_z = (wheel_odom_current.pose.pose.position.z - wheel_odom_old.pose.pose.position.z) / 1.67;
  wheel_odom_old.pose.pose.position.x = wheel_odom_current.pose.pose.position.x;
  wheel_odom_old.pose.pose.position.y = wheel_odom_current.pose.pose.position.y;
  wheel_odom_old.pose.pose.position.z = wheel_odom_current.pose.pose.position.z;
}

// ========== DownSample PointCloud ==========
pcl::PointCloud<pcl::PointXYZI> icp_localization::downSample(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  // Transform sensor_msgs to PCLPointCloud2
  pcl::fromROSMsg(msg, *cloud);
  std::cerr << "Scan Before filtered: " << cloud->points.size() << endl;
  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::toPCLPointCloud2(*cloud, *cloud2);
  sor.setInputCloud (cloud2);
  sor.setFilterFieldName("z");
  sor.setFilterLimits(-2.0, 10.5);
  sor.setLeafSize (0.1f, 0.1f, 0.6f);
  sor.filter (*cloud2);
  pcl::fromPCLPointCloud2(*cloud2, *cloud);
  std::cerr << "Scan After  filtered: " << cloud2->width * cloud2->height << endl;

  return *cloud;
}

// ========== Get Transform using tf listener ==========
Eigen::Matrix4f icp_localization::get_transfrom(std::string link_name)
{
	tf::StampedTransform transform;
	Eigen::Matrix4f eigen_transform;

  try{
		tf_listener.waitForTransform("car", link_name, ros::Time(0), ros::Duration(5.0));
		tf_listener.lookupTransform("car", link_name, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
		return eigen_transform;
	}
	Eigen::Quaternionf q(transform.getRotation().getW(), \
		transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
	Eigen::Matrix3f mat = q.toRotationMatrix();
	eigen_transform << mat(0,0), mat(0,1), mat(0,2), transform.getOrigin().getX(),
                     mat(1,0), mat(1,1), mat(1,2), transform.getOrigin().getY(),
                     mat(2,0), mat(2,1), mat(2,2), transform.getOrigin().getZ(),
                         0   ,     0   ,     0   ,     1   ;
  // pcl_ros::transformAsMatrix(transform, eigen_transform);
	return eigen_transform;
}

// ========== Match the PointCloud using icp ==========
void icp_localization::cb_matching(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_map1(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr crop_map2(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI> Final;
  
  // Crop the map
  pcl::PassThrough<pcl::PointXYZI> pass;
  pass.setInputCloud(map);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(initial_guess(0, 3) - 100.0, initial_guess(0, 3) + 100.0);
  pass.filter(*crop_map1);//儲存處理之後的點雲

  pass.setInputCloud(crop_map1);
  pass.setFilterFieldName("y");
  pass.setFilterLimits(initial_guess(1, 3) - 100.0, initial_guess(1, 3) + 100.0);
  pass.filter(*crop_map2);//儲存處理之後的點雲

  pass.setInputCloud(crop_map2);
  pass.setFilterFieldName("z");
  // pass.setFilterLimits(initial_guess(2, 3) - 2.5, initial_guess(2, 3) + 20.5);
  pass.setFilterLimits(1, 8);
  pass.filter(*crop_map2);//儲存處理之後的點雲
  
  Eigen::Matrix4f trans = get_transfrom("nuscenes_lidar");
  // Do voxel grid filtering and transform to car
  transformPointCloud(icp_localization::downSample(msg), *filtered_cloud, trans);
  ROS_INFO("Lidar Scan PointCloud transformed to car");

  // // Remove floor points
  // pass.setInputCloud(filtered_cloud);
  // pass.setFilterFieldName("z");
  // pass.setFilterLimits(0.2, 10.0);
  // pass.filter(*filtered_cloud);//儲存處理之後的點雲

  // voxel grid filter to z axis
  pcl::VoxelGrid<pcl::PointXYZI> sor;
  // sor.setInputCloud (filtered_cloud);
  // sor.setFilterFieldName("z");
  // sor.setFilterLimits(0.0, 2.0);
  // // Set to true if we want to return the data outside the interval specified by setFilterLimits (min, max).
  // sor.setFilterLimitsNegative(true);
  // sor.setLeafSize (0.1f, 0.1f, 0.4f);
  // sor.filter (*filtered_cloud2);

  sor.setInputCloud (filtered_cloud);
  sor.setFilterFieldName("z");
  sor.setFilterLimits(1.0, 7.5);
  sor.setLeafSize (0.1f, 0.1f, 0.4f);
  // sor.setFilterLimitsNegative(true);
  sor.filter (*filtered_cloud2);

  // Do ICP (Iterative Closest Point) for small cloud and map cloud
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(filtered_cloud2);
  // icp.setInputTarget(map);
  icp.setInputTarget(crop_map2);
  // set ICP parameters ================================================
  icp.setMaximumIterations (1000);              //最大迭代次數
  icp.setTransformationEpsilon (1e-12);         //上次轉換與當前轉換的差值(early stop)
  icp.setMaxCorrespondenceDistance (0.75);         //Distance to see
  icp.setEuclideanFitnessEpsilon (0.00075);        //前後兩次迭代誤差的差值(early stop)
  icp.setRANSACOutlierRejectionThreshold (0.05);//距離小於等於inlier_threshold的點對才會被當成是RANSAC的inlier
  icp.align(Final, initial_guess);

  std::cout << "has converged:" << icp.hasConverged() << " score: " <<
  icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  initial_guess = icp.getFinalTransformation();

  // broadcast TF from world to basic link
  tf::Transform tf_transform;
  tf::Vector3 origin;
  origin.setValue(initial_guess(0,3), initial_guess(1,3), initial_guess(2,3));
  tf::Matrix3x3 tf3d;
  tf3d.setValue((initial_guess(0,0)), (initial_guess(0,1)), (initial_guess(0,2)),
                (initial_guess(1,0)), (initial_guess(1,1)), (initial_guess(1,2)),
                (initial_guess(2,0)), (initial_guess(2,1)), (initial_guess(2,2)));
  tf::Quaternion tfqt;
  tf3d.getRotation(tfqt);
  tf_transform.setOrigin(origin);
  tf_transform.setRotation(tfqt);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(),"world","car"));
  
  // Publish map to world
  sensor_msgs::PointCloud2 Final_map;
  // pcl::toROSMsg(*map, Final_map);
  pcl::toROSMsg(*crop_map2, Final_map);
  Final_map.header.frame_id = "world";
  Final_map.header.stamp = ros::Time::now();
  pub_map.publish(Final_map);

  // Publish local lidar scan pointcloud after ICP.
  sensor_msgs::PointCloud2 Final_cloud;
  pcl::toROSMsg(Final, Final_cloud);
  Final_cloud.header = msg.header;
  Final_cloud.header.frame_id = "world";
  pub_matched_lidar_scan.publish(Final_cloud);

  // Publish Odometry
  nav_msgs::Odometry odom;
  odom.header.frame_id = "world";
  odom.child_frame_id = "car";
  odom.pose.pose.position.x = initial_guess(0,3);
  odom.pose.pose.position.y = initial_guess(1,3);
  odom.pose.pose.position.z = initial_guess(2,3);
  initial_guess(0,3) = initial_guess(0,3) + diff_x; // 0.37
  initial_guess(1,3) = initial_guess(1,3) + diff_y;
  initial_guess(2,3) = initial_guess(2,3) + diff_z;
  tf2::Matrix3x3 m;
  m.setValue(initial_guess(0,0) ,initial_guess(0,1) ,initial_guess(0,2) ,
             initial_guess(1,0) ,initial_guess(1,1) ,initial_guess(1,2) ,
             initial_guess(2,0) ,initial_guess(2,1) ,initial_guess(2,2));
  tf2::Quaternion tfq2;
  m.getRotation(tfq2);
  odom.pose.pose.orientation.x = tfq2[0];
  odom.pose.pose.orientation.y = tfq2[1];
  odom.pose.pose.orientation.z = tfq2[2];
  odom.pose.pose.orientation.w = tfq2[3];
  pub_odem.publish(odom);

  // Save result to an array
  tf::Quaternion qq(tfq2[0], tfq2[1], tfq2[2], tfq2[3]);
  tf::Matrix3x3 mm(qq);
  double roll, pitch, yaw;
  mm.getRPY(roll, pitch, yaw);

  pose_array[num][0] = num;
  pose_array[num][1] = odom.pose.pose.position.x;
  pose_array[num][2] = odom.pose.pose.position.y;
  pose_array[num][3] = odom.pose.pose.position.z;
  pose_array[num][4] = yaw;
  pose_array[num][5] = pitch;
  pose_array[num][6] = roll;
  num++;

}

// Write the results to .csv file
void toCSV(double array[][7], int num)
{ 
  printf("num of scan = %d\n", num);
  ROS_INFO("num of scan = %d", num);
  for (int i = 1; i <= num; i++) {
    printf("id:%.0f %f %f %f %f %f %f\n", array[i][0], array[i][1], array[i][2], array[i][3],
                                          array[i][4], array[i][5], array[i][6]);
  }
  std::ofstream myfile;
  // myfile.open ("src/output/icp_2.csv");
  myfile.open (result_save_path);
  myfile << "id,x,y,z,yaw,pitch,roll\n";
  for (int i = 1; i <= num; i++) {
    for (int j = 0; j < 6; j++) {
      myfile << array[i][j] << ",";
    }
    myfile << array[i][6] << "\n";
  }
  myfile.close();
}

int main(int argc, char** argv){
  ros::init(argc, argv, "icp_locolization");
  icp_localization haha_icp;
  printf("Initial done! Start to spin.\n");
  ros::spin();
  haha_icp.num--;
  printf("Writing odometry to cdv file...\n");
  toCSV(haha_icp.pose_array, haha_icp.num);
  printf("Done.\n");
}