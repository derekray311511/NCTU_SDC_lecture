#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

using namespace std;

class icp_localization
{
  private:
    // Publisher and Subscriber
    ros::Subscriber sub_lidar_scan;
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
  public:
    // initializer
    icp_localization();
    // Callback functions
    void cb_matching(const sensor_msgs::PointCloud2 &msg);
    void cb_gps(const geometry_msgs::PointStamped &msg);
    // functions
    Eigen::Matrix4f get_gps();
    Eigen::Matrix4f get_transfrom(const std::string link_name);
    pcl::PointCloud<pcl::PointXYZI> downSample(const sensor_msgs::PointCloud2 &msg);
    
};

// ========== initializer ==========
icp_localization::icp_localization()
{
  printf("Hi, this is icp_localization's initialization\n");
  // load in the downsampled map
  // pcl::PointCloud<pcl::PointXYZI>::Ptr map (new pcl::PointCloud<pcl::PointXYZI>);
  map = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("src/data/map_downsample.pcd", *map) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map_downsample.pcd \n");
    exit(0);
  }
  std::cout << "Loaded "
            << map->width * map->height
            << " data points from map_downsample.pcd with the following fields: "
            << std::endl;

  // Subscribe and publish
  sub_lidar_scan = nh.subscribe("lidar_points", 10, &icp_localization::cb_matching, this);
  pub_matched_lidar_scan = nh.advertise<sensor_msgs::PointCloud2>("pub_matched_lidar_scan", 10);
  pub_odem = nh.advertise<nav_msgs::Odometry>("result_odom", 10);
  pub_map = nh.advertise<sensor_msgs::PointCloud2>("load_map", 10);

  //wait for gps
  std::cout << "waiting for gps" << std::endl;
  initial_guess = get_gps();
  std::cout << "get initial guess" << std::endl;
  std::cout << initial_guess << std::endl;

  printf("init done \n");
}

// ========== Get initial guess from gps ==========
Eigen::Matrix4f icp_localization::get_gps()
{
  Eigen::Matrix4f trans_gps;
  geometry_msgs::PointStampedConstPtr gps_point;
  // Subscribe a topic for once
  gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", nh);
  // Given initial angle
  double yaw=-2.2370340344819 ; // rad
  trans_gps << cos(yaw), -sin(yaw),    0,  (*gps_point).point.x,
               sin(yaw),  cos(yaw),    0,  (*gps_point).point.y,
                  0,         0,        1,  (*gps_point).point.z,
                  0,         0,        0,       1;
  return trans_gps;
}

// ========== DownSample PointCloud ==========
pcl::PointCloud<pcl::PointXYZI> icp_localization::downSample(const sensor_msgs::PointCloud2 &msg)
{
  pcl::PCLPointCloud2::Ptr cloud2 (new pcl::PCLPointCloud2 ());
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

  // Transform sensor_msgs to PCLPointCloud2
  pcl::fromROSMsg(msg, *cloud);
  std::cerr << "Before filtered: " << cloud->points.size() << endl;

  // Create the filtering object
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  pcl::toPCLPointCloud2(*cloud, *cloud2);
  sor.setInputCloud (cloud2);
  sor.setLeafSize (0.1f, 0.1f, 0.1f);
  sor.filter (*cloud2);
  pcl::fromPCLPointCloud2(*cloud2, *cloud);
  std::cerr << "After  filtered: " << cloud2->width * cloud2->height << endl;

  return *cloud;
}

// ========== Get Transform using tf listener ==========
Eigen::Matrix4f icp_localization::get_transfrom(std::string link_name)
{
	tf::StampedTransform transform;
	Eigen::Matrix4f eigen_transform;

  try{
		tf_listener.waitForTransform("base_link", link_name, ros::Time(0), ros::Duration(5.0));
		tf_listener.lookupTransform("base_link", link_name, ros::Time(0), transform);
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
  pcl::PointCloud<pcl::PointXYZI> Final;
  // pcl::toROSMsg(icp_localization::downSample(msg), filtered_cloud);
  
  Eigen::Matrix4f trans = get_transfrom("velodyne");
  // Do voxel grid filtering and transform to basic_link
  transformPointCloud(icp_localization::downSample(msg), *filtered_cloud, trans);
  ROS_INFO("Small PointCloud transformed to base_link");

  // Do ICP (Iterative Closest Point) for small cloud and map cloud
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(filtered_cloud);
  icp.setInputTarget(map);
  // set ICP parameters
  icp.setMaximumIterations (1000);
  icp.setTransformationEpsilon (1e-12);
  icp.setMaxCorrespondenceDistance (4);
  icp.setEuclideanFitnessEpsilon (0.01);
  icp.setRANSACOutlierRejectionThreshold (0.06);
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
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(),"/world","/base_link"));
  
  // Publish map to world
  sensor_msgs::PointCloud2 Final_map;
  pcl::toROSMsg(*map, Final_map);
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
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = initial_guess(0,3);
  odom.pose.pose.position.y = initial_guess(1,3);
  odom.pose.pose.position.z = initial_guess(2,3);
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
}

int main(int argc, char** argv){
  ros::init(argc, argv, "icp_locolization");
  icp_localization haha_icp;
  // ros::Rate loop_rate(1);
  ros::spin();
}