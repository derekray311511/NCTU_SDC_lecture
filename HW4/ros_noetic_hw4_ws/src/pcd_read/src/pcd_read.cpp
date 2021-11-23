#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>


int main (int argc, char** argv)
{
  // load in the point cloud map
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

  if (pcl::io::loadPCDFile<pcl::PointXYZI> ("src/data/map.pcd", *cloud) == -1) //* load the file
  {
    PCL_ERROR ("Couldn't read file map_pcd.pcd \n");
    return (-1);
  }
  std::cout << "Loaded "
            << cloud->width * cloud->height
            << " data points from map_pcd.pcd with the following fields: "
            << std::endl;

  // Initialize ROS
  ros::init (argc, argv, "map");  // my_node_name
  ros::NodeHandle nh;
  ros::Rate loop_rate(1);
  pcl::PCLPointCloud2 pcl_pc;
  sensor_msgs::PointCloud2 msg;

  ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>("map", 1);
  
  pcl::toPCLPointCloud2(*cloud, pcl_pc);
  
  // for (const auto& point: *cloud)
  //   std::cout << "    " << point.x
  //             << " "    << point.y
  //             << " "    << point.z << std::endl;


  // Transfer PCL pointcloud to msg type (sensor_msgs::PointCloud2)
  pcl_conversions::fromPCL(pcl_pc, msg);

  //this has been done in order to be able to visualize our PointCloud2 message on the RViz visualizer
  msg.header.frame_id="world";
  msg.header.stamp = ros::Time::now();

  while (ros::ok()){
    pub.publish(msg); //my_pc topic
    ros::spinOnce();
    loop_rate.sleep();
  }

  return (0);
}