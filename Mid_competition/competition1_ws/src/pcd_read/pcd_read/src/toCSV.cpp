#include <iostream>
#include <fstream>
#include <vector>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include "math.h"

int array[210][7] = {0};

void write(const nav_msgs::Odometry odom)
{
  printf("1\n");
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "toCSV");
  ros::NodeHandle nh;
  ros::Subscriber sub_toCSV;

  sub_toCSV = nh.subscribe("result_odom", 10, &write);
  ros::spin();
  printf("Hi\n");
}