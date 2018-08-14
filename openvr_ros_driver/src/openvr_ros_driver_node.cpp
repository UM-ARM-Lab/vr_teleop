// ROS
#include <ros/ros.h>

#include "openvr_driver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "openvr_ros_driver_node");

  OpenVRDriver openvr_driver;

  ros::Rate r(90); // in hz
  while (ros::ok())
   {
     openvr_driver.publishTrackingData();

     r.sleep();
   }

  ros::shutdown();
  return 0;
}