#ifndef OPENVR_ROS_DRIVER_OPENVR_DRIVER_H
#define OPENVR_ROS_DRIVER_OPENVR_DRIVER_H


// ROS
#include <ros/ros.h>
#include <vive_msgs/ViveSystem.h>

// OpenVR
#include <openvr.h>

class OpenVRDriver
{
private:
  vr::IVRSystem *m_pHMD = NULL;

  ros::NodeHandle n;
  ros::Publisher pub;

  // Position and rotation of pose
  vr::HmdVector3_t getPosition(vr::HmdMatrix34_t matrix);
  vr::HmdQuaternion_t getRotation(vr::HmdMatrix34_t matrix);

public:
  ~OpenVRDriver();
  OpenVRDriver();

  bool waitForEvent();
  bool processVREvent(const vr::VREvent_t &event);

  void publishTrackingData();
};


#endif //OPENVR_ROS_DRIVER_OPENVR_DRIVER_H
