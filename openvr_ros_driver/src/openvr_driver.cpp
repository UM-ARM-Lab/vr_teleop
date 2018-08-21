#include "openvr_driver.h"

// Constructor
OpenVRDriver::OpenVRDriver() {
  vr::EVRInitError eError = vr::VRInitError_None;
  m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

  if (eError != vr::VRInitError_None)
  {
    m_pHMD = NULL;
    char buf[1024];
    sprintf(buf, "Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
    printf("%s\n", buf);
    exit(EXIT_FAILURE);
  }

  pub = n.advertise<vive_msgs::ViveSystem>("vive", 10);
}

bool OpenVRDriver::handleVREvent() {
  vr::VREvent_t event;

  if (!m_pHMD->PollNextEvent(&event, sizeof(event))) return true;

  // Process event
  if (!printVREvent(event)) {
    char buf[1024];
    sprintf(buf, "Service quit");
    printf("%s\n", buf);
    //return false;
  }
}

bool OpenVRDriver::printVREvent(const vr::VREvent_t &event)
{
  switch (event.eventType)
  {
    case vr::VREvent_TrackedDeviceActivated:
      std::cout << "Device " << event.trackedDeviceIndex << " attached" << std::endl;
      break;

    case vr::VREvent_TrackedDeviceDeactivated:
      std::cout << "Device " << event.trackedDeviceIndex << " detached" << std::endl;
      break;

    case vr::VREvent_TrackedDeviceUpdated:
      std::cout << "Device " << event.trackedDeviceIndex << " updated" << std::endl;
      break;

    case vr::VREvent_TrackedDeviceRoleChanged:
      std::cout << "TrackedDeviceRoleChanged: " << event.trackedDeviceIndex << std::endl;
      break;

    case vr::VREvent_TrackedDeviceUserInteractionStarted:
      std::cout << "TrackedDeviceUserInteractionStarted: " << event.trackedDeviceIndex << std::endl;
      break;

    default:
      std::cout << vr::VRSystem()->GetEventTypeNameFromEnum((vr::EVREventType) event.eventType) << " (" <<  event.eventType << ")" << std::endl;
      break;
  }

  return true;
}

// Get the quaternion representing the rotation
vr::HmdQuaternion_t OpenVRDriver::getRotation(vr::HmdMatrix34_t matrix) {
  vr::HmdQuaternion_t q;

  q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
  q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
  q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
  q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
  q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
  q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
  q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
  return q;
}

// Get the vector representing the position
vr::HmdVector3_t OpenVRDriver::getPosition(vr::HmdMatrix34_t matrix) {
  vr::HmdVector3_t vector;

  vector.v[0] = matrix.m[0][3];
  vector.v[1] = matrix.m[1][3];
  vector.v[2] = matrix.m[2][3];

  return vector;
}

void OpenVRDriver::publishTrackingData() {
  vive_msgs::ViveSystem msg;

  // Process SteamVR device states
  for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
  {
    // if not connected just skip the rest of the routine
    if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
      continue;

    vr::TrackedDevicePose_t trackedDevicePose;
    vr::TrackedDevicePose_t *devicePose = &trackedDevicePose;

    vr::VRControllerState_t controllerState;

    vr::HmdVector3_t position;
    vr::HmdQuaternion_t quaternion;

    // Get what type of device it is and work with its data
    vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
    switch (trackedDeviceClass) {
      case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
      {
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);

        position = getPosition(devicePose->mDeviceToAbsoluteTracking);
        quaternion = getRotation(devicePose->mDeviceToAbsoluteTracking);

        vive_msgs::HeadMountedDisplay msg_hmd;

        // Pose
        msg_hmd.posestamped.pose.position.x = position.v[0];
        msg_hmd.posestamped.pose.position.y = position.v[1];
        msg_hmd.posestamped.pose.position.z = position.v[2];
        msg_hmd.posestamped.pose.orientation.x = quaternion.x;
        msg_hmd.posestamped.pose.orientation.y = quaternion.y;
        msg_hmd.posestamped.pose.orientation.z = quaternion.z;
        msg_hmd.posestamped.pose.orientation.w = quaternion.w;

        // Header
        msg_hmd.posestamped.header.stamp = ros::Time::now();
        msg_hmd.posestamped.header.frame_id = "vive_base";

        msg.hmd = msg_hmd;

        break;
      }

      case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
      {
        vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, sizeof(controllerState), &trackedDevicePose);

        position = getPosition(devicePose->mDeviceToAbsoluteTracking);
        quaternion = getRotation(devicePose->mDeviceToAbsoluteTracking);

        vive_msgs::Controller msg_controller;

        // Role
        msg_controller.id = vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice);

        // Pose
        msg_controller.posestamped.pose.position.x = position.v[0];
        msg_controller.posestamped.pose.position.y = position.v[1];
        msg_controller.posestamped.pose.position.z = position.v[2];
        msg_controller.posestamped.pose.orientation.x = quaternion.x;
        msg_controller.posestamped.pose.orientation.y = quaternion.y;
        msg_controller.posestamped.pose.orientation.z = quaternion.z;
        msg_controller.posestamped.pose.orientation.w = quaternion.w;

        // Buttons
        const uint64_t buttonBitmasks[4] = {
            (uint64_t)pow(2, (int) vr::k_EButton_ApplicationMenu),
            (uint64_t)pow(2, (int) vr::k_EButton_Grip),
            (uint64_t)pow(2, (int) vr::k_EButton_SteamVR_Touchpad),
            (uint64_t)pow(2, (int) vr::k_EButton_SteamVR_Trigger)
        };

        msg_controller.joystick.buttons.resize(4);

        for (int button = 0; button < 4; ++button) {
          if ((controllerState.ulButtonPressed & buttonBitmasks[button]) != 0) {
            msg_controller.joystick.buttons[button] = 2;
          } else if ((controllerState.ulButtonTouched & buttonBitmasks[button]) != 0) {
            msg_controller.joystick.buttons[button] = 1;
          } else {
            msg_controller.joystick.buttons[button] = 0;
          }
        }

        // Axes
        msg_controller.joystick.axes.push_back(controllerState.rAxis[0].x);
        msg_controller.joystick.axes.push_back(controllerState.rAxis[0].y);
        msg_controller.joystick.axes.push_back(controllerState.rAxis[1].x);

        // Header
        msg_controller.posestamped.header.stamp = ros::Time::now();
        msg_controller.posestamped.header.frame_id = "vive_base";

        msg.controllers.push_back(msg_controller);

        break;
      }
    }
  }

  pub.publish(msg);

  ros::spinOnce();
}