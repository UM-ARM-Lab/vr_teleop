#include "openvr_driver.h"

// Destructor
OpenVRDriver::~OpenVRDriver() {
  if (m_pHMD != NULL)
  {
    vr::VR_Shutdown();
    m_pHMD = NULL;
  }
}

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

bool OpenVRDriver::waitForEvent() {
  // Process VREvent
  vr::VREvent_t event;
  while (m_pHMD->PollNextEvent(&event, sizeof(event)))
  {
    // Process event
    if (!processVREvent(event)) {
      char buf[1024];
      sprintf(buf, "(OpenVR) service quit\n");
      printf("%s\n", buf);
      //return false;
    }
  }
}

bool OpenVRDriver::processVREvent(const vr::VREvent_t &event)
{
  switch (event.eventType)
  {
    case vr::VREvent_TrackedDeviceActivated:
    {
      //SetupRenderModelForTrackedDevice(event.trackedDeviceIndex);
      char buf[1024];
      sprintf(buf, "(OpenVR) Device : %d attached\n", event.trackedDeviceIndex);
      printf("%s\n", buf);
    }
      break;

    case vr::VREvent_TrackedDeviceDeactivated:
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Device : %d detached\n", event.trackedDeviceIndex);
      printf("%s\n", buf);
    }
      break;

    case vr::VREvent_TrackedDeviceUpdated:
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Device : %d updated\n", event.trackedDeviceIndex);
      printf("%s\n", buf);
    }
      break;

    case (vr::VREvent_DashboardActivated) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Dashboard activated\n");
      printf("%s\n", buf);
    }
      break;

    case (vr::VREvent_DashboardDeactivated) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Dashboard deactivated\n");
      printf("%s\n", buf);

    }
      break;

    case (vr::VREvent_ChaperoneDataHasChanged) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Chaperone data has changed\n");
      printf("%s\n", buf);

    }
      break;

    case (vr::VREvent_ChaperoneSettingsHaveChanged) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Chaperone settings have changed\n");
      printf("%s\n", buf);
    }
      break;

    case (vr::VREvent_ChaperoneUniverseHasChanged) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Chaperone universe has changed\n");
      printf("%s\n", buf);

    }
      break;

    case (vr::VREvent_ApplicationTransitionStarted) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Application Transition: Transition has started\n");
      printf("%s\n", buf);

    }
      break;

    case (vr::VREvent_ApplicationTransitionNewAppStarted) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Application transition: New app has started\n");
      printf("%s\n", buf);

    }
      break;

    case (vr::VREvent_Quit) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) Received SteamVR Quit (%d", vr::VREvent_Quit, ")\n");
      printf("%s\n", buf);

      return false;
    }
      break;

    case (vr::VREvent_ProcessQuit) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) SteamVR Quit Process (%d", vr::VREvent_ProcessQuit, ")\n");
      printf("%s\n", buf);

      return false;
    }
      break;

    case (vr::VREvent_QuitAborted_UserPrompt) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) SteamVR Quit Aborted UserPrompt (%d", vr::VREvent_QuitAborted_UserPrompt, ")\n");
      printf("%s\n", buf);

      return false;
    }
      break;

    case (vr::VREvent_QuitAcknowledged) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) SteamVR Quit Acknowledged (%d", vr::VREvent_QuitAcknowledged, ")\n");
      printf("%s\n", buf);

      return false;
    }
      break;

    case (vr::VREvent_TrackedDeviceRoleChanged) :
    {

      char buf[1024];
      sprintf(buf, "(OpenVR) TrackedDeviceRoleChanged: %d\n", event.trackedDeviceIndex);
      printf("%s\n", buf);
      break;
    }

    case (vr::VREvent_TrackedDeviceUserInteractionStarted) :
    {
      char buf[1024];
      sprintf(buf, "(OpenVR) TrackedDeviceUserInteractionStarted: %d\n", event.trackedDeviceIndex);
      printf("%s\n", buf);
      break;
    }

    default:
      char buf[1024];
      sprintf(buf, "(OpenVR) Event: %d\n", event.eventType);
      printf("%s\n", buf);
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
    vr::VRControllerState_t *ontrollerState_ptr = &controllerState;

    vr::HmdVector3_t position;
    vr::HmdQuaternion_t quaternion;

    bool bPoseValid = trackedDevicePose.bPoseIsValid;
    vr::HmdVector3_t vVel;
    vr::HmdVector3_t vAngVel;
    vr::ETrackingResult eTrackingResult;

    // Get what type of device it is and work with its data
    vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
    switch (trackedDeviceClass) {
      case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
      {
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);

        position = getPosition(devicePose->mDeviceToAbsoluteTracking);
        quaternion = getRotation(devicePose->mDeviceToAbsoluteTracking);

        vVel = trackedDevicePose.vVelocity;
        vAngVel = trackedDevicePose.vAngularVelocity;
        eTrackingResult = trackedDevicePose.eTrackingResult;
        bPoseValid = trackedDevicePose.bPoseIsValid;

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

        vVel = trackedDevicePose.vVelocity;
        vAngVel = trackedDevicePose.vAngularVelocity;
        eTrackingResult = trackedDevicePose.eTrackingResult;
        bPoseValid = trackedDevicePose.bPoseIsValid;

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

      case vr::ETrackedDeviceClass::TrackedDeviceClass_TrackingReference:
        vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);

        position = getPosition(devicePose->mDeviceToAbsoluteTracking);
        quaternion = getRotation(devicePose->mDeviceToAbsoluteTracking);

        vVel = trackedDevicePose.vVelocity;
        vAngVel = trackedDevicePose.vAngularVelocity;
        eTrackingResult = trackedDevicePose.eTrackingResult;
        bPoseValid = trackedDevicePose.bPoseIsValid;

        break;
    }
  }

  pub.publish(msg);

  ros::spinOnce();
}