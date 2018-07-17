#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cmath>

#include <Windows.h>

#include <openvr.h>
#include "rclcpp/rclcpp.hpp"
#include "vive_msgs/msg/vive_system.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

struct Device {
  int index;
  vr::ETrackedDeviceClass type;
};

const std::string stateNames[3] = { "Released", "Touched ", "Pressed " }; // padded so each occupies the same number of characters
enum ButtonStates {
  Button_Released = 0,
  Button_Touched = 1,
  Button_Pressed = 2
};

const std::string buttonNames[4] = { "menu", "grip", "dpad", "trigger" };
const uint64_t buttonBitmasks[4] = {
  (uint64_t)pow(2, (int) vr::k_EButton_ApplicationMenu),
  (uint64_t)pow(2, (int) vr::k_EButton_Grip),
  (uint64_t)pow(2, (int) vr::k_EButton_SteamVR_Touchpad),
  (uint64_t)pow(2, (int) vr::k_EButton_SteamVR_Trigger)
};

Device devices[vr::k_unMaxTrackedDeviceCount];
std::vector<int> controllerIndices;
vr::IVRSystem* ivrSystem;

//*devices must be a pointer to an array of Device structs of size vr::k_unMaxTrackedDeviceCount
void catalogDevices(vr::IVRSystem* ivrSystem, Device* devices, bool printOutput) {
  std::string TrackedDeviceTypes[] = {
    "Invalid (disconnected)", 
    "HMD",
    "Controller",
    "Generic Tracker",
    "Tracking Reference (base station)",
    "Display Redirect"
  };

  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
    if (printOutput) {
      if (ivrSystem->GetTrackedDeviceClass(i) != 0) {
        std::cout << "Tracked Device " << i << " has type " << TrackedDeviceTypes[ivrSystem->GetTrackedDeviceClass(i)] << "." << std::endl;
      }
    }
    (*(devices + i)).index = i;
    (*(devices + i)).type = ivrSystem->GetTrackedDeviceClass(i);
  }
}

void catalogControllers(Device* devices, bool printOutput) {
  controllerIndices.erase(controllerIndices.begin(), controllerIndices.begin() + controllerIndices.size());
  for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; ++i) {
    if ((devices + i)->type == vr::TrackedDeviceClass_Controller) {
      controllerIndices.push_back((devices + i)->index);
    }
  }

  if (printOutput) {
    for (std::vector<int>::iterator i = controllerIndices.begin(); i != controllerIndices.end(); ++i) {
      std::cout << "There is a controller with index " << *i << "." << std::endl;
    }
    std::cout << "There are " << controllerIndices.size() << " controllers." << std::endl;
  }
}

// Create a Talker class that subclasses the generic rclcpp::Node base class.
// The main function below will instantiate the class as a ROS node.
class Talker : public rclcpp::Node {
public:
  explicit Talker(const std::string & topic_name) : Node("talker") {
    msg_ = std::make_shared<vive_msgs::msg::ViveSystem>();

    // Create a function for when messages are to be sent.
    auto publish_message = [this]() -> void {
      std::string controllersString[2] = {"",""};

      std::vector<int>::iterator itr = controllerIndices.begin();
      for (int controller = 0; itr + controller != controllerIndices.end(); ++controller) {
        while (msg_->controllers[controller].joystick.buttons.size() < 4) {
          msg_->controllers[controller].joystick.buttons.push_back(0);
        }

        while (msg_->controllers[controller].joystick.axes.size() < 3) {
          msg_->controllers[controller].joystick.axes.push_back(0.0);
        }

        vr::VRControllerState_t state;
        vr::TrackedDevicePose_t pose;
        //std::cout << "Controller " << *(itr+controller) << "    ";

        ivrSystem->GetControllerStateWithPose(vr::TrackingUniverseRawAndUncalibrated, *(itr+controller), &state, sizeof(state), &pose);

        // button states
        for (int button = 0; button < 4; ++button) {
          if ((state.ulButtonPressed & buttonBitmasks[button]) != 0) {
            //states[button + 4*controller] = Button_Pressed;
				*(msg_->controllers[controller].joystick.buttons.begin() + button) = Button_Pressed;
              }
              else if ((state.ulButtonTouched & buttonBitmasks[button]) != 0) {
                //states[button + 4*controller] = Button_Touched;
				*(msg_->controllers[controller].joystick.buttons.begin() + button) = Button_Touched;
              }
              else {
                //states[button + 4*controller] = Button_Released;
				*(msg_->controllers[controller].joystick.buttons.begin() + button) = Button_Released;
              }

	          controllersString[controller] += std::to_string(msg_->controllers[controller].joystick.buttons.at(button));
	          if(button != 3) {
                controllersString[controller] += ",";
	          }
            //std::cout << buttonNames[button] << ": " << stateNames[states[button]] << "  ";
        }
			
			//axis
			*(msg_->controllers[controller].joystick.axes.begin() + 0) = state.rAxis[0].x;
			*(msg_->controllers[controller].joystick.axes.begin() + 1) = state.rAxis[0].y;
			*(msg_->controllers[controller].joystick.axes.begin() + 2) = state.rAxis[1].x;

			//position
			msg_->controllers[controller].pose.position.x = pose.mDeviceToAbsoluteTracking.m[0][3];
			msg_->controllers[controller].pose.position.y = pose.mDeviceToAbsoluteTracking.m[1][3];
			msg_->controllers[controller].pose.position.z = pose.mDeviceToAbsoluteTracking.m[2][3];

			//orientation
			tf2::Matrix3x3 rotMatrix;
			tf2::Quaternion quaternion;

			rotMatrix.setValue(pose.mDeviceToAbsoluteTracking.m[0][0], pose.mDeviceToAbsoluteTracking.m[0][1], pose.mDeviceToAbsoluteTracking.m[0][2],
			                   pose.mDeviceToAbsoluteTracking.m[1][0], pose.mDeviceToAbsoluteTracking.m[1][1], pose.mDeviceToAbsoluteTracking.m[1][2],
				               pose.mDeviceToAbsoluteTracking.m[2][0], pose.mDeviceToAbsoluteTracking.m[2][1], pose.mDeviceToAbsoluteTracking.m[2][2]
			);

			rotMatrix.getRotation(quaternion);
			msg_->controllers[controller].pose.orientation.x = quaternion.x();
			msg_->controllers[controller].pose.orientation.y = quaternion.y();
			msg_->controllers[controller].pose.orientation.z = quaternion.z();
			msg_->controllers[controller].pose.orientation.w = quaternion.w();

            /*std::string matrix = "";
            matrix += "{";
            for (int r = 0; r < 3; ++r) {
              matrix += "{";
              for (int c = 0; c < 4; ++c) {
                matrix += std::to_string(pose.mDeviceToAbsoluteTracking.m[r][c]);
                if (c != 3) {
                  matrix += ", ";
                }
              }
              matrix += "}";
              if (r != 2) {
                matrix += ", ";
              }
            }
            matrix += "}";
            for (int i = matrix.size(); i < 138; ++i) {
              matrix += " ";
            }
            std::cout << matrix << "                                                  ";
            //std::cout << "ETrackingResult: " << pose.eTrackingResult << "   ";*/
          }

          //std::cout << "\r";

      //RCLCPP_INFO(this->get_logger(), "Publishing: controller 1 buttons = [%s] and controller 2 buttons = [%s]", controllersString[0], controllersString[1]);
		  //msg_->controllers[0].joystick.buttons.push_back(2);
		  pub_->publish(msg_);
      };

    // Create a publisher with a custom Quality of Service profile.
    rmw_qos_profile_t custom_qos_profile = rmw_qos_profile_default;
    custom_qos_profile.depth = 7;
    //pub_ = this->create_publisher<sensor_msgs::msg::Joy>(topic_name, custom_qos_profile);
    pub_ = this->create_publisher<vive_msgs::msg::ViveSystem>(topic_name, custom_qos_profile);

    // Use a timer to schedule periodic message publishing.
    timer_ = this->create_wall_timer(1ms, publish_message);
  }

private:
  size_t count_ = 1;
  /*std::shared_ptr<sensor_msgs::msg::Joy> msg_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pub_;*/
  std::shared_ptr<vive_msgs::msg::ViveSystem> msg_;
  rclcpp::Publisher<vive_msgs::msg::ViveSystem>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within the launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  // Initialize any global resources needed by the middleware and the client library.
  // You must call this before using any other part of the ROS system.
  // This should be called once per process.
  rclcpp::init(argc, argv);

  // Create a node.
  auto node = std::make_shared<Talker>("vive");

  // Check whether there is an HMD plugged-in and the SteamVR runtime is installed
  if (!vr::VR_IsHmdPresent()) {
    std::cout << "No HMD was found in the system, quitting app" << std::endl;
    return -1;
  } else {
    std::cout << "An HMD was successfully found in the system" << std::endl;
  }

  if (!vr::VR_IsRuntimeInstalled()) {
    std::cout << "Runtime was not found, quitting app" << std::endl;
    return -1;
  } else {
    const char* runtime_path = vr::VR_RuntimePath();
    std::cout << "Runtime correctly installed at '" << runtime_path << "'" << std::endl;
  }

  // Initialize system
  vr::HmdError error;
  ivrSystem = vr::VR_Init(&error, vr::VRApplication_Other);
  std::cout << "VR Init status: " << vr::VR_GetVRInitErrorAsSymbol(error) << std::endl;
  std::cout << "Pointer to the IVRSystem is " << ivrSystem << std::endl;

  catalogDevices(ivrSystem, devices, false);
  catalogControllers(devices, true);

  while (true) {
    // Wait for input
    std::cout << "> ";
    std::string input;
    std::cin >> input;
    
    if (input == "refresh") {
      catalogDevices(ivrSystem, devices, true);
      catalogControllers(devices, true);
    } else if (input == "start") {
      rclcpp::spin(node);
    } else if (input == "exit") {
      break;
    } else if (input == "help") {
      std::cout << "Available commands:" << std::endl;
      std::cout << "  help - shows this" << std::endl;
      std::cout << "  refresh - requeries system for VR devices" << std::endl;
      std::cout << "  start - starts broadcasting VR data" << std::endl;
      std::cout << "  exit - quits program" << std::endl;
    } else {
      std::cout << "Command not found, type 'help' for help" << std::endl;
    }
  }
  
  vr::VR_Shutdown();
  rclcpp::shutdown();

  return 0;
}
