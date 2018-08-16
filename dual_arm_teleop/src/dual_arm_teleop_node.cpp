#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
#include <vive_msgs/ViveSystem.h>

// TF
#include <tf/transform_broadcaster.h>

#include "robot_arm.h"

class DualArmTeleop
{
private:
  ros::NodeHandle n;
  ros::Subscriber sub_vive;
  ros::Publisher pub_display_robot_state;

  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;

  RobotArm* victor_arms[2];

public:
  DualArmTeleop()
  {
    sub_vive = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);
    pub_display_robot_state = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

    // Initialize kinematic model
    robot_model_loader::RobotModelLoader robot_model_load("robot_description");

    kinematic_model = robot_model_load.getModel();
    kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

    kinematic_state->setToDefaultValues();

    // Initialize each arm
    victor_arms[0] = new RobotArm("left_arm", 1, kinematic_model, n);
    victor_arms[1] = new RobotArm("right_arm", 2, kinematic_model, n);
  }

  void callback(vive_msgs::ViveSystem msg) {
    for (auto &victor_arm : victor_arms)
    {
      victor_arm->control(msg);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");

  DualArmTeleop dual_arm_teleop_node;

  ros::spin();

  ros::shutdown();
  return 0;
}