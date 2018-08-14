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

#include "RobotArm.h"

robot_model::RobotModelPtr kinematic_model;
robot_state::RobotStatePtr kinematic_state;

ros::NodeHandle n;
ros::Subscriber sub_vive;
tf::TransformBroadcaster tf_broadcaster;
ros::Publisher pub_rviz;

RobotArm victor_arms[2] {
  {kinematic_model, kinematic_state, n, "left_arm"},
  {kinematic_model, kinematic_state, n, "right_arm"}
};

void callback(vive_msgs::ViveSystem msg) {
  // Assign controllers to arms
  for (int arm = 0; arm < 2; ++arm)
  {
    victor_arms[arm].enabled = false;
  }

  std::vector<int> unassigned_controller_indices;
  for (int controller = 0; controller < msg.controllers.size(); ++controller)
  {
    int controller_id = msg.controllers[controller].id;

    bool assigned = false;
    for (int arm = 0; arm < 2; ++arm)
    {
      if (victor_arms[arm].assigned_controller_id == controller_id)
      {
        victor_arms[arm].enabled = true;
        assigned = true;
        victor_arms[arm].assigned_controller_index = controller;
      }
    }
    if (!assigned) unassigned_controller_indices.push_back(controller);
  }

  for (int arm = 0; arm < 2; ++arm)
  {
    if (!victor_arms[arm].enabled)
    {
      victor_arms[arm].assigned_controller_index = unassigned_controller_indices.back();
      victor_arms[arm].assigned_controller_id = msg.controllers[victor_arms[arm].assigned_controller_index].id;
      unassigned_controller_indices.pop_back();
      victor_arms[arm].enabled = true;
    }
  }

  for (int arm = 0; arm < 2; ++arm)
  {
    victor_arms[arm].control(msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");

  sub_vive = n.subscribe<vive_msgs::ViveSystem>("vive", 10, callback);
  pub_rviz = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

  // Initialize kinematic model
  robot_model_loader::RobotModelLoader robot_model_load("robot_description");

  kinematic_model = robot_model_load.getModel();
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

  // Enforce joint limits
  kinematic_state->setToDefaultValues();
  kinematic_state->enforceBounds();

  ros::spin();

  ros::shutdown();
  return 0;
}