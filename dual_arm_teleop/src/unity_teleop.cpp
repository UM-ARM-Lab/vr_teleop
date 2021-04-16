#include "unity_teleop.hpp"

DualArmTeleop::DualArmTeleop() {
  // Initialize kinematic model
  robot_model_loader::RobotModelLoader robot_model_load("robot_description");

  kinematic_model = robot_model_load.getModel();
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

  // Initialize each arm
  victor_arms[LEFT_IND] = new RobotArm("left_arm", 1, kinematic_model, kinematic_state, n);
  victor_arms[RIGHT_IND] = new RobotArm("right_arm", 2, kinematic_model, kinematic_state, n);

  sub_right =
      n.subscribe<geometry_msgs::PoseStamped>("target_pose/right_flange", 10, &DualArmTeleop::callbackRight, this);
  sub_left = n.subscribe<geometry_msgs::PoseStamped>("target_pose/left_flange", 10, &DualArmTeleop::callbackLeft, this);
  sub_joy_right = n.subscribe<sensor_msgs::Joy>("right_gripper/target", 10, &DualArmTeleop::callbackRightJoy, this);
  sub_joy_left = n.subscribe<sensor_msgs::Joy>("left_gripper/target", 10, &DualArmTeleop::callbackLeftJoy, this);

  pub_joint_state = n.advertise<sensor_msgs::JointState>("target_joint_states", 1);

  srv_enable = n.advertiseService("set_enabled", &DualArmTeleop::setEnabledCallback, this);
}

void DualArmTeleop::callbackArm(geometry_msgs::PoseStamped target_pose, int arm_ind) {
  auto joint_positions = victor_arms[arm_ind]->IK(target_pose);
  sensor_msgs::JointState joint_state;
  robot_state::robotStateToJointStateMsg(*kinematic_state, joint_state);
  pub_joint_state.publish(joint_state);

  if (teleop_enabled) {
    victor_arms[arm_ind]->publishArmCommand(joint_positions);
  }
}

void DualArmTeleop::callbackJoy(sensor_msgs::Joy joy, int arm_ind) {
  if (teleop_enabled) {
    victor_arms[arm_ind]->publishGripperCommand(joy.axes[0]);
  }
}

bool DualArmTeleop::setEnabledCallback(dual_arm_teleop::SetEnabled::Request &req,
                                       dual_arm_teleop::SetEnabled::Request &res) {
  teleop_enabled = req.enabled;
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dual_arm_teleop_node");

  DualArmTeleop dual_arm_teleop_node;

  ros::spin();

  ros::shutdown();
  return 0;
}
