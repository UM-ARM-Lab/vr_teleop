#ifndef DUAL_ARM_TELEOP_ROBOT_ARM_H
#define DUAL_ARM_TELEOP_ROBOT_ARM_H

#include <ros/ros.h>

#include <queue>

// MoveIt!
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
#include <vive_msgs/ViveSystem.h>

// Msgs
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

// Victor
#include <victor_hardware_interface_msgs/MotionCommand.h>
#include <victor_hardware_interface_msgs/MotionStatus.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerCommand.h>
#include <victor_hardware_interface_msgs/Robotiq3FingerStatus.h>

#include <victor_hardware_interface/victor_utils.hpp>

// TF
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

class RobotArm {
 private:
  // Kinematics
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;

  Eigen::Isometry3d palm_to_flange;
  bool palm_to_flange_calculated = false;

  std::vector<double> joint_position_measured;

  // Topic publishers/subscribers
  ros::Publisher pub_arm;
  ros::Publisher pub_gripper;
  ros::Publisher pub_target;
  ros::Publisher pub_is_valid;
  ros::Subscriber sub_arm_status;

  tf::TransformBroadcaster tf_broadcaster;
  tf::TransformListener tf_listener;
  ros::Publisher pub_controller_mesh;

  bool enabled = false;      // arm control toggle by keybind
  bool initialized = false;  // true if *_start_[translation/rotation] has been initialized
  bool trackpad_pressed = false;

  Eigen::Isometry3d gripper_transform;

  double prev_gripper_command = 0;

  int controller_hand;

  // Methods
  bool armWithinDelta(std::vector<double> joint_position_commanded, double delta);
  void callbackArmStatusUpdate(victor_hardware_interface_msgs::MotionStatus msg);

 public:
  // Methods
  RobotArm(std::string joint_model_group_name, int controller_hand, robot_model::RobotModelPtr kinematic_model,
           robot_state::RobotStatePtr kinematic_state, ros::NodeHandle n);

  void handleGripperCommand(double command_position);

  void publishGripperCommand(double gripper_pos);

  void publishArmCommand(std::vector<double> joint_positions);

  std::vector<double> IK(geometry_msgs::PoseStamped ee_target_pose);

  std::vector<double> IK(Eigen::Isometry3d ee_target_pose);
};

struct SeedDistanceFunctor {
  using Solution = std::vector<double>;
  const Solution seed;
  SeedDistanceFunctor(Solution _seed) : seed(std::move(_seed)) {}
  static double distance(const Solution& a, const Solution& b) {
    assert(a.size() == b.size());
    double d = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
      // d += fabs(a[i]-b[i]);
      d += fabs(a[i] - b[i]);
    }
    return d;
  }

  double cost(const Solution& q) const {
    double c = 0.0;
    for (double qi : q) {
      c += fabs(qi);
    }
    return c + distance(seed, q);
  }

  // NB: priority_queue is a max-heap structure, so less() should actually return >
  // "highest priority"
  bool operator()(const Solution& a, const Solution& b) const { return cost(a) > cost(b); }
};

#endif  // DUAL_ARM_TELEOP_ROBOT_ARM_H
