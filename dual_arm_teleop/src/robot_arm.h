#ifndef DUAL_ARM_TELEOP_ROBOT_ARM_H
#define DUAL_ARM_TELEOP_ROBOT_ARM_H

#include <ros/ros.h>

#include <queue>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
#include <vive_msgs/ViveSystem.h>

// Victor
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>
#include <victor_hardware_interface/Robotiq3FingerStatus.h>

// TF
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

// rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

class RobotArm
{
private:
  // Kinematics
  robot_model::RobotModelPtr kinematic_model;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;

  Eigen::Vector3d ee_start_translation;
  Eigen::Affine3d ee_last_valid_pose;
  Eigen::Affine3d controller_last_pose;
  Eigen::Affine3d controller_frame_diff_pose;

  std::vector<double> joint_position_measured;

  // Topic publishers/subscribers
  ros::Publisher pub_arm;
  ros::Publisher pub_gripper;
  ros::Subscriber sub_arm_status;

  tf::TransformBroadcaster tf_broadcaster;

  // Methods
  void updateMeasuredState(victor_hardware_interface::MotionStatus msg);
  bool armWithinDelta(std::vector<double> joint_position_commanded);

  static Eigen::Affine3d translationAndRotationToAffine(Eigen::Vector3d translation, Eigen::Quaterniond rotation);
  static Eigen::Vector3d viveToVictorTranslation(Eigen::Vector3d vive);
  static Eigen::Quaterniond viveToVictorRotation(Eigen::Quaterniond vive);
  static Eigen::Vector3d pointMsgToEigen(geometry_msgs::Point point);
  static Eigen::Quaterniond quatMsgToEigen(geometry_msgs::Quaternion quaternion);
  static Eigen::Affine3d poseMsgToEigen(geometry_msgs::Pose pose);
  static std::vector<double> jvqToVector(victor_hardware_interface::JointValueQuantity jvq);

public:
  bool enabled = false; // arm control toggle by keybind
  bool initialized = false; // true if *_start_[translation/rotation] has been initialized
  bool trackpad_pressed = false;

  int controller_hand;

  // Methods
  RobotArm(std::string joint_model_group_name, int controller_hand, robot_model::RobotModelPtr kinematic_model, robot_state::RobotStatePtr kinematic_state, ros::NodeHandle n);

  void control(vive_msgs::ViveSystem msg);
};

struct SeedDistanceFunctor
{
  using Solution = std::vector<double>;
  const Solution seed;
  SeedDistanceFunctor(Solution _seed) : seed(std::move(_seed)) {}
  static double distance(const Solution& a, const Solution& b)
  {
    assert(a.size() == b.size());
    double d = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
    {
      //d += fabs(a[i]-b[i]);
      d += pow(a[i]-b[i], 2);
    }
    return d;
  }

  // NB: priority_queue is a max-heap structure, so less() should actually return >
  // "highest priority"
  bool operator()(const Solution& a, const Solution& b) const
  {
    return distance(seed, a) > distance(seed, b);
  }
};

#endif //DUAL_ARM_TELEOP_ROBOT_ARM_H
