#include "unity_arm.h"

#include "std_msgs/String.h"

#define DELTA 0.7

///
/// \param joint_model_group_name Name of joint model group to be controlled
/// \param controller_hand Hand role (0 = invalid, 1 = left, 2 = right)
/// \param kinematic_model Reference to global kinematic model
/// \param n Reference to global node handle
RobotArm::RobotArm(std::string joint_model_group_name, int controller_hand, robot_model::RobotModelPtr kinematic_model,
                   robot_state::RobotStatePtr kinematic_state, ros::NodeHandle n) {
  const std::string& name = joint_model_group_name;
  this->kinematic_model = kinematic_model;
  this->kinematic_state = kinematic_state;

  joint_model_group = kinematic_model->getJointModelGroup(joint_model_group_name);

  kinematic_state->setToDefaultValues();

  joint_position_measured.resize(7);
  pub_arm = n.advertise<victor_hardware_interface_msgs::MotionCommand>(name + "/motion_command", 10);
  pub_gripper = n.advertise<victor_hardware_interface_msgs::Robotiq3FingerCommand>(name + "/gripper_command", 10);
  sub_arm_status = n.subscribe(name + "/motion_status", 10, &RobotArm::callbackArmStatusUpdate, this);
  pub_target = n.advertise<sensor_msgs::JointState>(name + "/target", 10);
  pub_is_valid = n.advertise<std_msgs::String>(name + "/is_ik_valid", 10);
}

void RobotArm::handleGripperCommand(double command_position) {
  if (command_position != prev_gripper_command) {
    publishGripperCommand(command_position);
    prev_gripper_command = command_position;
  }
}

std::vector<double> RobotArm::IK(geometry_msgs::PoseStamped ee_target_pose) {
  Eigen::Isometry3d target;
  tf::poseMsgToEigen(ee_target_pose.pose, target);
  return IK(target);
}

std::vector<double> RobotArm::IK(Eigen::Isometry3d ee_target_pose) {
  // Generate IK solutions
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
  assert(solver.get());

  Eigen::Isometry3d solverTrobot = Eigen::Isometry3d::Identity();
  kinematic_state->setToIKSolverFrame(solverTrobot, solver);

  // Convert to solver frame
  Eigen::Isometry3d pt_solver = solverTrobot * ee_target_pose;

  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose pose;
  tf::poseEigenToMsg(pt_solver, pose);
  target_poses.push_back(pose);

  std::vector<double> seed = joint_position_measured;
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;
  options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;

  solver->getPositionIK(target_poses, seed, solutions, result, options);

  // Pick the solution that matches closest to the measured joint state
  if (solutions.empty()) {
    std_msgs::String msg;
    msg.data = "invalid";
    pub_is_valid.publish(msg);
  } else {
    SeedDistanceFunctor functor(seed);
    std::priority_queue<std::vector<double>, std::vector<std::vector<double>>, SeedDistanceFunctor> slnQueue(
        solutions.begin(), solutions.end(), functor);
    kinematic_state->setJointGroupPositions(joint_model_group, slnQueue.top());
  }

  std::cerr << "Got " << solutions.size() << " solutions for " << joint_model_group->getName() << std::endl;

  // Arm control

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
  return joint_values;
}

void RobotArm::publishArmCommand(std::vector<double> joint_positions) {
  victor_hardware_interface_msgs::MotionCommand msg_out_arm;
  msg_out_arm.control_mode.mode = 2;

  msg_out_arm.joint_position = victor_utils::vectorToJvq(joint_positions);
  // Publish state messages
  std_msgs::String msg;
  msg.data = "invalid";

  if (armWithinDelta(victor_utils::jvqToVector(msg_out_arm.joint_position), DELTA)) {
    pub_arm.publish(msg_out_arm);
    msg.data = "valid";
  }
  pub_is_valid.publish(msg);
  // pub_arm.publish(msg_out_arm);
}

void RobotArm::publishGripperCommand(double gripper_pos) {
  // Gripper control
  using namespace victor_hardware_interface_msgs;
  Robotiq3FingerCommand msg_out_gripper;

  Robotiq3FingerActuatorCommand scissor;
  scissor.speed = 1.0;
  scissor.force = 1.0;
  scissor.position = 1;

  Robotiq3FingerActuatorCommand finger_a;
  finger_a.speed = 1.0;
  finger_a.force = 1.0;
  finger_a.position = gripper_pos;

  Robotiq3FingerActuatorCommand finger_b;
  finger_b.speed = 1.0;
  finger_b.force = 1.0;
  finger_b.position = gripper_pos;

  Robotiq3FingerActuatorCommand finger_c;
  finger_c.speed = 1.0;
  finger_c.force = 1.0;
  finger_c.position = gripper_pos;

  msg_out_gripper.scissor_command = scissor;
  msg_out_gripper.finger_a_command = finger_a;
  msg_out_gripper.finger_b_command = finger_b;
  msg_out_gripper.finger_c_command = finger_c;

  pub_gripper.publish(msg_out_gripper);
}

bool RobotArm::armWithinDelta(std::vector<double> joint_position_commanded, double delta) {
  assert(joint_position_commanded.size() == joint_position_measured.size());

  double distance = 0;

  for (int i = 0; i < joint_position_commanded.size(); ++i) {
    distance += pow(joint_position_commanded[i] - joint_position_measured[i], 2);
  }

  distance = sqrt(distance);

  std::cout << "Joint space error for " << joint_model_group->getName() << ": " << distance << std::endl;

  return distance < delta;
}

void RobotArm::callbackArmStatusUpdate(victor_hardware_interface_msgs::MotionStatus msg) {
  joint_position_measured = victor_utils::jvqToVector(msg.measured_joint_position);
  // kinematic_state->setJointGroupPositions(joint_model_group, joint_position_measured);
}
