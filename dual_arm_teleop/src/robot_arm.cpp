#include "robot_arm.h"

///
/// \param joint_model_group_name Name of joint model group to be controlled
/// \param controller_hand Hand role (0 = invalid, 1 = left, 2 = right)
/// \param kinematic_model Reference to global kinematic model
/// \param n Reference to global node handle
RobotArm::RobotArm(std::string joint_model_group_name, int controller_hand, robot_model::RobotModelPtr kinematic_model, robot_state::RobotStatePtr kinematic_state, ros::NodeHandle n)
{
  this->controller_hand = controller_hand;
  this->kinematic_model = kinematic_model;
  this->kinematic_state = kinematic_state;

  joint_model_group = kinematic_model->getJointModelGroup(joint_model_group_name);

  kinematic_state->setToDefaultValues();

  ee_last_valid_pose = kinematic_state->getGlobalLinkTransform("victor_" + joint_model_group->getName() + "_link_7");
  joint_position_measured.resize(7);

  pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(joint_model_group->getName() + "/motion_command", 10);
  pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(joint_model_group->getName() + "/gripper_command", 10);
  sub_arm_status = n.subscribe(joint_model_group->getName() + "/motion_status", 10, &RobotArm::updateMeasuredState, this);
}

void RobotArm::control(vive_msgs::ViveSystem msg)
{
  // If controller hand is invalid, stop
  if (controller_hand != 1 && controller_hand != 2) return;

  // Find the controller index who's hand we've been assigned
  int assigned_controller_index = -1;
  for (int i = 0; i < msg.controllers.size(); ++i)
  {
    if (msg.controllers[i].id == controller_hand)
    {
      assigned_controller_index = i;
    }
  }

  // If there's no match, don't continue
  if (assigned_controller_index == -1) return;

  // A reference to the assigned controller
  vive_msgs::Controller msg_controller = msg.controllers[assigned_controller_index];

  // Toggle activation status
  if (msg_controller.joystick.buttons[2] == 2 && !trackpad_pressed) {
    enabled = !enabled;
  }
  trackpad_pressed = (msg_controller.joystick.buttons[2] == 2);

  // Skip control if not enabled
  if (!enabled) return;

  Eigen::Affine3d controller_pose = poseMsgToEigen(msg_controller.posestamped.pose);

  // Store reset pose
  if (msg_controller.joystick.buttons[1] == 2 || !initialized)
  {
    controller_reset_pose = controller_pose;
    ee_reset_pose = kinematic_state->getGlobalLinkTransform("victor_" + joint_model_group->getName() + "_link_7");

    initialized = true;
  }

  // Pose representing controller delta between last and current
  Eigen::Affine3d controller_delta_pose = controller_reset_pose.inverse() * controller_pose;

  // Pose representing desired end effector pose
  Eigen::Affine3d ee_target_pose = ee_reset_pose * controller_delta_pose;

  // Generate IK solutions
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
  assert(solver.get());

  Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
  kinematic_state->setToIKSolverFrame(solverTrobot, solver);

  // Convert to solver frame
  Eigen::Affine3d pt_solver = solverTrobot * ee_target_pose;

  std::vector<geometry_msgs::Pose> target_poses;
  geometry_msgs::Pose pose;
  Eigen::Quaterniond q(pt_solver.linear());
  pose.position.x = pt_solver.translation().x();
  pose.position.y = pt_solver.translation().y();
  pose.position.z = pt_solver.translation().z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  target_poses.push_back(pose);

  std::vector<double> seed = joint_position_measured;
  std::vector<std::vector<double>> solutions;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;
  options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;

  solver->getPositionIK(target_poses, seed, solutions, result, options);

  // Pick the solution that matches closest to the measured joint state
  if (!solutions.empty()) {
    SeedDistanceFunctor functor(seed);
    std::priority_queue<std::vector<double>, std::vector<std::vector<double>>, SeedDistanceFunctor> slnQueue(solutions.begin(), solutions.end(), functor);
    kinematic_state->setJointGroupPositions(joint_model_group, slnQueue.top());

    ee_last_valid_pose = ee_target_pose;
  }

  std::cerr << "Got " << solutions.size() << " solutions for " << joint_model_group->getName() << std::endl;

  // Arm control
  victor_hardware_interface::MotionCommand msg_out_arm;
  msg_out_arm.control_mode.mode = 2;

  std::vector<double> joint_values;
  kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

  msg_out_arm.joint_position.joint_1 = joint_values[0];
  msg_out_arm.joint_position.joint_2 = joint_values[1];
  msg_out_arm.joint_position.joint_3 = joint_values[2];
  msg_out_arm.joint_position.joint_4 = joint_values[3];
  msg_out_arm.joint_position.joint_5 = joint_values[4];
  msg_out_arm.joint_position.joint_6 = joint_values[5];
  msg_out_arm.joint_position.joint_7 = joint_values[6];

  // Gripper control
  victor_hardware_interface::Robotiq3FingerCommand msg_out_gripper;

  victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
  scissor.speed = 1.0;
  scissor.force = 1.0;
  scissor.position = 1;

  victor_hardware_interface::Robotiq3FingerActuatorCommand finger_a;
  finger_a.speed = 1.0;
  finger_a.force = 1.0;
  finger_a.position = msg_controller.joystick.axes[2];

  victor_hardware_interface::Robotiq3FingerActuatorCommand finger_b;
  finger_b.speed = 1.0;
  finger_b.force = 1.0;
  finger_b.position = msg_controller.joystick.axes[2];

  victor_hardware_interface::Robotiq3FingerActuatorCommand finger_c;
  finger_c.speed = 1.0;
  finger_c.force = 1.0;
  finger_c.position = msg_controller.joystick.axes[2];

  msg_out_gripper.scissor_command = scissor;
  msg_out_gripper.finger_a_command = finger_a;
  msg_out_gripper.finger_b_command = finger_b;
  msg_out_gripper.finger_c_command = finger_c;

  // Publish state messages
  if (armWithinDelta(jvqToVector(msg_out_arm.joint_position))) {
    pub_arm.publish(msg_out_arm);
  }

  pub_gripper.publish(msg_out_gripper);

  // Display rviz poses
  tf::Transform tf_controller_global;
  tf::poseEigenToTF(controller_pose, tf_controller_global);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_global, ros::Time::now(), "victor_root", joint_model_group->getName() + "/controller_global"));

  tf::Transform tf_controller_reset;
  tf::poseEigenToTF(controller_reset_pose, tf_controller_reset);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_reset, ros::Time::now(), "victor_root", joint_model_group->getName() + "/controller_reset"));

  tf::Transform tf_ee_last_valid;
  tf::poseEigenToTF(ee_last_valid_pose, tf_ee_last_valid);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_last_valid, ros::Time::now(), "victor_root", joint_model_group->getName() + "/ee_last_valid"));

  tf::Transform tf_ee_target;
  tf::poseEigenToTF(ee_target_pose, tf_ee_target);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_target, ros::Time::now(), "victor_root", joint_model_group->getName() + "/ee_target"));
}

void RobotArm::updateMeasuredState(victor_hardware_interface::MotionStatus msg) {
  joint_position_measured = jvqToVector(msg.measured_joint_position);
}

bool RobotArm::armWithinDelta(std::vector<double> joint_position_commanded)
{
  assert(joint_position_commanded.size() == joint_position_measured.size());

  double distance = 0;

  for (int i = 0; i < joint_position_commanded.size(); ++i) {
    distance += pow(joint_position_commanded[i] - joint_position_measured[i], 2);
  }

  distance = sqrt(distance);

  std::cout << "Joint space error for " << joint_model_group->getName() << ": " << distance << std::endl;

  return distance < .7;
}

Eigen::Affine3d RobotArm::translationAndRotationToAffine(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
{
  Eigen::Affine3d out = Eigen::Affine3d::Identity();
  out.translate(translation);
  out.rotate(rotation);
  return out;
}

Eigen::Vector3d RobotArm::pointMsgToEigen(geometry_msgs::Point point)
{
  return Eigen::Vector3d(
      point.x,
      point.y,
      point.z
  );
}

Eigen::Quaterniond RobotArm::quatMsgToEigen(geometry_msgs::Quaternion quaternion)
{
  Eigen::Quaterniond q(
      quaternion.w,
      quaternion.x,
      quaternion.y,
      quaternion.z
  );

  // Rotate 180 degrees to correct controller orientation
  Eigen::Quaterniond rot(0, 0, 1, 0);

  return rot * q;
}

Eigen::Affine3d RobotArm::poseMsgToEigen(geometry_msgs::Pose pose)
{
  return translationAndRotationToAffine(pointMsgToEigen(pose.position), quatMsgToEigen(pose.orientation));
}

std::vector<double> RobotArm::jvqToVector(victor_hardware_interface::JointValueQuantity jvq)
{
  std::vector<double> v{jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4,
                        jvq.joint_5, jvq.joint_6, jvq.joint_7};
  return v;
};