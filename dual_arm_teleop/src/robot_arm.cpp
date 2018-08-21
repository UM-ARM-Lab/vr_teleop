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

  ee_last_valid_pose = kinematic_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());
  joint_position_measured.resize(7);

  pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(joint_model_group->getName() + "/motion_command", 10);
  pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(joint_model_group->getName() + "/gripper_command", 10);
  sub_arm_status = n.subscribe(joint_model_group->getName() + "/motion_status", 10, &RobotArm::callbackArmStatusUpdate, this);

  pub_controller_mesh = n.advertise<visualization_msgs::Marker>(joint_model_group->getName() + "/controller_mesh", 10);
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

  Eigen::Affine3d controller_pose;
  tf::poseMsgToEigen(msg_controller.posestamped.pose, controller_pose);

  // Rotate to correct controller orientation
  Eigen::AngleAxisd rotX(M_PI/2, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotY(M_PI/2, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rotZ(M_PI/2, Eigen::Vector3d::UnitZ());
  controller_pose = controller_pose;// * rotY * rotZ;

  // Store reset pose
  if (msg_controller.joystick.buttons[1] == 2 || !initialized)
  {
    controller_reset_pose = controller_pose;
    ee_reset_pose = ee_last_valid_pose;

    initialized = true;
  }

  // Position lock
  if (msg_controller.joystick.buttons[0] == 2)
  {
    controller_reset_pose.translation() = controller_pose.translation();
    ee_reset_pose.translation() = ee_last_valid_pose.translation();
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
  tf::poseEigenToMsg(pt_solver, pose);
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
  msg_out_arm.joint_position = victor_utils::vectorToJvq(joint_values);

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
  if (armWithinDelta(victor_utils::jvqToVector(msg_out_arm.joint_position))) {
    pub_arm.publish(msg_out_arm);
  }

  pub_gripper.publish(msg_out_gripper);

  // Display rviz poses
  tf::Transform tf_controller_global;
  tf::poseEigenToTF(controller_pose, tf_controller_global);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_global, ros::Time::now(), kinematic_model->getRootLinkName(), joint_model_group->getName() + "/controller_global"));

  tf::Transform tf_controller_reset;
  tf::poseEigenToTF(controller_reset_pose, tf_controller_reset);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_controller_reset, ros::Time::now(), kinematic_model->getRootLinkName(), joint_model_group->getName() + "/controller_reset"));

  tf::Transform tf_ee_last_valid;
  tf::poseEigenToTF(ee_last_valid_pose, tf_ee_last_valid);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_last_valid, ros::Time::now(), kinematic_model->getRootLinkName(), joint_model_group->getName() + "/ee_last_valid"));

  tf::Transform tf_ee_target;
  tf::poseEigenToTF(ee_target_pose, tf_ee_target);
  tf_broadcaster.sendTransform(tf::StampedTransform(tf_ee_target, ros::Time::now(), kinematic_model->getRootLinkName(), joint_model_group->getName() + "/ee_target"));

  // Display controller mesh
  visualization_msgs::Marker msg_out_controller_mesh;

  msg_out_controller_mesh.header.stamp = ros::Time::now();
  msg_out_controller_mesh.header.frame_id = kinematic_model->getRootLinkName();
  msg_out_controller_mesh.ns = "vive";
  msg_out_controller_mesh.type = 10;

  tf::poseEigenToMsg(ee_target_pose, msg_out_controller_mesh.pose);

  msg_out_controller_mesh.scale.x = 1;
  msg_out_controller_mesh.scale.y = 1;
  msg_out_controller_mesh.scale.z = 1;

  std_msgs::ColorRGBA color;
  color.r = 1;
  color.g = 1;
  color.b = 1;
  color.a = 1;
  msg_out_controller_mesh.color = color;

  msg_out_controller_mesh.text = "controller_mesh";
  msg_out_controller_mesh.mesh_resource = "file:///home/andriym/.steam/steam/steamapps/common/SteamVR/resources/rendermodels/vr_controller_vive_1_5/body.obj";
  msg_out_controller_mesh.mesh_use_embedded_materials = 0;

  pub_controller_mesh.publish(msg_out_controller_mesh);

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

void RobotArm::callbackArmStatusUpdate(victor_hardware_interface::MotionStatus msg) {
  joint_position_measured = victor_utils::jvqToVector(msg.measured_joint_position);
}