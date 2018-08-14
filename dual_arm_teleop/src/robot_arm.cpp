#include "robot_arm.h"

RobotArm::RobotArm(std::string joint_model_group_name, robot_model::RobotModelPtr kinematic_model, robot_state::RobotStatePtr kinematic_state, ros::NodeHandle n)
{
//  this->kinematic_model = kinematic_model;
//  this->kinematic_state = kinematic_state;

  // Initialize kinematic model
  robot_model_loader::RobotModelLoader robot_model_load("robot_description");

  kinematic_model = robot_model_load.getModel();
  kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

  joint_model_group = kinematic_model->getJointModelGroup(joint_model_group_name);

  last_valid_pose = kinematic_state->getGlobalLinkTransform("victor_" + joint_model_group->getName() + "_link_7");
  ee_start_translation = last_valid_pose.translation();
  joint_position_measured.resize(7);

  pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(joint_model_group->getName() + "/motion_command", 10);
  pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(joint_model_group->getName() + "/gripper_command", 10);
  sub_arm_status = n.subscribe(joint_model_group->getName() + "/motion_status", 10, &RobotArm::updateMeasuredState, this);
}

void RobotArm::control(vive_msgs::ViveSystem msg)
{
  // Skip control if no controller is assigned
  if (!enabled)
  {
    initialized = false;
    return;
  }

  // Skip control if deactivated
  if (!activated) {
    return;
  }

  vive_msgs::Controller msg_controller = msg.controllers[assigned_controller_index];

  // The controller pose in victor frame
  //Eigen::Affine3d controller_pose = viveToVictorTranslation(pointMsgToEigen(msg_controller.posestamped.pose.position));

  // Update this arm's activation status
  if (msg_controller.joystick.buttons[2] == 2) {
    if (!trackpad_pressed) {
      trackpad_pressed = true;
      activated = !activated;
    }
  } else {
    trackpad_pressed = false;
  }

  // Reset frame when button is pressed
  if (msg_controller.joystick.buttons[0] == 2 || !initialized)
  {
    // Controller frame
    controller_start_translation = pointMsgToEigen(msg_controller.posestamped.pose.position);

    initialized = true;
  }

  // Reset orientation
  if (msg_controller.joystick.buttons[1] == 2) {
    controller_start_rotation = quatMsgToEigen(msg_controller.posestamped.pose.orientation);
  }

  // Calculate the relative pose
  Eigen::Affine3d relative_pose = Eigen::Affine3d::Identity();

  // Base
  relative_pose.translate(ee_start_translation);

  // Translation
  Eigen::Vector3d translation(0, 0, 0);
  translation += viveToVictorTranslation(pointMsgToEigen(msg_controller.posestamped.pose.position));
  translation -= viveToVictorTranslation(controller_start_translation);
  translation *= (msg_controller.joystick.axes[1] + 1.5) * .5; // motion scaling

  relative_pose.translate(translation);

  // Rotation
  relative_pose.rotate(viveToVictorRotation(quatMsgToEigen(msg_controller.posestamped.pose.orientation)));
  relative_pose.rotate(viveToVictorRotation(controller_start_rotation).inverse());

  // Generate IK solutions
  const kinematics::KinematicsBaseConstPtr& solver = joint_model_group->getSolverInstance();
  assert(solver.get());

  Eigen::Affine3d solverTrobot = Eigen::Affine3d::Identity();
  kinematic_state->setToIKSolverFrame(solverTrobot, solver);

  // Convert to solver frame
  Eigen::Affine3d pt_solver = solverTrobot * relative_pose;

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

    last_valid_pose = relative_pose;
    ee_start_translation = relative_pose.translation();

    err_msg.text = "";
  }
  else
  {
    err_msg.text = "Did not find IK solution";
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

  controller_start_translation = pointMsgToEigen(msg_controller.posestamped.pose.position);

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

//  // Display rviz poses
//  tf::Transform transform1;
//  tf::poseEigenToTF(relative_pose, transform1);
//  tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "victor_root", joint_model_group->getName() + "/relative_pose"));
//
//  tf::Transform transform2;
//  tf::poseEigenToTF(translationAndRotationToAffine(controller_start_translation, controller_start_rotation), transform2);
//  tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "victor_root", joint_model_group->getName() + "/reset_pose"));
//
//  tf::Transform transform3;
//  tf::poseEigenToTF(poseMsgToEigen(msg_controller.posestamped.pose), transform3);
//  tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "victor_root", joint_model_group->getName() + "/global_pose"));
//
//  tf::Transform transform4;
//  tf::poseEigenToTF(last_valid_pose, transform4);
//  tf_broadcaster.sendTransform(tf::StampedTransform(transform4, ros::Time::now(), "victor_root", joint_model_group->getName() + "/last_valid_pose"));
//
//  tf::Transform transform5;
//  tf::poseEigenToTF(translationAndRotationToAffine(ee_start_translation, Eigen::Quaterniond::Identity()), transform5);
//  tf_broadcaster.sendTransform(tf::StampedTransform(transform5, ros::Time::now(), "victor_root", joint_model_group->getName() + "/ee_start_translation"));
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

Eigen::Vector3d RobotArm::viveToVictorTranslation(Eigen::Vector3d vive)
{
  return Eigen::Vector3d(
      -vive[2],
      -vive[0],
      vive[1]
  );
}

Eigen::Quaterniond RobotArm::viveToVictorRotation(Eigen::Quaterniond vive)
{
  return Eigen::Quaterniond(
      vive.x(),
      vive.w(),
      -vive.y(),
      -vive.z()
  );
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
  return Eigen::Quaterniond(
      quaternion.x,
      quaternion.y,
      quaternion.z,
      quaternion.w
  );
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