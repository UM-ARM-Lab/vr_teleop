#include "robot_arm.h"

#define DELTA 0.7

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

    gripper_transform = getGripperTransform();
}

Eigen::Affine3d RobotArm::getGripperTransform()
{
    Eigen::AngleAxisd rotx(M_PI, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd rotz(-M_PI*1/4, Eigen::Vector3d::UnitZ());
    Eigen::Affine3d transz(Eigen::Translation3d(0, 0, 0.15));
    return transz*rotz*rotx;
}

bool RobotArm::getThisArmMsg(vive_msgs::ViveSystem msg, vive_msgs::Controller &controller)
{
    if(controller_hand != 1 && controller_hand != 2)
    {
        return false;
    }

    for(auto ctrl: msg.controllers)
    {
        if(ctrl.id == controller_hand)
        {
            controller = ctrl;
            return true;
        }
    }
    return false;
}

void RobotArm::control(vive_msgs::ViveSystem msg)
{
    vive_msgs::Controller msg_controller;
    if(!getThisArmMsg(msg, msg_controller))
    {
        return;
    }
    updateEnabledStatus(msg_controller);

    Eigen::Affine3d controller_pose;
    tf::poseMsgToEigen(msg_controller.posestamped.pose, controller_pose);
    // Rotate to correct controller orientation
    controller_pose = controller_pose * gripper_transform;

    handleReset(msg_controller, controller_pose);


    // Pose representing controller delta between last and current
    Eigen::Affine3d controller_delta_pose = controller_reset_pose.inverse() * controller_pose;

    // Pose representing desired end effector pose
    Eigen::Affine3d ee_target_pose = ee_reset_pose * controller_delta_pose;

    // Skip control if not enabled
    if (enabled)
    {
        std::vector<double> joint_values = IK(ee_target_pose);
        publishArmCommand(joint_values);
        handleGripperCommand(msg_controller.joystick.axes[2]);

    }

    broadcastPose(controller_pose, "controller_global");
    broadcastPose(controller_reset_pose, "controller_reset");
    broadcastPose(ee_last_valid_pose, "ee_last_valid");
    broadcastPose(ee_target_pose, "ee_target");

    publishControllerMarker(ee_target_pose * gripper_transform);
}

void RobotArm::updateEnabledStatus(vive_msgs::Controller controller_msg)
{
    // Toggle activation status
    if (controller_msg.joystick.buttons[2] == 2 && !trackpad_pressed) {
        enabled = !enabled;
    }
    trackpad_pressed = (controller_msg.joystick.buttons[2] == 2);

}

void RobotArm::handleReset(vive_msgs::Controller controller_msg, Eigen::Affine3d controller_pose)
{
    // Store reset pose
    if (controller_msg.joystick.buttons[1] == 2 || !initialized)
    {
        controller_reset_pose = controller_pose;
        ee_reset_pose = ee_last_valid_pose;

        initialized = true;
    }
}

void RobotArm::handleGripperCommand(double command_position)
{
    if(command_position != prev_gripper_command)
    {
        publishGripperCommand(command_position);
        prev_gripper_command = command_position;
    }
}

std::vector<double> RobotArm::IK(geometry_msgs::PoseStamped ee_target_pose)
{
    Eigen::Affine3d target;
    tf::poseMsgToEigen(ee_target_pose.pose, target);
    return IK(target);
}

std::vector<double> RobotArm::IK(Eigen::Affine3d ee_target_pose)
{
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

        // ee_last_valid_pose = ee_target_pose;
    }

    std::cerr << "Got " << solutions.size() << " solutions for " << joint_model_group->getName() << std::endl;

    // Arm control

    std::vector<double> joint_values;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
    return joint_values;
}

void RobotArm::publishArmCommand(std::vector<double> joint_positions)
{
    victor_hardware_interface::MotionCommand msg_out_arm;
    msg_out_arm.control_mode.mode = 2;

    msg_out_arm.joint_position = victor_utils::vectorToJvq(joint_positions);
    // Publish state messages
    // if (armWithinDelta(victor_utils::jvqToVector(msg_out_arm.joint_position), DELTA)) {
    //     pub_arm.publish(msg_out_arm);
    // }
    pub_arm.publish(msg_out_arm);
}

void RobotArm::publishGripperCommand(double gripper_pos)
{
    // Gripper control
    victor_hardware_interface::Robotiq3FingerCommand msg_out_gripper;

    victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
    scissor.speed = 1.0;
    scissor.force = 1.0;
    scissor.position = 1;

    victor_hardware_interface::Robotiq3FingerActuatorCommand finger_a;
    finger_a.speed = 1.0;
    finger_a.force = 1.0;
    finger_a.position = gripper_pos;

    victor_hardware_interface::Robotiq3FingerActuatorCommand finger_b;
    finger_b.speed = 1.0;
    finger_b.force = 1.0;
    finger_b.position = gripper_pos;

    victor_hardware_interface::Robotiq3FingerActuatorCommand finger_c;
    finger_c.speed = 1.0;
    finger_c.force = 1.0;
    finger_c.position = gripper_pos;

    msg_out_gripper.scissor_command = scissor;
    msg_out_gripper.finger_a_command = finger_a;
    msg_out_gripper.finger_b_command = finger_b;
    msg_out_gripper.finger_c_command = finger_c;

    pub_gripper.publish(msg_out_gripper);

}

void RobotArm::broadcastPose(Eigen::Affine3d pose, std::string name)
{
    tf::Transform tf_pose;
    tf::poseEigenToTF(pose, tf_pose);
    tf_broadcaster.sendTransform(tf::StampedTransform(tf_pose, ros::Time::now(),
                                                      kinematic_model->getRootLinkName(),
                                                      joint_model_group->getName() + "/" + name));
}

void RobotArm::publishControllerMarker(Eigen::Affine3d mesh_pose)
{
    // Display controller mesh
    visualization_msgs::Marker msg_out_controller_mesh;
    msg_out_controller_mesh.header.frame_id = kinematic_model->getRootLinkName();
    msg_out_controller_mesh.header.stamp = ros::Time();
    msg_out_controller_mesh.ns = "vive";
    msg_out_controller_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    tf::poseEigenToMsg(mesh_pose, msg_out_controller_mesh.pose);
    msg_out_controller_mesh.scale.x = 1;
    msg_out_controller_mesh.scale.y = 1;
    msg_out_controller_mesh.scale.z = 1;
    msg_out_controller_mesh.color.r = 1;
    msg_out_controller_mesh.color.g = 1;
    msg_out_controller_mesh.color.b = 1;
    msg_out_controller_mesh.color.a = 1;
    msg_out_controller_mesh.mesh_resource = "package://dual_arm_teleop/meshes/vr_controller_vive_1_5/vr_controller_vive_1_5.obj";
    msg_out_controller_mesh.mesh_use_embedded_materials = 0;

    pub_controller_mesh.publish(msg_out_controller_mesh);

}

bool RobotArm::armWithinDelta(std::vector<double> joint_position_commanded, double delta)
{
    assert(joint_position_commanded.size() == joint_position_measured.size());

    double distance = 0;

    for (int i = 0; i < joint_position_commanded.size(); ++i) {
        distance += pow(joint_position_commanded[i] - joint_position_measured[i], 2);
    }

    distance = sqrt(distance);

    std::cout << "Joint space error for " << joint_model_group->getName() << ": " << distance << std::endl;

    return distance < delta;
}

// Eigen::Affine3d RobotArm::getPalmToFlange()
// {
//     if(!palm_to_flange_calulated)
//     {
//         palm_to_flange = tf_listener.get;
//         palm_to_flange_calculated=true;
//     }
//     return palm_to_flange;
// }

void RobotArm::callbackArmStatusUpdate(victor_hardware_interface::MotionStatus msg) {
    joint_position_measured = victor_utils::jvqToVector(msg.measured_joint_position);
    kinematic_state->setJointGroupPositions(joint_model_group, joint_position_measured);
    ee_last_valid_pose = kinematic_state->getGlobalLinkTransform(joint_model_group->getLinkModelNames().back());    
}
