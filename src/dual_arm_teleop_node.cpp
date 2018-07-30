/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman*/

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h> 
#include <moveit/robot_state/robot_state.h>

// Vive
#include "vive_msgs/ViveSystem.h"

// Victor
#include "victor_hardware_interface/MotionCommand.h"
#include "victor_hardware_interface/Robotiq3FingerCommand.h"

// Transform broadcaster
#include <tf/transform_broadcaster.h>

// TF to Eigen
#include "tf_conversions/tf_eigen.h"

struct vive_controller
{
  vive_controller() : initialized(false) {}

  bool initialized;

  int id;
  Eigen::Affine3d base_pose;
};

struct victor_arm
{
  victor_arm() : enabled(false), initialized(false) {}

  bool enabled;
  bool initialized;

  int assigned_controller_id;
  Eigen::Affine3d ee_start_pose;
};

class DualArmTeleop
{
  public:
    DualArmTeleop()
    {
      // Initialize ROS message handlers
      pub_left_arm = n.advertise<victor_hardware_interface::MotionCommand>("left_arm/msg_out_motion", 10);
      pub_right_arm = n.advertise<victor_hardware_interface::MotionCommand>("right_arm/msg_out_motion", 10);
      pub_left_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>("left_arm/gripper_command", 10);
      pub_right_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>("right_arm/gripper_command", 10);

      sub = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);

      // Initialize Victor kinematics
      robot_model_loader::RobotModelLoader robot_model_load_temp("robot_description");
      kinematic_model = robot_model_load_temp.getModel();
      joint_model_group = kinematic_model->getJointModelGroup("left_arm");
      joint_names = joint_model_group->getVariableNames();
      kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

      // Enforce joint limits
      kinematic_state->setToDefaultValues();
      kinematic_state->enforceBounds();
    }

    void callback(vive_msgs::ViveSystem msg)
    {
      if (controllers.size() != msg.controllers.size())
      {
        controllers.resize(msg.controllers.size());
      }

      // Gather information about controllers
      std::vector<int> active_IDs;

      for (int controller = 0; controller < msg.controllers.size(); ++controller) {
      {
        if ()
        active_IDs.push_back(msg.controllers[controller].id);
      }

      for (int arm = 0; arm < victor_arms.size(); ++arm) {
      {
        if (!victor_arms[arm].enabled) {
          // Assign the arm a controller
          victor_arms[arm].assigned_controller_id = active_IDs[0]
        }
      }

      // Decide/assign controllers to arms


      // Loop through left/right controller array
      for (int arm = 0; arm < victor_arms.size(); ++arm) {
        victor_arm victor_arm = victor_arms[arm];
        if (!victor_arm.enabled) continue;

        vive_controller controller = controllers[victor_arm.assigned_controller_id];
        vive_msgs::Controller msg_controller = msg.controllers[victor_arm.assigned_controller_id];

        
        // Reset frame when button is pressed
        if (msg_controller.joystick.buttons[0] == 2 || !controllers[arm].initialized) {
          // Controller frame
          controllers[controller].base_pose = controller_transform * getTrackedPose(msg_controller.posestamped.pose);
          controllers[controller].initialized = true;

          victor_arms[arm] = kinematic_state->getGlobalLinkTransform("victor_left_arm_link_7");
        }

        // A(base-reset) = B(reset-controller) * C(base-controller)
        Eigen::Affine3d relative_pose = victor_arm.ee_start_pose * getTrackedPose(controller_transform * msg_controller.posestamped.pose).inverse() * controllers[controller].base_pose;

        tf::Transform transform1;
        tf::poseEigenToTF(relative_pose, transform1);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "victor_root", "vive_left_hand/relative_pose"));

        tf::Transform transform2;
        tf::poseEigenToTF(controllers[controller].base_pose, transform2);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "victor_root", "vive_left_hand/reset_pose"));

        tf::Transform transform3;
        tf::poseEigenToTF(getTrackedPose(msg_controller.posestamped.pose), transform3);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "victor_root", "vive_left_hand/global_pose"));

        // Compute IK solution
        victor_hardware_interface::MotionCommand msg_out_motion;

        std::size_t attempts = 10;
        double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, relative_pose, attempts, timeout);

        if (found_ik)
        {
          std::vector<double> joint_values;
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);

          msg_out_motion.joint_position.joint_1 = joint_values[0];
          msg_out_motion.joint_position.joint_2 = joint_values[1];
          msg_out_motion.joint_position.joint_3 = joint_values[2];
          msg_out_motion.joint_position.joint_4 = joint_values[3];
          msg_out_motion.joint_position.joint_5 = joint_values[4];
          msg_out_motion.joint_position.joint_6 = joint_values[5];
          msg_out_motion.joint_position.joint_7 = joint_values[6];
        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        // Gripper control
        victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
        scissor.speed = 1.0;
        scissor.force = 1.0;
        scissor.position = .5 * (1 - msg_controller.joystick.axes[0]);

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

        victor_hardware_interface::Robotiq3FingerCommand msg_out_gripper;
        msg_out_gripper.scissor_command = scissor;
        msg_out_gripper.finger_a_command = finger_a;
        msg_out_gripper.finger_b_command = finger_b;
        msg_out_gripper.finger_c_command = finger_c;

        if (arm == 0)
        {
          pub_left_arm.publish(msg_out_motion);
          pub_left_gripper.publish(msg_out_gripper);
        }
        else
        {
          pub_right_arm.publish(msg_out_motion);
          pub_right_gripper.publish(msg_out_gripper);
        }
      }
    }
    
    Eigen::Vector3d getTrackedPosition(geometry_msgs::Point point)
    {
      return Eigen::Vector3d(
        point.x,
        point.y,
        point.z
      );
    }

    Eigen::Quaterniond getTrackedRotation(geometry_msgs::Quaternion quaternion)
    {
      return Eigen::Quaterniond(
        quaternion.x,
        quaternion.y,
        quaternion.z,
        quaternion.w
      );
    }

    Eigen::Affine3d getTrackedPose(geometry_msgs::Pose pose)
    {
      Eigen::Affine3d affine = Eigen::Affine3d::Identity();

      affine.translate(getTrackedPosition(pose.position));
      affine.rotate(getTrackedRotation(pose.orientation));

      return affine;
    }
    
  private:
    ros::NodeHandle n;
    ros::Publisher pub_left_gripper;
    ros::Publisher pub_right_gripper;
    ros::Publisher pub_left_arm;
    ros::Subscriber sub;
    tf::TransformBroadcaster tf_broadcaster;

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;
    robot_state::JointModelGroup* joint_model_group;
    std::vector<std::string> joint_names;

    std::vector<vive_controller> controllers;
    std::vector<victor_arm> victor_arms;

    static Eigen::Matrix4d controller_transform(
      1, 0, 0, 0,
      0, -1, 0, 0,
      0, 0, -1, 0,
      0, 0, 0, 1
    );
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");
  
  DualArmTeleop dual_arm_teleop;

  ros::spin();

  ros::shutdown();
  return 0;
}