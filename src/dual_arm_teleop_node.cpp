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

// TF
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

struct victor_arm
{
  victor_arm() : enabled(false), initialized(false) , ee_start_pose(Eigen::Affine3d::Identity()) {}

  bool enabled;
  bool initialized;

  int assigned_controller_index;
  int assigned_controller_id;
  Eigen::Affine3d ee_start_pose;
  Eigen::Affine3d controller_start_pose;

  // State publishers
  ros::Publisher pub_arm;
  ros::Publisher pub_gripper;

  // Kinematics
  std::string joint_model_group_name;
  robot_state::RobotStatePtr kinematic_state;
  robot_state::JointModelGroup* joint_model_group;
  std::vector<std::string> joint_names;
};

class DualArmTeleop
{
  public:
    DualArmTeleop()
    {
      controller_transform <<
              -1, 0, 0, 0,
              0, -1, 0, 0,
              0, 0, -1, 0,
              0, 0, 0, 1;

      sub = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);

      // Initialize victor kinematic model
      robot_model_loader::RobotModelLoader robot_model_load("robot_description");
      kinematic_model = robot_model_load.getModel();

      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

      // Initialize victor arms
      victor_arms[0].joint_model_group_name = "left_arm";
      victor_arms[1].joint_model_group_name = "right_arm";

      for (int arm = 0; arm < 2; ++arm)
      {
        victor_arms[arm].joint_model_group = kinematic_model->getJointModelGroup(victor_arms[arm].joint_model_group_name);
        victor_arms[arm].joint_names = victor_arms[arm].joint_model_group->getVariableNames();
        victor_arms[arm].kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

        // Enforce joint limits
        victor_arms[arm].kinematic_state->setToDefaultValues();
        victor_arms[arm].kinematic_state->enforceBounds();

        victor_arms[arm].ee_start_pose.translation() = victor_arms[arm].kinematic_state->getGlobalLinkTransform("victor_" + victor_arms[arm].joint_model_group_name + "_link_7").translation();
        //victor_arms[arm].ee_start_pose.linear() = victor_arms[arm].kinematic_state->getGlobalLinkTransform("victor_root").linear();
        Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
        rot.setFromTwoVectors(Eigen::Vector3d(1, 0, 0), Eigen::Vector3d(0, 1, 0));
        victor_arms[arm].ee_start_pose.rotate(rot);
        rot.setFromTwoVectors(Eigen::Vector3d(0, 1, 0), Eigen::Vector3d(0, 0, 1));
        victor_arms[arm].ee_start_pose.rotate(rot);

        victor_arms[arm].pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(victor_arms[arm].joint_model_group_name + "/motion_command", 10);
        victor_arms[arm].pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(victor_arms[arm].joint_model_group_name + "/gripper_command", 10);
      }
    }

    void callback(vive_msgs::ViveSystem msg)
    {
      for (int arm = 0; arm < 2; ++arm)
      {
        victor_arms[arm].enabled = false;
      }

      // Gather information about controllers
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
        if (!victor_arms[arm].enabled)
        {
          victor_arms[arm].initialized = false;
          continue;
        }

        vive_msgs::Controller msg_controller = msg.controllers[victor_arms[arm].assigned_controller_index];

        // Reset frame when button is pressed
        if (msg_controller.joystick.buttons[0] == 2 || !victor_arms[arm].initialized)
        {
          // Controller frame
          victor_arms[arm].controller_start_pose = getTrackedPose(msg_controller.posestamped.pose);
          victor_arms[arm].initialized = true;
        }

        // A(base-reset) = B(reset-controller) * C(base-controller)
        Eigen::Affine3d relative_pose = victor_arms[arm].ee_start_pose * getTrackedPose(msg_controller.posestamped.pose).inverse() * victor_arms[arm].controller_start_pose;

        // Compute IK solution
        victor_hardware_interface::MotionCommand msg_out_motion;

        std::size_t attempts = 10;
        double timeout = 0.01;
        bool found_ik = victor_arms[arm].kinematic_state->setFromIK(victor_arms[arm].joint_model_group, relative_pose, attempts, timeout);

        if (found_ik)
        {
          std::vector<double> joint_values;
          victor_arms[arm].kinematic_state->copyJointGroupPositions(victor_arms[arm].joint_model_group, joint_values);

          msg_out_motion.joint_position.joint_1 = joint_values[0];
          msg_out_motion.joint_position.joint_2 = joint_values[1];
          msg_out_motion.joint_position.joint_3 = joint_values[2];
          msg_out_motion.joint_position.joint_4 = joint_values[3];
          msg_out_motion.joint_position.joint_5 = joint_values[4];
          msg_out_motion.joint_position.joint_6 = joint_values[5];
          msg_out_motion.joint_position.joint_7 = joint_values[6];

          ROS_INFO("Found IK solution");
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
        
        // Publish state messages
        victor_arms[arm].pub_arm.publish(msg_out_motion);
        victor_arms[arm].pub_gripper.publish(msg_out_gripper);

        // Display rviz poses
        tf::Transform transform1;
        tf::poseEigenToTF(relative_pose, transform1);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group_name + "/relative_pose"));

        tf::Transform transform2;
        tf::poseEigenToTF(victor_arms[arm].controller_start_pose, transform2);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group_name + "/reset_pose"));

        tf::Transform transform3;
        tf::poseEigenToTF(getTrackedPose(msg_controller.posestamped.pose), transform3);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group_name + "/global_pose"));
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
    ros::Subscriber sub;
    tf::TransformBroadcaster tf_broadcaster;
    
    robot_model::RobotModelPtr kinematic_model;

    victor_arm victor_arms[2];

    Eigen::Matrix4d controller_transform;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");
  
  DualArmTeleop dual_arm_teleop;

  ros::spin();

  ros::shutdown();
  return 0;
}