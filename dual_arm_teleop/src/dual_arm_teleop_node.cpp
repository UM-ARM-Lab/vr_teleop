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
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
#include "vive_msgs/ViveSystem.h"

// Victor
#include "victor_hardware_interface/MotionCommand.h"
#include "victor_hardware_interface/Robotiq3FingerCommand.h"

// TF
#include <tf/transform_broadcaster.h>
#include "tf_conversions/tf_eigen.h"

// rviz
#include <rviz_visual_tools/rviz_visual_tools.h>

struct victor_arm
{
  victor_arm() : activated(false), trackpad_pressed(false), enabled(false), initialized(false), ee_start_translation(Eigen::Vector3d::Identity()) {}

  bool publishing = true;
  bool activated; // true if arm is currently being teleoperated
  bool trackpad_pressed;
  bool enabled; // true if arm has been paired with a controller
  bool initialized; // true if *_start_[translation/rotation] has been initialized

  int assigned_controller_index;
  int assigned_controller_id;
  Eigen::Vector3d ee_start_translation;
  Eigen::Vector3d controller_start_translation;
  Eigen::Quaterniond controller_start_rotation;
  Eigen::Affine3d last_valid_pose;

  visualization_msgs::Marker err_msg;

  // State publishers
  ros::Publisher pub_arm;
  ros::Publisher pub_gripper;

  // Kinematics
  std::string joint_model_group_name;
  robot_state::JointModelGroup* joint_model_group;
  std::vector<std::string> joint_names;
};

class DualArmTeleop
{
  public:
    DualArmTeleop()
    {
      sub = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);

      // Initialize victor kinematic model
      robot_model_loader::RobotModelLoader robot_model_load("robot_description");
      kinematic_model = robot_model_load.getModel();

      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

      kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);
      // Enforce joint limits
      kinematic_state->setToDefaultValues();
      kinematic_state->enforceBounds();

      // Initialize victor arms
      victor_arms[0].joint_model_group_name = "left_arm";
      victor_arms[1].joint_model_group_name = "right_arm";

      //rvt = rviz_visual_tools::RvizVisualTools("victor_root", "err_msg", n);
      //pub_err_msg = n.advertise<visualization_msgs::Marker>("err_msg", 10);

      for (int arm = 0; arm < 2; ++arm) {
        victor_arms[arm].joint_model_group = kinematic_model->getJointModelGroup(
                victor_arms[arm].joint_model_group_name);
        victor_arms[arm].joint_names = victor_arms[arm].joint_model_group->getVariableNames();

        victor_arms[arm].last_valid_pose = kinematic_state->getGlobalLinkTransform(
                "victor_" + victor_arms[arm].joint_model_group_name + "_link_7");

        victor_arms[arm].ee_start_translation = victor_arms[arm].last_valid_pose.translation();

        victor_arms[arm].pub_arm = n.advertise<victor_hardware_interface::MotionCommand>(
                "unchecked_victor/" + victor_arms[arm].joint_model_group_name + "/motion_command", 10);
        victor_arms[arm].pub_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>(
                "unchecked_victor/" + victor_arms[arm].joint_model_group_name + "/gripper_command", 10);

        pub_rviz = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

        victor_arms[arm].err_msg = visualization_msgs::Marker();
        victor_arms[arm].err_msg.header.frame_id = "victor_root";
        victor_arms[arm].err_msg.ns = "my_namespace";
        victor_arms[arm].err_msg.id = arm;
        victor_arms[arm].err_msg.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        victor_arms[arm].err_msg.action = visualization_msgs::Marker::ADD;
        victor_arms[arm].err_msg.lifetime = ros::Duration();
        victor_arms[arm].err_msg.pose.position.x = victor_arms[arm].ee_start_translation[0];
        victor_arms[arm].err_msg.pose.position.y = victor_arms[arm].ee_start_translation[1];
        victor_arms[arm].err_msg.pose.position.z = victor_arms[arm].ee_start_translation[2];
        victor_arms[arm].err_msg.pose.orientation.x = 0.0;
        victor_arms[arm].err_msg.pose.orientation.y = 0.0;
        victor_arms[arm].err_msg.pose.orientation.z = 0.0;
        victor_arms[arm].err_msg.pose.orientation.w = 1.0;
        victor_arms[arm].err_msg.scale.x = 1;
        victor_arms[arm].err_msg.scale.y = 0.1;
        victor_arms[arm].err_msg.scale.z = 0.1;
        victor_arms[arm].err_msg.color.a = 1.0;
        victor_arms[arm].err_msg.color.r = 0.0;
        victor_arms[arm].err_msg.color.g = 1.0;
        victor_arms[arm].err_msg.color.b = 0.0;
        victor_arms[arm].err_msg.text = "This msg is being published for arm " + std::to_string(victor_arms[arm].err_msg.id);
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
        /*if (victor_arms[arm].publishing) {
          std::cout << "Published msg for arm " + std::to_string(arm) << std::endl;
          pub_err_msg.publish(victor_arms[arm].err_msg);
          //victor_arms[arm].publishing = false;
        }*/
        Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
        //rvt.publishText()
        //rvt.publishText(text_pose, "hello");

        // Skip this arm if no controller is assigned to it
        if (!victor_arms[arm].enabled)
        {
          victor_arms[arm].initialized = false;
          continue;
        }

        vive_msgs::Controller msg_controller = msg.controllers[victor_arms[arm].assigned_controller_index];

        // Update this arm's activation status
        if (msg_controller.joystick.buttons[2] == 2) {
          if (!victor_arms[arm].trackpad_pressed) {
            victor_arms[arm].trackpad_pressed = true;
            victor_arms[arm].activated = !victor_arms[arm].activated;
          }
        } else {
          victor_arms[arm].trackpad_pressed = false;
        }
        if (!victor_arms[arm].activated) {
          continue;
        }


        // Reset frame when button is pressed
        if (msg_controller.joystick.buttons[0] == 2 || !victor_arms[arm].initialized)
        {
          //victor_arms[arm].ee_start_pose = victor_arms[arm].last_valid_pose;
          /*Eigen::Transform<double, 3, 2, 0>::LinearMatrixType rot = kinematic_state->getGlobalLinkTransform(
            "victor_" + victor_arms[arm].joint_model_group_name + "_link_7").linear();
          victor_arms[arm].ee_start_rotation = viveToVictorRotation(Eigen::Quaterniond(rot[0], rot[1], rot[2], rot[3]));*/
          /*victor_arms[arm].ee_start_pose = kinematic_state->getGlobalLinkTransform(
            "victor_" + victor_arms[arm].joint_model_group_name + "_link_7");*/

          // Controller frame
          victor_arms[arm].controller_start_translation = getTrackedPosition(msg_controller.posestamped.pose.position);
          //victor_arms[arm].controller_start_rotation = getTrackedRotation(msg_controller.posestamped.pose.orientation);
          //victor_arms[arm].controller_start_rotation = Eigen::Quaterniond::Identity();

          victor_arms[arm].initialized = true;
        }

        // Reset orientation
        if (msg_controller.joystick.buttons[1] == 2) {
          victor_arms[arm].controller_start_rotation = getTrackedRotation(msg_controller.posestamped.pose.orientation);
        }


        //Calculate the relative pose
        Eigen::Affine3d relative_pose = Eigen::Affine3d::Identity();

        //Base
        relative_pose.translate(victor_arms[arm].ee_start_translation);

        //Translation
        Eigen::Vector3d translation(0, 0, 0);
        translation += viveToVictorTranslation(getTrackedPosition(msg_controller.posestamped.pose.position));
        translation -= viveToVictorTranslation(victor_arms[arm].controller_start_translation);
        translation *= (msg_controller.joystick.axes[1] + 1.5) * .5;
        if (translation.norm() < .1)
        {
          relative_pose.translate(translation);
        }

        // Rotation
        relative_pose.rotate(viveToVictorRotation(getTrackedRotation(msg_controller.posestamped.pose.orientation)));
        relative_pose.rotate(viveToVictorRotation(victor_arms[arm].controller_start_rotation).inverse());


        // Compute IK solution
        victor_hardware_interface::MotionCommand msg_out_motion;

        std::size_t attempts = 10;
        double timeout = 0.01;
        bool found_ik = kinematic_state->setFromIK(victor_arms[arm].joint_model_group, relative_pose, attempts, timeout);

        if (found_ik) {
          victor_arms[arm].last_valid_pose = relative_pose;
          victor_arms[arm].ee_start_translation = relative_pose.translation();


          //ROS_INFO("Found IK solution");
          victor_arms[arm].err_msg.text = "";
        }
        else
        {
          relative_pose = victor_arms[arm].last_valid_pose;
          victor_arms[arm].err_msg.text = "Did not find IK solution";
          //ROS_INFO("Did not find IK solution");
        }
        std::vector<double> joint_values;
        kinematic_state->copyJointGroupPositions(victor_arms[arm].joint_model_group, joint_values);

        msg_out_motion.joint_position.joint_1 = joint_values[0];
        msg_out_motion.joint_position.joint_2 = joint_values[1];
        msg_out_motion.joint_position.joint_3 = joint_values[2];
        msg_out_motion.joint_position.joint_4 = joint_values[3];
        msg_out_motion.joint_position.joint_5 = joint_values[4];
        msg_out_motion.joint_position.joint_6 = joint_values[5];
        msg_out_motion.joint_position.joint_7 = joint_values[6];

        victor_arms[arm].controller_start_translation = getTrackedPosition(msg_controller.posestamped.pose.position);

        victor_arms[arm].err_msg.id = arm;
        victor_arms[arm].err_msg.pose.position.x = relative_pose.translation()[0];
        victor_arms[arm].err_msg.pose.position.y = relative_pose.translation()[1];
        victor_arms[arm].err_msg.pose.position.z = relative_pose.translation()[2];
        victor_arms[arm].err_msg.header.stamp = ros::Time();
        //pub_err_msg.publish(victor_arms[arm].err_msg);

        // Gripper control
        victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
        scissor.speed = 1.0;
        scissor.force = 1.0;
        //scissor.position = .5 * (1 - msg_controller.joystick.axes[0]);
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
        tf::poseEigenToTF(translationAndRotationToAffine(victor_arms[arm].controller_start_translation, victor_arms[arm].controller_start_rotation), transform2);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform2, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group_name + "/reset_pose"));

        tf::Transform transform3;
        tf::poseEigenToTF(getTrackedPose(msg_controller.posestamped.pose), transform3);
        tf_broadcaster.sendTransform(tf::StampedTransform(transform3, ros::Time::now(), "victor_root", victor_arms[arm].joint_model_group_name + "/global_pose"));
      }

      moveit_msgs::DisplayRobotState display_robot_state;
      robot_state::robotStateToRobotStateMsg(*kinematic_state, display_robot_state.state);
      pub_rviz.publish(display_robot_state);
    }

    Eigen::Affine3d translationAndRotationToAffine(Eigen::Vector3d translation, Eigen::Quaterniond rotation)
    {
      Eigen::Affine3d out = Eigen::Affine3d::Identity();
      out.translate(translation);
      out.rotate(rotation);
      return out;
    }

    Eigen::Vector3d viveToVictorTranslation(Eigen::Vector3d vive)
    {
      return Eigen::Vector3d(
        -vive[2],
        -vive[0],
        vive[1]
      );
    }

    Eigen::Quaterniond viveToVictorRotation(Eigen::Quaterniond vive)
    {
      return Eigen::Quaterniond(
        vive.x(),
        vive.w(),
        -vive.y(),
        -vive.z()
      );
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

    /*
    Eigen::Affine3d getTrackedPoseController(geometry_msgs::Pose pose) {
      Eigen::Affine3d affine = Eigen::Affine3d::Identity();

      affine.translate(getTrackedPositionController(pose.position));
      affine.rotate(getTrackedRotation(pose.orientation));

      return affine;
    }*/

  private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    tf::TransformBroadcaster tf_broadcaster;
    ros::Publisher pub_rviz;
    //ros::Publisher pub_err_msg;
    //rviz_visual_tools::RvizVisualTools rvt;

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    victor_arm victor_arms[2];
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");
  
  DualArmTeleop dual_arm_teleop;

  ros::spin();

  ros::shutdown();
  return 0;
}