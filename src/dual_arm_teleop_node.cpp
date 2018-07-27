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

#include <Eigen/Dense>

class DualArmTeleop
{
  public:
    DualArmTeleop()
    {
      pub_left_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>("left_arm/gripper_command", 10);
      pub_right_gripper = n.advertise<victor_hardware_interface::Robotiq3FingerCommand>("right_arm/gripper_command", 10);
      sub = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);

      robot_model_loader::RobotModelLoader robot_model_load_temp("robot_description");
      // robot_model_load = &robot_model_load_temp;
      kinematic_model = robot_model_load_temp.getModel();

      joint_model_group = kinematic_model->getJointModelGroup("left_arm");

      joint_names = joint_model_group->getVariableNames();

      // kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
      kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

      ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());

      // Enforce joint limits
      kinematic_state->setToDefaultValues();
      kinematic_state->enforceBounds();
    }

    void callback(vive_msgs::ViveSystem msg)
    {
      assert(msg.controllers.size() == 2);
      for (int controller = 0; controller < msg.controllers.size(); ++controller)
      //for (const vive_msgs::Controller& controller : msg.controller)
      {
        if (msg.controllers[controller].joystick.axes.size() == 0)
        {
          continue;
        }
        if (!initialized)
        {
          initialPoses[controller] = Eigen::Affine3d::Identity();
          initialPoses[controller].translation() = Eigen::Vector3d(
            msg.controllers[controller].posestamped.pose.position.x,
            msg.controllers[controller].posestamped.pose.position.y,
            msg.controllers[controller].posestamped.pose.position.z
          );
          initialPoses[controller].rotate(Eigen::Quaterniond(
            msg.controllers[controller].posestamped.pose.orientation.x,
            msg.controllers[controller].posestamped.pose.orientation.y,
            msg.controllers[controller].posestamped.pose.orientation.z,
            msg.controllers[controller].posestamped.pose.orientation.w
          ));
          initialized = true;
        }

        // Joint state control
        Eigen::Affine3d controller_pose = Eigen::Affine3d::Identity();
        controller_pose.translation() = initialPoses[controller].translation() -
        Eigen::Vector3d(
          msg.controllers[controller].posestamped.pose.position.x,
          msg.controllers[controller].posestamped.pose.position.y,
          msg.controllers[controller].posestamped.pose.position.z
        );
        controller_pose.rotate(Eigen::Quaterniond(
          msg.controllers[controller].posestamped.pose.orientation.x,
          msg.controllers[controller].posestamped.pose.orientation.y,
          msg.controllers[controller].posestamped.pose.orientation.z,
          msg.controllers[controller].posestamped.pose.orientation.w
        ));
        controller_pose.rotate(initialPoses[controller].rotation().inverse());

        // Print end-effector pose. Remember that this is in the model frame
        ROS_INFO_STREAM("Translation: \n" << controller_pose.translation() << "\n");
        ROS_INFO_STREAM("Rotation: \n" << controller_pose.rotation() << "\n");

        // Compute IK solution
        std::size_t attempts = 10;
        double timeout = 0.1;
        bool found_ik = kinematic_state->setFromIK(joint_model_group, controller_pose, attempts, timeout);

        // Now, we can print out the IK solution (if found):
        if (found_ik)
        {
          std::vector<double> joint_values;
          kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
          for (std::size_t i = 0; i < joint_names.size(); ++i)
          {
            ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
          }
        }
        else
        {
          ROS_INFO("Did not find IK solution");
        }

        // Gripper control

        assert(msg.controllers[controller].joystick.axes.size() == 3);

        victor_hardware_interface::Robotiq3FingerActuatorCommand scissor;
        scissor.speed = 1.0;
        scissor.force = 1.0;
        scissor.position = .5 * (1 - msg.controllers[controller].joystick.axes[0]);

        victor_hardware_interface::Robotiq3FingerActuatorCommand finger_a;
        finger_a.speed = 1.0;
        finger_a.force = 1.0;
        finger_a.position = msg.controllers[controller].joystick.axes[2];

        victor_hardware_interface::Robotiq3FingerActuatorCommand finger_b;
        finger_b.speed = 1.0;
        finger_b.force = 1.0;
        finger_b.position = msg.controllers[controller].joystick.axes[2];

        victor_hardware_interface::Robotiq3FingerActuatorCommand finger_c;
        finger_c.speed = 1.0;
        finger_c.force = 1.0;
        finger_c.position = msg.controllers[controller].joystick.axes[2];

        char info[60];
        sprintf(info, "Controller: %i Scissor: %f Finger: %f", controller, scissor.position, finger_a.position);
        ROS_INFO("%s", info);

        victor_hardware_interface::Robotiq3FingerCommand msg_out_gripper;
        msg_out_gripper.scissor_command = scissor;
        msg_out_gripper.finger_a_command = finger_a;
        msg_out_gripper.finger_b_command = finger_b;
        msg_out_gripper.finger_c_command = finger_c;

        if (controller == 0)
        {
          pub_left_gripper.publish(msg_out_gripper);
        }
        else
        {
          pub_right_gripper.publish(msg_out_gripper);
        }
      }
    }
    
  private:
    ros::NodeHandle n;
    ros::Publisher pub_left_gripper;
    ros::Publisher pub_right_gripper;
    ros::Subscriber sub;

    bool initialized = false;
    Eigen::Affine3d initialPoses[2];

    // robot_model_loader::RobotModelLoader* robot_model_load;
    robot_model::RobotModelPtr kinematic_model;

    robot_state::RobotStatePtr kinematic_state;
    /*const*/ robot_state::JointModelGroup* joint_model_group;

    /*const*/ std::vector<std::string> joint_names;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dual_arm_teleop_node");
  
  DualArmTeleop dual_arm_teleop;

  ros::spin();

  ros::shutdown();
  return 0;
}