#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/DisplayRobotState.h>

// Vive
// #include <vive_msgs/ViveSystem.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>

// TF
#include <tf/transform_broadcaster.h>

#include "robot_arm.h"
#define LEFT_IND 0
#define RIGHT_IND 1



class DualArmTeleop
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_right;
    ros::Subscriber sub_joy_right;
    ros::Subscriber sub_left;
    ros::Subscriber sub_joy_left;
    
    ros::Publisher pub_joint_state;

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    bool teleop_enabled=true;

    RobotArm* victor_arms[2];

public:
    DualArmTeleop();

    void callbackArm(geometry_msgs::PoseStamped target_pose, int arm_ind);

    void callbackRight(geometry_msgs::PoseStamped target_pose)
    {
        callbackArm(target_pose, RIGHT_IND);
    }

    void callbackLeft(geometry_msgs::PoseStamped target_pose)
    {
        callbackArm(target_pose, LEFT_IND);
    }

    void callbackRightJoy(sensor_msgs::Joy joy)
    {
        victor_arms[RIGHT_IND]->publishGripperCommand(joy.axes[0]);
    }

    void callbackLeftJoy(sensor_msgs::Joy joy)
    {
        victor_arms[LEFT_IND]->publishGripperCommand(joy.axes[0]);
    }
};
