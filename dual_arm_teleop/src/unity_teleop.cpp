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
    ros::Publisher pub_display_robot_state;

    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    RobotArm* victor_arms[2];

public:
    DualArmTeleop()
        {
            // sub_vive = n.subscribe<vive_msgs::ViveSystem>("vive", 10, &DualArmTeleop::callback, this);
            // pub_display_robot_state = n.advertise<moveit_msgs::DisplayRobotState>("display_robot_state", 1);

            // Initialize kinematic model
            robot_model_loader::RobotModelLoader robot_model_load("robot_description");

            kinematic_model = robot_model_load.getModel();
            kinematic_state = std::make_shared<robot_state::RobotState>(kinematic_model);

            // Initialize each arm
            victor_arms[LEFT_IND] = new RobotArm("left_arm", 1, kinematic_model, kinematic_state, n);
            victor_arms[RIGHT_IND] = new RobotArm("right_arm", 2, kinematic_model, kinematic_state, n);

            sub_right = n.subscribe<geometry_msgs::PoseStamped>("target_pose/right_ee", 10, &DualArmTeleop::callbackRight, this);
        }

    void callbackRight(geometry_msgs::PoseStamped target_pose)
        {
            auto joint_positions = victor_arms[RIGHT_IND]->IK(target_pose);
            victor_arms[RIGHT_IND]->publishArmCommand(joint_positions);
        }

    void callback(vive_msgs::ViveSystem msg) {
        for (auto &victor_arm : victor_arms)
        {
            victor_arm->control(msg);
        }

        moveit_msgs::DisplayRobotState display_robot_state;
        robot_state::robotStateToRobotStateMsg(*kinematic_state, display_robot_state.state);
        pub_display_robot_state.publish(display_robot_state);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dual_arm_teleop_node");

    DualArmTeleop dual_arm_teleop_node;

    ros::spin();

    ros::shutdown();
    return 0;
}