#include <cmath>

#include "ros/ros.h"

// Messages
#include <victor_hardware_interface/MotionCommand.h>
#include <victor_hardware_interface/MotionStatus.h>

// Keep track of latest real robot state
victor_hardware_interface::MotionStatus robotStatusLeft;
victor_hardware_interface::MotionStatus robotStatusRight;

// Publishers
ros::Publisher pub_motion_left;
ros::Publisher pub_motion_right;

void updateRobotStatusLeft(victor_hardware_interface::MotionStatus msg) {
    robotStatusLeft = msg;
}

void updateRobotStatusRight(victor_hardware_interface::MotionStatus msg) {
    robotStatusRight = msg;
}

void callback(victor_hardware_interface::MotionCommand msg, int arm) {
    geometry_msgs::Pose cartesian_pose = msg.cartesian_pose;
    geometry_msgs::Pose measured_cartesian_pose;
    if (arm == 0) {
        measured_cartesian_pose = robotStatusLeft.measured_cartesian_pose;
    } else {
        measured_cartesian_pose = robotStatusRight.measured_cartesian_pose;
    }

    // Compare pose request with measured pose
    double distance = sqrt(pow(cartesian_pose.position.x - measured_cartesian_pose.position.x, 2) + pow(cartesian_pose.position.y - measured_cartesian_pose.position.y, 2) + pow(cartesian_pose.position.z - measured_cartesian_pose.position.z, 2));
    if (distance < .5) {
        std::cout << arm << " within limits" << std::endl;

        if (arm == 0) {
            pub_motion_left.publish(msg);
        } else {
            pub_motion_right.publish(msg);
        }
    } else {
        std::cout << arm << " moved too quickly, not publishing" << std::endl;
    }
}

void callbackLeft(victor_hardware_interface::MotionCommand msg) {
    callback(msg, 0);
}

void callbackRight(victor_hardware_interface::MotionCommand msg) {
    callback(msg, 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dual_arm_teleop_publisher_node");

    ros::NodeHandle n;
    ros::Subscriber sub_motion_command_left = n.subscribe("unchecked_victor/left_arm/motion_command", 10, callbackLeft);
    ros::Subscriber sub_motion_command_right = n.subscribe("unchecked_victor/right_arm/motion_command", 10, callbackRight);
    ros::Subscriber sub_motion_status_left = n.subscribe("left_arm/motion_status", 10, updateRobotStatusLeft);
    ros::Subscriber sub_motion_status_right = n.subscribe("right_arm/motion_status", 10, updateRobotStatusRight);
    pub_motion_left = n.advertise<victor_hardware_interface::MotionCommand>("left_arm/motion_command", 10);
    pub_motion_right = n.advertise<victor_hardware_interface::MotionCommand>("right_arm/motion_command", 10);

    ros::spin();

    ros::shutdown();
    return 0;
}
