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

std::vector<double> jvqToVector(victor_hardware_interface::JointValueQuantity jvq)
{
    std::vector<double> v{jvq.joint_1, jvq.joint_2, jvq.joint_3, jvq.joint_4,
                          jvq.joint_5, jvq.joint_6, jvq.joint_7};
    return v;
};

void updateRobotStatusLeft(victor_hardware_interface::MotionStatus msg) {
    robotStatusLeft = msg;
}

void updateRobotStatusRight(victor_hardware_interface::MotionStatus msg) {
    robotStatusRight = msg;
}

void callback(victor_hardware_interface::MotionCommand msg, int arm) {
    std::vector<double> joint_positions = jvqToVector(msg.joint_position);
    std::vector<double> measured_joint_positions;
    if (arm == 0) {
        measured_joint_positions = jvqToVector(robotStatusLeft.measured_joint_position);
    } else {
        measured_joint_positions = jvqToVector(robotStatusRight.measured_joint_position);
    }

    double distance = 0;

    for (int i = 0; i < joint_positions.size(); ++i) {
        distance += pow(joint_positions[i] - measured_joint_positions[i], 2);
    }

    distance = sqrt(distance);

    std::cout << "Joint space error (arm " << arm << "): " << distance << std::endl;

    if (distance < .1) {
        //std::cout << arm << " within limits" << std::endl;

        if (arm == 0) {
            pub_motion_left.publish(msg);
        } else {
            pub_motion_right.publish(msg);
        }
    } else {
        //std::cout << arm << " moved too quickly, not publishing" << std::endl;
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
