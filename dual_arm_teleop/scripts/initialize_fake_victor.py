#!/usr/bin/env python

from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
import rospy

# ros publishers do not immediately connect to the topic after the constructer is called.
# Without a sleep param the message is lost
# Sleeping for some small time allows the publisher to connect
# A more guaranteed but also more complicated method would be to periodly republish the message until the desired state is reached
MAGIC_SLEEP_PARAM = 0.2



def init_left_arm():
    vu.set_control_mode(ControlMode.JOINT_IMPEDANCE, "left_arm", stiffness=vu.Stiffness.STIFF)
    msg = MotionCommand()
    msg.control_mode.mode = ControlMode.JOINT_IMPEDANCE;
    msg.joint_position.joint_1 = 1.6;
    msg.joint_position.joint_2 = 0.5;
    msg.joint_position.joint_3 = -1;
    msg.joint_position.joint_4 = -1;
    msg.joint_position.joint_5 = 0;
    msg.joint_position.joint_6 = 1;
    msg.joint_position.joint_7 = 0;

    pub = rospy.Publisher("left_arm/motion_command", MotionCommand, queue_size=10)
    rospy.sleep(MAGIC_SLEEP_PARAM)
    pub.publish(msg)


def init_right_arm():
    vu.set_control_mode(ControlMode.JOINT_IMPEDANCE, "right_arm", stiffness=vu.Stiffness.STIFF)
    msg = MotionCommand()
    msg.control_mode.mode = ControlMode.JOINT_IMPEDANCE;
    msg.joint_position.joint_1 = 1.6;
    msg.joint_position.joint_2 = -0.5;
    msg.joint_position.joint_3 = 1;
    msg.joint_position.joint_4 = 1;
    msg.joint_position.joint_5 = 0;
    msg.joint_position.joint_6 = -1;
    msg.joint_position.joint_7 = 0;
    pub = rospy.Publisher("right_arm/motion_command", MotionCommand, queue_size=10)
    rospy.sleep(MAGIC_SLEEP_PARAM)
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("initialize_fake_victor")

    init_left_arm()
    init_right_arm()




    
