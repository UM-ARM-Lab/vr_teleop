#!/usr/bin/env python

from victor_hardware_interface import victor_utils as vu
from victor_hardware_interface.msg import *
import rospy

# ros publishers do not immediately connect to the topic after the constructer is called.
# Without a sleep param the message is lost
# Sleeping for some small time allows the publisher to connect
# A more guaranteed but also more complicated method would be to periodly republish the message until the desired state is reached
MAGIC_SLEEP_PARAM = 1.0


left = [-0.292, 0.002, 0.466, -1.053, -0.016, 1.198, 0.825]
right = [-0.226, 0.209, 0.095, -0.889, -0.203, 1.247, -2.122]


def init_left_arm():
    cm = ControlMode.JOINT_IMPEDANCE
    # vu.set_control_mode(cm, "left_arm", stiffness=vu.Stiffness.MEDIUM, vel=1.0, accel=1.0)
    vu.set_control_mode(cm, "left_arm", stiffness=vu.Stiffness.STIFF, vel=1.0, accel=1.0)


def init_right_arm():
    cm = ControlMode.JOINT_IMPEDANCE
    # vu.set_control_mode(cm, "right_arm", stiffness=vu.Stiffness.MEDIUM, vel=1.0, accel=1.0)
    vu.set_control_mode(cm, "right_arm", stiffness=vu.Stiffness.STIFF, vel=1.0, accel=1.0)
    # rospy.sleep(MAGIC_SLEEP_PARAM)
    # pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("initialize_fake_victor")
    
    init_left_arm()
    init_right_arm()




    
