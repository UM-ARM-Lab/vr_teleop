#! /usr/bin/env python

import arm_or_robots.motion_victor
from dual_arm_teleop.srv import SetEnabled
from sensor_msgs.msg import Joy
import rospy

left = [0.263, 0.15, 0.001, -1.255, 0.118, 0.943, -0.045]


class ScriptedController:
    def __init__(self):
        self.mev = arm_or_robots.motion_victor.MotionEnabledVictor(viewer=False)
        self.enabled_srv = rospy.ServiceProxy("unity_teleop/set_enabled", SetEnabled)
        
        rospy.Subscriber("unity_teleop/left_gripper/target", Joy, self.left_callback);

        self.executing = False


    def left_callback(self, joy_msg):
        if self.executing:
            return
        
        if joy_msg.axes[0] < 0.9:
            return

        self.executing = True
        
        rospy.loginfo("Left Close Triggered")
        rospy.loginfo("Stopping Teleop")
        self.enabled_srv(False)

        rospy.sleep(5)
        self.enabled_srv(True)
        rospy.loginfo("Restarting Teleop")
        self.executing = False

if __name__ == "__main__":
    rospy.init_node("scripted_takeover")
    controller = ScriptedController()
    rospy.spin()
