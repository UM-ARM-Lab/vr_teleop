#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros

def talker():
    buf = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buf)
    pub_roof = rospy.Publisher('kinect_pose_roof', PoseStamped, queue_size=10)
    pub_victor_head = rospy.Publisher('kinect_pose_victor_head', PoseStamped, queue_size=10)
    
    rospy.init_node('kinect_pose_publisher', anonymous=True)
    rate = rospy.Rate(1) # 10hz
    
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        # rospy.loginfo(hello_str)

        try_publish_frame('kinect2_roof_ir_optical_frame', pub_roof, buf)
        try_publish_frame('kinect2_victor_head_ir_optical_frame', pub_victor_head, buf)

        rate.sleep()

def try_publish_frame(frame_name, publisher, buf):
    try:
        trans = buf.lookup_transform('victor_root', frame_name, rospy.Time())
    except (tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        return;
    publisher.publish(trans_to_pose(trans))

def trans_to_pose(trans):
    pose = PoseStamped()
    pose.pose.position = trans.transform.translation
    pose.pose.orientation = trans.transform.rotation
    return pose

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
