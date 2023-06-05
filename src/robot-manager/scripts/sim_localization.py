#! /usr/bin/python3

import rospy
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mobile_platform_msgs.msg import ScanMatchingStatus
import tf2_ros, tf
from geometry_msgs.msg import TransformStamped

status_pub = None
tf_pub = None

def handler(msg : Odometry):
    global status_pub, tf_pub

    pub.publish(msg)    

    sms = ScanMatchingStatus()
    sms.header.stamp = rospy.Time.now()
    sms.header.frame_id = 'map'
    sms.has_converged = True
    sms.inlier_fraction = 1.0
    sms.matching_error = 0.0
    status_pub.publish(sms)

    tfs = TransformStamped()
    tfs.header.frame_id = "map"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "base_link"
    tfs.transform.translation.x = msg.pose.pose.position.x
    tfs.transform.translation.y = msg.pose.pose.position.y
    tfs.transform.translation.z = msg.pose.pose.position.z
    tfs.transform.rotation.x = msg.pose.pose.orientation.x
    tfs.transform.rotation.y = msg.pose.pose.orientation.y
    tfs.transform.rotation.z = msg.pose.pose.orientation.z
    tfs.transform.rotation.w = msg.pose.pose.orientation.w
    tf_pub.sendTransform(tfs)


rospy.init_node('pub_odom_and_status')
tf_pub = tf2_ros.TransformBroadcaster()

print('-------- sim odom and status --------')
sub_odom = rospy.Subscriber("/odom_basefootprint", Odometry, handler)
pub = rospy.Publisher("/odom", Odometry, queue_size=1)
status_pub = rospy.Publisher("/status", ScanMatchingStatus, queue_size=1)

rospy.spin()
