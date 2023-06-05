#! /usr/bin/python3

from turtle import pos
import rospy
import std_msgs.msg
import geometry_msgs.msg
from mobile_platform_msgs.srv import *

rospy.init_node("test_charging")

charging_save = rospy.ServiceProxy("/ui/charging/save", ChargingSave)

pose_list = [[-7.0,-20.0], [2.0, 2.0]]
chargingPoints = ChargingSaveRequest()

for e in pose_list:
    pose = geometry_msgs.msg.Pose()
    pose.position.x = e[0]
    pose.position.y = e[1]
    pose.position.z = 0
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1.0

    chargingPoints.poses.append(pose)

print("call charging points save!!")
res = charging_save.call(chargingPoints)
print(f"success {res.message}, {res.status}!!")


