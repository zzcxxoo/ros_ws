#! /usr/bin/python3

import rospy
import std_msgs.msg
import json
from mobile_platform_msgs.srv import *

jf = "/home/jtcx/9tian_ws/src/robot-manager/test/test_json.json"

# data = None
# with open(jf) as f:
#     data = json.load(f)

# print(json.dumps(data))

# rospy.init_node("test_pub")

# logger_pub = rospy.Publisher("/ui/cleaning_plan/create", std_msgs.msg.String, queue_size=1)

# def pubCB(event):
#     msg = std_msgs.msg.String()
#     msg.data = json.dumps(data)
#     logger_pub.publish(msg)

# rospy.Timer(rospy.Duration(1.0), pubCB, True)

def handleMappingSave (req):

    if len(req.filename) > 0:
        print(req.filename)
    return MappingSaveResponse(0,'ok')


rospy.Service('/ui/mapping/save', MappingSave , handleMappingSave)


a = rospy.ServiceProxy('/ui/mapping/save', MappingSave)

req = MappingSaveRequest()
req.filename = "test"
a.call(req)

rospy.spin()