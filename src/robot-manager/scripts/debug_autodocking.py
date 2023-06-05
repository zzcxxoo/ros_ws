#! /usr/bin/python3
import rospy
from mobile_platform_msgs.srv import HomeToDock, HomeToDockRequest
from geometry_msgs.msg import Pose

if __name__ == '__main__':

    rospy.init_node('debug_home_to_dock')
    client = rospy.ServiceProxy("/auto_dock/home_to_dock", HomeToDock)

    data = "-2.7307854369282722 11.071639396250248 0 0 0 1.0".split(' ')
    data = [float(x) for x in data]

    print(data)
    p = Pose()
    p.position.x = data[0]
    p.position.y = data[1]
    p.position.z = 0
    p.orientation.x = data[2]
    p.orientation.y = data[3]
    p.orientation.z = data[4]
    p.orientation.w = data[5]
    
    req = HomeToDockRequest()
    req.start.pose.pose = p
    res = client.call(req)
    print(f'pose size get: \n{len(res.path.poses)}')
