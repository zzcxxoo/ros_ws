#! /usr/bin/python3
import rospy
import numpy as np
import os
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mobile_platform_msgs.srv import Pursuit, PursuitRequest, PursuitResponse
from mobile_platform_msgs.srv import TargetPoint, TargetPointRequest, TargetPointResponse, HomeToDock, HomeToDockResponse
from mobile_platform_msgs.msg import PurePursuitResult, PurePursuitStatus

def readCSV(fn):
    cnt = 0
    with open(fn, 'r') as f:
        str_list = f.readline().split(' ')
        while len(str_list) > 2:
            cnt += 1
            str_list = f.readline().split(' ')
    return cnt

class PursuitSim:

    def __init__(self) -> None:
        print("------- serve as pure pursuit simulator ------")
        rospy.Service("/pursuit", Pursuit, self.pursuitCB)
        rospy.Service("/target_point_planner", TargetPoint, self.plannerCB)
        rospy.Service("/auto_dock/home_to_dock", HomeToDock, self.dockCB)

        self.res_pub = rospy.Publisher("/pure_pursuit/tracking_result", PurePursuitResult, queue_size=10)
        self.stat_pub = rospy.Publisher("/pure_pursuit/status", PurePursuitStatus, queue_size=1)
        self.tracking_flag = False
        self.path_size = 0
        self.progress = 0
        self.step_rate = 0.3334
        self.status_timer = rospy.Timer(rospy.Duration(1, 0), self.statusCB)

    def plannerCB(self, req):
        print("call planner success!")
        res = TargetPointResponse()
        for i in range(20):
            _p = PoseStamped()
            res.planner_path.poses.append(_p)
        res.message = "ok"
        res.status = 0
        return res
    
    def dockCB(self, req):
        print("sim auto docking")
        res = HomeToDockResponse()
        for i in range(20):
            _p = PoseStamped()
            res.path.poses.append(_p)
        res.message = "ok"
        return res
        
    def pursuitCB(self, req : PursuitRequest):
        res = PursuitResponse()
        # start tracking
        if req.command == 1:
            pn = req.path_name
            if(os.path.exists(pn)):
                self.path_size = readCSV(pn)
                if self.path_size > 5:
                    print(f"start to track : {pn}")
                    self.progress = 0
                    res.message = "ok"
                    res.status = 0
                    self.tracking_flag = True
                else:
                    res.message = f"path size({self.path_size}) is too little"
                    res.status = -1
                    self.tracking_flag = False
            else:
                res.message = f"{pn} doesnt exist!"
                res.status = -1
                self.tracking_flag = False

        elif req.command == 2:
            self.sendResult("Manual_Stop")
            res.message = "stop"
            res.status = 0

        return res

    def statusCB(self, e):
        if self.tracking_flag:
            if self.progress >= 1.0:
                self.sendResult("Success")
                print("Success")
                return

            self.progress += self.step_rate
            bar = "=" * int(self.progress * 10) + ">"
            print(f"\rworking: {bar}", end="")
            s = PurePursuitStatus()
            s.waypoint = int(self.progress * self.path_size)
            s.state = "WORKING"
            self.stat_pub.publish(s)
        else:
            print("idle")
            s = PurePursuitStatus()
            s.state = "IDLE"
            self.stat_pub.publish(s)

    def sendResult(self, res):
        self.tracking_flag = False
        r = PurePursuitResult()
        r.tracking_result = res
        self.res_pub.publish(r)

rospy.init_node("pure_pursuit")
PursuitSim()
rospy.spin()
