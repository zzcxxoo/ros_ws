#! /usr/bin/python3
import rospy
import json
from mobile_platform_msgs.srv import Agent, AgentRequest
from mobile_platform_msgs.srv import Pursuit, PursuitRequest
import argparse
import time

parser = argparse.ArgumentParser(prog="start-cleaning", description="a script for running cleaning task!!")

parser.add_argument('--type', type=str, help='type of cmd: create/modify/delete/enable/disable')
parser.add_argument('-s', '--start_time', type=str, help='please specify start time')
parser.add_argument('-m', '--map_id', type=str, help='please specify selected map name')
parser.add_argument('-z', '--zone_id', type=str, help="please specify zone_id")
parser.add_argument('-d', '--repeat_dates', dest='repeat_dates', nargs='*', help='date: 1~7')
parser.add_argument('-t', '--times', type=int, help='time of task')
parser.add_argument('-p', '--path', type=str, help='list of path: edge/coverage/both')
parser.add_argument('-r', '--rubtype', type=str, help='list of rubtype: small/median/big/huge')

args = vars(parser.parse_args())

# print(args)

def cleaning_start():
    global args

    cmd_type = args['type']
    map_id = args['map_id']
    zone_id = args['zone_id']
    times = args['times']
    path = args['path']
    rubtype = args['rubtype']

    data = dict()
    data["type"] = cmd_type
    data["task_id"] = "timing-" + str(int(time.time()))
    data["start_time"] = args["start_time"]
    data["repeat_dates"] = [int(x) for x in args["repeat_dates"]]

    plan = dict(map_id=map_id)
    plan["plan_id"] = "plan"
    plan["zone_id"] = zone_id
    plan["times"] = times
    plan["mode"] = path
    plan["rubtype"] = rubtype

    data["plan"] = plan

    req = AgentRequest()
    
    req.data = json.dumps(data)
    print(f'cmd:------------------------ \n{req.data}')

    client = rospy.ServiceProxy("/ui/cleaning/timing_task", Agent)
    res = client.call(req)
    print(f'res:------------------------ \n{res}')


if __name__ == '__main__':

    rospy.init_node('callservice')
    cleaning_start()