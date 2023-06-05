#! /usr/bin/python3
import rospy
import json
from mobile_platform_msgs.srv import Agent, AgentRequest
from mobile_platform_msgs.srv import Pursuit, PursuitRequest
import argparse

parser = argparse.ArgumentParser(prog="start-cleaning", description="a script for running cleaning task!!")

parser.add_argument('--type', type=str, help='type of action')
parser.add_argument('-M', '--map', type=str, help='please specify selected map name')
parser.add_argument('-m', '--mode', type=str, help="please specify mode new/load")
parser.add_argument('-z', '--zone', dest='zones', nargs='+', help='list of zone name')
parser.add_argument('-t', '--times', dest='times', nargs='+', help='list of times')
parser.add_argument('-p', '--path', dest='path', nargs='+', help='list of path: edge/coverage/both')
parser.add_argument('-r', '--rubtype', dest='rubtype', nargs='+', help='list of rubtype: small/median/big/huge')

args = vars(parser.parse_args())

# print(args)

def cleaning_start():
    global args

    action_type = args['type']
    map = args['map']
    mode = args['mode']
    zones = args['zones']
    times = args['times']
    path = args['path']
    rubtype = args['rubtype']

    if len(zones) != len(times) or len(times) != len(path) or len(path) != len(rubtype):
        print('invalid argument!!')
        return

    data = dict()
    data["type"] = action_type
    data["content"] = dict(map_id=map, plan_id="test", mode=mode)
    tasks = list()
    for i in range(len(zones)):
        tmp = dict(zone_id=zones[i], times=int(times[i]), mode=path[i], rubtype=rubtype[i])
        tasks.append(tmp)

    data["content"]["zones"] = tasks

    req = AgentRequest()
    
    req.data = json.dumps(data)
    print(f'cmd:------------------------ \n{req.data}')

    client = rospy.ServiceProxy("/ui/cleaning/control", Agent)
    res = client.call(req)
    print(f'res:------------------------ \n{res}')


if __name__ == '__main__':

    rospy.init_node('callservice')
    cleaning_start()