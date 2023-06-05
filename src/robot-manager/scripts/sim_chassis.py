#! /usr/bin/python3

import rospy
from mobile_platform_msgs.msg import Chassis, BMS
import sys


rospy.init_node('simChassisNode')

chassisPub = rospy.Publisher('/chassis', Chassis, queue_size=1)

rate = rospy.Rate(50)

msg = Chassis()

if len(sys.argv) > 2:
    raise Exception("invalid argument!!")

msg.bms.battery_soc_percentage = 50

if len(sys.argv) == 2:
    if sys.argv[1] == 's':
        print('send vcu stop, prepare to upgrade!')
        msg.vcu.emergency_button = msg.vcu.ON
        msg.driving_mode = msg.VCU_EMERGENCY_STOP
    else:
        msg.bms.battery_soc_percentage = int(sys.argv[1])
        msg.driving_mode = msg.IPC_AUTO
else:
    msg.driving_mode = msg.IPC_AUTO

msg.bms.charger_connected = False

while not rospy.is_shutdown():
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'odom_raw'
    
    chassisPub.publish(msg)

    rate.sleep()
