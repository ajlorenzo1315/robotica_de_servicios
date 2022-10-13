#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

import numpy as np 

"""
header: 
  seq: 96
  stamp: 
    secs: 619
    nsecs: 457000000
  frame_id: "robot1/laser_front"
angle_min: -1.57000005245
angle_max: 1.57000005245
angle_increment: 0.628000020981 = 36º
time_increment: 0.0
scan_time: 0.0
range_min: 0.0799999982119
range_max: 10.0
            D                                                                                               I
ranges: [3.4365234375, 1.569897174835205, 0.9716949462890625, 0.9722259044647217, 1.5735327005386353, 2.286879777908325]

intensities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

neg gira derecha
pos gira izq

"""

#robot 1 sensor front

r1_dsf={}


#robot 1 sensor back

r1_dbf1={}



def robot_1_sensorfront(msg):

    global r1_dsf

    #rospy.loginfo("I heard %s",data.data) 

    r1_dsf={}
    
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsf[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsf[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)


def robot_1_sensorback(msg):

    global r1_dsb

    r1_dsb={}

    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsb[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsb[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)



rospy.init_node('node_name')
rospy.Subscriber("/robot1/laser_front/scan", LaserScan, robot_1_sensorfront)
rospy.Subscriber("/robot1/laser_back/scan", LaserScan, robot_1_sensorback)
rate=rospy.Rate(10)
print("probando si funciona")
print(r1_dbf1)
# spin() simply keeps python from exiting until this node is stopped
#rospy.spin()
rate.sleep()
