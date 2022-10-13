#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


import numpy as np 
import sys


#robot 1 sensor front

r1_dsf={}


#robot 1 sensor back

r1_dsb={}



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
    #print("s")
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsb[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsb[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)

def angle_min(dis_dist):
    if len(dis_dist.keys())>0:
        minimo=np.min(np.array(dis_dist.keys()))
        angulo=dis_dist[minimo][0]
        return angulo,minimo
    else:
        return 0,0

def cero_move(pub,move):

    move.linear.x=0
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=0

    pub.publish(move)

def alante(pub,move):

    move.linear.x=0.1
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=0

    pub.publish(move)

def get_giro(ang,dist,pub,move):
    
    move.angular.z= -1/(dist*ang)
    pub.publish(move)



rospy.init_node('node_name')
sub=rospy.Subscriber("/robot1/laser_front/scan", LaserScan, robot_1_sensorfront)
sub2=rospy.Subscriber("/robot1/laser_back/scan", LaserScan, robot_1_sensorback)

move_r1=rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=1)
move=Twist()

rate=rospy.Rate(10)

cero_move(move_r1,move)

while not rospy.is_shutdown():
    print("probando si funciona")
    
    print(r1_dsf)
    angulo,minimo=angle_min(r1_dsf)
    if minimo>0:
        if minimo>0.5:
            alante(move_r1,move)
        else:
            get_giro( angulo,minimo,move_r1,move)
    else:
        cero_move(move_r1,move)

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()
    rate.sleep()
