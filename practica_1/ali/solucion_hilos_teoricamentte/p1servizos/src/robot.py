#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import numpy as np 
import sys


#robot 1 sensor front

r_dsf={}


#robot 1 sensor back

r_dsb={}

# suscripciones sensores

sub_sen={}

# publicacion move robot

robots_move={}

# action contro from robot

robots_control={}



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

def robot_1_sensorfront(robot,msg):

    global r_dsf

    #rospy.loginfo("I heard %s",data.data) 

    r1_dsf={}
    
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsf[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsf[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)

    r_dsf[robot]=r1_dsf

def robot_1_sensorback(msg):

    global r1_dsb

    r1_dsb={}
    #print("s")
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsb:
            r1_dsb[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsb[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)

def control_robot(msg):
    global robots_control
    print("conectamos")
    actions_new=msg.data.split(",")

    for i in range(0,len(actions_new)//2):
        #print(actions_new[i])
        if actions_new[i] in robots_control:
            robots_control[actions_new[i]]=actions_new[len(actions_new)//2+i+1]
    print(robots_control)

def get_giro(ang,dist,pub,move):
    
    move.angular.z= -1/(dist*ang)
    pub.publish(move)


def sub_pub(robot):


    global sub_sen,robots_move

    topic_robot=["/",robot]
    sub=rospy.Subscriber( "/".join([topic_robot,"laser_front/scan"]) , LaserScan, robot_1_sensorfront)
    sub2=rospy.Subscriber("/".join([topic_robot,"laser_back/scan"]), LaserScan, robot_1_sensorback)
    
    sub_sen[robot]=[sub,sub2]

    move_r1=rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=1)

    robots_move[robot]=move_r1

    
rospy.init_node('node_name')
sub=rospy.Subscriber("/control", String, control_robot)

while len(robots_control)==0:
    a=1

for i in robots_control.keys():
    sub_pub(i)

move=Twist()

rate=rospy.Rate(10)



while not rospy.is_shutdown():
    print("probando si funciona")
    

    rate.sleep()
