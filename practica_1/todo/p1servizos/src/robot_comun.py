#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from std_msgs.msg import String


import numpy as np 
# para trabajar con argumentos
import sys, getopt
import time


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

#def info_robot(vel,time,time_inicial,distancias):

def iniciar_nodo(argv):
    
    limit=None
    try:
        opts, args = getopt.getopt(argv,":r:t:")
    except getopt.GetoptError:
        print("Argumento no valido: usar -r (robot name)")
        opts=["-r","robot1"]
    print(opts)
    for opt, arg in opts:
        print(opt)
        if opt in ("-r" "--robot"):
            robot = arg

        elif opt in ("-t" "--time"):
            limit = int(arg)

        else:
            raise Exception("Flag no valida")

    rospy.init_node(robot)
    print(robot)
    topic_robot="".join(["/",robot])
    sub=rospy.Subscriber( "/".join([topic_robot,"laser_front/scan"]) , LaserScan, robot_1_sensorfront)
    sub2=rospy.Subscriber("/".join([topic_robot,"laser_back/scan"]), LaserScan, robot_1_sensorback)
        
    move_r1=rospy.Publisher("/".join([topic_robot,"cmd_vel"]), Twist,queue_size=1)


    move=Twist()

    rate=rospy.Rate(10)

    cero_move(move_r1,move)
    datos=rospy.Publisher("/datos_fin", String,queue_size=1)

    inicial = time.time()
    if limit!=None:
        limit = inicial + limit
    while not rospy.is_shutdown():
        #print("probando si funciona")
        
        #print(r1_dsf)
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

        inicial = time.time()
        if limit!=None:
            if inicial>=limit:
                cero_move(move_r1,move)
                datos.publish(robot)
                break


if __name__ == '__main__':
    try:

        iniciar_nodo(sys.argv[1:])

    except rospy.ROSInterruptException:
        pass