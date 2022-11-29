#!/usr/bin/env python

from readline import write_history_file
from traceback import print_tb
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import numpy as np 
import sys

import time
#robot 1 sensor front

r1_dsf={}


#robot 1 sensor back

r1_dsb={}

#robot 1 sensor front

r1_dsf2=[]


#robot 1 sensor back

r1_dsb2=[]

def robot_1_sensorfront(msg):

    global r1_dsf,r1_dsf2

    #rospy.loginfo("I heard %s",data.data) 

    r1_dsf={}
    
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsf[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsf[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)
    r1_dsf2=list(msg.ranges)

def robot_1_sensorback(msg):

    global r1_dsb,r1_dsb2

    r1_dsb={}
    #print("s")
    for ind in range(len(msg.ranges)):

        if not msg.ranges[ ind ] in r1_dsf:
            r1_dsb[ msg.ranges[ ind ] ]= [ind * msg.angle_increment - msg.angle_min]
        else:
            r1_dsb[ msg.ranges[ ind ] ].append(ind * msg.angle_increment - msg.angle_min)
    r1_dsb2=list(msg.ranges)

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

def alante(minimo,pub,move):

    move.linear.x=0.1
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=0

    pub.publish(move)
    return move.linear.x

def magnitude(vector): 
    return np.sqrt(sum(pow(element, 2) for element in vector))

r1_positions=[]
def robot_1_Odometry(msg):
    global r1_positions
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    #if len(r1_positions)>=1:
    #    desplacamiento+=magnitude(np.array(r1_positions[-1])-np.array([x,y]))
    r1_positions.append([x,y])

def dist_recorrida(p,previous_p):
    #print(p,previous_p)
    x,y=p
    previous_x,previous_y=previous_p
    #print
    d_increment = np.sqrt((x - previous_x) * (x - previous_x) +
                          (y - previous_y) * (y - previous_y))
    #print(d_increment)
    return d_increment

def funcion_fitness(distancia_minimas,vels,angulos,tiempo):
    global dist_minima_front_l,dist_minima_front_r,r1_positions,dist_minima_back

    #print(robot,tiempo)
    #tiempo_n=tiempo
    #if tiempo>50:
    #    tiempo_n=50
    #tiempo_n=cambio_de_tiempo(tiempo_n)
    vels=np.array(vels)
    angulos=np.array(angulos)
    
    desplacamiento=np.sum(np.array([ dist_recorrida( r1_positions[i],r1_positions[i-1] ) for i in range(1, len(r1_positions) ) ]) )
    print("tiempo,desplazamiento,distanciaminima")
    print(tiempo,desplacamiento,np.mean(distancia_minimas))
    print("mena angulos,sum angulos,maximode angulos,mean vels,min vels")
    if len(angulos)>0:
        print(np.mean(angulos),np.sum(angulos),np.max(angulos),np.mean(vels),np.min(vels))
    print("mean vels,min vels,max vels")
    print(np.mean(vels),np.min(vels),np.max(vels))
    print("minimo frontal,minimo derecha,minimo izquierda")
    print(np.mean(dist_minima_front_l),np.mean(dist_minima_front_r))

    #distancias=np.mean(dist_minima_front_l)+np.mean(dist_minima_front_r)+2.5*desplacamiento
    #distancias=np.mean(distancia_minim_front)*15+np.mean(dist_minima_front_l)*5+np.mean(dist_minima_front_r)*5+100*desplacamiento+30*np.mean(distancia_minimas)
    #distancias+=np.mean(dist_minima_back)*30
    #vel=3*np.mean(vels)+(3/np.mean(angulos))+len(angulos)/np.sum(angulos)
    #vel=100*np.mean(vels)+(100/(np.mean(angulos)*10))+(100/(np.sum(angulos)))+(tiempo/(np.sum(angulos)*10))

    #a=distancias + vel
    #lo que hace es que el robot se mueva muchisimo mas lento para no colisionar

    #0.5*np.mean(distancia_minimas)+(np.mean(vels)/3)+(3/np.mean(angulos))+(time/np.sum(angulos))+desplacamiento+np.mean(distancia_minim_front)+(len(angulos)/np.sum(angulos))
   


def print_valores_fitness():
    print("minimo_frontal")

def get_giro(ang,dist,pub,move):
    
    move.angular.z= -1/(dist*ang)
    pub.publish(move)
    return move.angular.x,move.angular.z


rospy.init_node('node_name')
sub=rospy.Subscriber("/robot1/laser_front/scan", LaserScan, robot_1_sensorfront)
sub2=rospy.Subscriber("/robot1/laser_back/scan", LaserScan, robot_1_sensorback)
sub3=rospy.Subscriber("/robot1/laser_back/odom", Odometry, robot_1_Odometry) 
move_r1=rospy.Publisher("/robot1/cmd_vel", Twist,queue_size=1)
move=Twist()

rate=rospy.Rate(10)

cero_move(move_r1,move)
dist_minima=[]
dist_minima_front=[]
global dist_minima_front_l,dist_minima_front_r,dist_minima_back
dist_minima_front_l=[]
dist_minima_front_r=[]
dist_minima_back=[]
angulos=[]
inicial = time.time()

vels=[]

while not rospy.is_shutdown():
    print("probando si funciona")
    if len(r1_dsf2)>0:
        #print(r1_dsf)
        angulo,minimo=angle_min(r1_dsf)

        X = np.array([ r1_dsf2+r1_dsb2 ]).T
        dist_minima.append(np.min(X))
        dist_minima_front.append(np.min(np.array(r1_dsf2)))
        dist_minima_front_l.append(np.min(np.array(r1_dsf2[:len(r1_dsf2)//2 ])))
        dist_minima_front_r.append(np.min(np.array(r1_dsf2[len(r1_dsf2)//2:] )))
        dist_minima_back.append(np.min(np.array(r1_dsb2)))
        ahora=time.time()
        if minimo>0:
            if minimo>0.5:
                vels.append(alante(minimo,move_r1,move))
            else:
                a=get_giro( angulo,minimo,move_r1,move)
                vels.append(a[0])
                angulos.append(a[1])

            funcion_fitness(dist_minima,vels,angulos,ahora-inicial) 

        else:
            cero_move(move_r1,move)

        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
    rate.sleep()
