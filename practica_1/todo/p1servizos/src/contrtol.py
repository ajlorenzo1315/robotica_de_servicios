#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import String

import os
#os.system('rostopic list')
import sys,time

def call_robot(robot_ind,limit):
    for i in range(0,limit):
        os.system("".join(['rosrun p1 robot_comun.py -r ','robot',str(robot_ind), ' -t 10 ']))



import threading

"""
def talker():
    robot=2
    action={}
    for i in range(1,robot+1):
        action["".join(["robot",str(i)])]="Star"
    pub = rospy.Publisher('control', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        mensage=",".join(list(action.keys())+list(action.values()))
        rospy.loginfo(mensage)
        pub.publish(mensage)
        # robot0,Star,robot1,Star
        actions_new=str(input("nueva accion robots,actios "))
        actions_new=actions_new.split(",")
        for i in range(0,len(actions_new)//2):
            print(actions_new[i*2])
            if actions_new[i*2] in action:
                action[actions_new[i*2]]=actions_new[i*2+1]
        rate.sleep()
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
"""


lista_de_datos=[]
def feedback(msg):
    global lista_de_datos
    lista_de_datos.append(msg.data)


def talker():

    rospy.init_node('control')
    pub1=rospy.Publisher("datos_fin", String,queue_size=1)
    sub=rospy.Subscriber("/datos_fin", String, feedback)
    rate = rospy.Rate(10) # 10hz
    ROBOTS = 3
    hilos_todos=[]
    for num_hilo in range(1,ROBOTS+1):
        hilo = threading.Thread(name='hilo%s' %num_hilo, 
                                target=call_robot,args = (num_hilo,3, ))    
        hilo.start()
        hilos_todos.append(hilo)
        #time.sleep(1)
        #hilo.join() 
        print("thread finished...exiting") 

    
    cont=len(lista_de_datos)    
    while not rospy.is_shutdown():

        if len(lista_de_datos)!=cont:
            print(lista_de_datos) 
            cont=len(lista_de_datos)

        #if len(lista_de_datos)>=8:
        #    break

    for hilo in hilos_todos:    

        hilo.join() 
  
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
  