#!/usr/bin/env python

from turtle import position
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import String

import os
#os.system('rostopic list')
import sys,time

def call_robot(robot_ind,time_eje,positions):
   

        os.system("".join(['rosrun p1 robot_comun.py -r ','robot',str(robot_ind), ' -t ',str(time_eje)]))
        #rosservice call /gazebo/set_model_state 
        #'{model_state: { model_name: robot1, pose: { position: { x: -1.5, y: 0 ,z: 0 }, orientation: {x: 0, y: 0, z: 0, w: 0 } },
        #  twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
        #print("".join(['rosservice call /gazebo/set_model_state ',"'",'{','model_state: ','{','model_name:',' robot', str(robot_ind),
        #', pose: { position: ',positions[robot_ind-1], ', orientation: ' , '{' , 'x: 0, y: 0, z: 0, w: 0 } }',
        #', twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } ','}',"'" ]))
        #rosservice call /gazebo/set_model_state 
        #'{model_state:{model_name: robot1, pose: { position: { x: -1.5, y: 15 ,z: 0 } orientation: {: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.0 , y: 0 ,z: 0 } ,
        #  angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'


        
        os.system("".join(['rosservice call /gazebo/set_model_state ',"'",'{','model_state:','{','model_name:',' robot', str(robot_ind),
        ', pose: { position: ',positions[robot_ind-1], ', orientation: ' , '{' , 'x: 0, y: 0, z: 0, w: 0 } }',
        ', twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } ','}',"'" ]))

        #os.system("".join(['rosservice call gazebo/delete_model ','{','model_name:',' robot', str(robot_ind),'}' ]) )
        #os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/robot/model.sdf -sdf -model',' robot',str(robot_ind),' -y 0.2 -x -0.3'] ))
        time.sleep(1)


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

positions=['{ x: -1.5, y: 0 ,z: 0 }','{ x: -1.5, y: 15 ,z: 0 }','{ x: -1.5, y: -15 ,z: 0 }']

def talker():

    rospy.init_node('control')

    pub1=rospy.Publisher("datos_fin", String,queue_size=1)

    sub=rospy.Subscriber("/datos_fin", String, feedback)
    rate = rospy.Rate(10) # 10hz
    ROBOTS = 3
    hilos_todos=[]
    

    time_eje=30
    cont=len(lista_de_datos)    
    while not rospy.is_shutdown():

        for num_hilo in range(1,ROBOTS+1):
            hilo = threading.Thread(name='hilo%s' %num_hilo, 
                                    target=call_robot,args = (num_hilo,time_eje,positions, ))    
            hilo.start()
            hilos_todos.append(hilo)
            
            #time.sleep(1)
            #hilo.join() 
            #print("thread finished...exiting") 


        if len(lista_de_datos)!=cont:
            print(lista_de_datos) 
            cont=len(lista_de_datos)
            
        for hilo in hilos_todos:    

            hilo.join() 
        #if len(lista_de_datos)>=8:
        #    break

    
  
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
  