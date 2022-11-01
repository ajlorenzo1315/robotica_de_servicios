#!/usr/bin/env python

import sys
import rospy
from math import sqrt
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

import rospy
from turtle import position
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np 
from math import sqrt

# Para trabajar con argumentos
import sys, getopt, time

# Red neuronal
import numpy as np
import matplotlib.pyplot as plt

class OdometryModifier:
    def __init__(self):
        self.sub = rospy.Subscriber("odom", Odometry, self.callback)
        self.pub = rospy.Publisher('odom2', Odometry, queue_size=10)
        self.total_distance = 0.
        self.previous_x = 0
        self.previous_y = 0
        self.first_run = True

    def callback(self, data):
        if(self.first_run):
            self.previous_x = data.pose.pose.position.x
            self.previous_y = data.pose.pose.position.y
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        d_increment = sqrt((x - self.previous_x) * (x - self.previous_x) +
                    (y - self.previous_y)(y - self.previous_y))
        self.total_distance = self.total_distance + d_increment
        print("Total distance traveled is {.2f}m".format(self.total_distance))
        self.pub.publish(data)
        self.previous_x = data.pose.pose.position.x
        self.previous_y = data.pose.pose.position.y
        self.first_run = False


def iniciar_nodo(argv):
    
    limit = None
    W1 = []
    b1 = []


    try:
        opts, args = getopt.getopt(argv,":r:t:w:b:")
    except getopt.GetoptError:
        print("Argumento no valido: usar -r (robot name)")
        opts = ["-r","robot1"]
    #print(opts)
    for opt, arg in opts:
        #print(opt)
        if opt in ("-r" "--robot"):
            robot = arg

        elif opt in ("-t" "--time"):
            limit = int(arg)

        elif opt in ("-w" "--wheigth"):
            #z="0.01624345 -0.00611756 -0.00528172 -0.01072969,0.00865408 -0.02301539  0.01744812 -0.00761207"
            z = arg
            #print(z)
            for x in z.split("_"):
                W1.append([float(y) for y in x.split(',')])
            W1 = np.array(W1)

        elif opt in ("-b" "--bias"):
            x = arg
            b1 = [float(y) for y in x.split(",")]
            b1 = np.array([b1]).T

        else:
            raise Exception("Flag no valida")

    rospy.init_node(robot)
    print(robot)
    #print(W1)
    #print("bias")
    #print(b1)
    topic_robot = "".join(["/", robot])
    sub = rospy.Subscriber( "/".join([topic_robot, "laser_front/scan"]) , LaserScan, robot_1_sensorfront)
    sub2 = rospy.Subscriber("/".join([topic_robot, "laser_back/scan"]), LaserScan, robot_1_sensorback)
    sub3 = rospy.Subscriber("/".join([topic_robot, "odom"]), Odometry, robot_1_Odometry)    
    move_r1 = rospy.Publisher("/".join([topic_robot, "cmd_vel"]), Twist,queue_size=1)

    sub_odom = rospy.Subscriber("Odom", Odometry, callback_odom)
    pub_odom = rospy.Publisher("Odom2", Odometry, queue_size=10)
    

    move = Twist()
    rate = rospy.Rate(10)
    cero_move(move_r1,move)

    datos_fedback = rospy.Publisher("/datos_fin", String,queue_size=1)

    inicial = time.time()
    ahora = time.time()
    if limit != None:
        limit = inicial + limit

    while len(r1_dsf) == 0 or len(r1_dsb) == 0:
        #print("esperando")
        a = 1

    X = np.array([ r1_dsf+r1_dsb ]).T / 10
    #print("X: " ,X)
    #print(X.shape[0])
    
    if len(W1) <= 0:
        parametros = inicializar(X.shape[0], 2, 1)
        # Recuperamos los valores de la matriz de pesos (W1) y el vector con los bias (b1)
        W1 = parametros["W1"]
        b1 = parametros["b1"]
    
    dist_media = []
    dist_minima = []
    dist_minima_front = []
    global dist_minima_front_l, dist_minima_front_r
    dist_minima_front_l = []
    dist_minima_front_r = []
    seguridad_dist = 0.45
    while not rospy.is_shutdown():
        #print("probando si funciona")
        X = np.array([ r1_dsf+r1_dsb ]).T
        dist_minima.append(np.min(X))
        dist_minima_front.append(np.min(np.array(r1_dsf)))
        dist_minima_front_l.append(np.min(np.array(r1_dsf[:len(r1_dsf)//2 ])))
        dist_minima_front_r.append(np.min(np.array(r1_dsf[len(r1_dsf)//2:] )))

        distancias = dist_minima_front + dist_minima_front_l + dist_minima_front_r
        dist_media.append(sum(distancias)/len(distancias)) 
        
        
        #print("X: " ,X)
        #print(r1_dsf)
        A = propagar(X, W1, b1, "sigmoide")[0]
        
        #print("A : ",A[0])
        alante(move_r1,move,A)
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        
        if dist_minima_front[-1]<=seguridad_dist:
                print("distancia_minima",dist_minima[-1])
                #print("X: " ,X)
                cero_move(move_r1,move)
                msg_send = send_info(robot, W1, b1, str(funcion_fitness(dist_minima, vels, angulos, ahora-inicial, dist_minima_front, 1)))
                #print("send info",robot)
                #print(angulos)
                datos_fedback.publish(msg_send)
                time.sleep(3)
                break

        ahora = time.time()
        if limit != None:
            if ahora >= limit:
                cero_move(move_r1,move)
                msg_send = send_info(robot, W1, b1, str(funcion_fitness(dist_minima, vels, angulos, ahora-inicial, dist_minima_front, 0)))
                #print("send info",robot)
                #print(angulos)
                datos_fedback.publish(msg_send)
                time.sleep(3)
                break

        print("\n DISTANCIA TOTALPERCORRIDA: ", )
        print("DISTANCIA MEDIA A OBSTACULOS: ", dist_media)
        print("DISTANCIA MINIMA A OBSTACULOS: ", dist_minima)

        rate.sleep()

if __name__ == '__main__':
    try:
        iniciar_nodo(sys.argv[1:])
        odom = OdometryModifier()
    except rospy.ROSInterruptException:
        pass