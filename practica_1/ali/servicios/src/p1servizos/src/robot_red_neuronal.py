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

r1_dsf=[]


#robot 1 sensor back

r1_dsb=[]



def robot_1_sensorfront(msg):

    global r1_dsf

    #rospy.loginfo("I heard %s",data.data) 

    r1_dsf=list(msg.ranges)
    
    


def robot_1_sensorback(msg):

    global r1_dsb

    r1_dsb=list(msg.ranges)
   

# red neuronal
import numpy as np
import matplotlib.pyplot as plt

#https://github.com/ArztSamuel/Applying_EANNs

def inicializar(n_x, n_h, n_y,semilla=1,mult=0.01):
    """
    Argument:
    n_x -- n neuronas da capa de entrada
    n_h -- n neuronas da capa oculta
    n_y -- n neuronas da capa de salida
    semilla -- semilla  para realizar todos a mesma inicializacion
    mult -- para que los valores sean float o anular el valor a cero

    Returns:
    parametros --  diccionario que conten:
                    W1 -- matriz de pesos (n_h, n_x)
                    b1 -- vector de bias (n_h, 1)
                    W2 -- matriz de pesos (n_y, n_h)
                    b2 -- vector de bias  (n_y, 1)
    """
    #Establecemos a mesma semilla para realizar todos a mesma inicializacion
    
    W1 = np.random.random(size=(n_h, n_x))
    #W1 = np.zeros((n_h, n_x))
    b1 = np.random.random(size=((n_h, 1)))
    
    # Gardanse os parametros nun diccionario
    parametros = {"W1": W1,
                  "b1": b1}

    return parametros

def combinacion_lineal(A, W, b):
    """
    Implementacion da parte lineal da propagacion.

    Arguments:
    A -- saidas da capa anterior (ou datos de entrada): dimensions (tamano da capa anterior ou dos datos de entrada)
    W -- matriz de pesos: dimensions (tamano da capa actual, tamano da capa previa)
    b -- vector cos bias (tamano da capa actual, 1)

    Returns:
    Z -- entrada para a funcion de activacion  
    cache -- tupla que conten os valores dos argumentos para a fase de retropropagacion
    """
    
    # Aplicase a expresion para o calculo da parte lineal 
    Z = np.dot(W,A) + b
    
    # Gardanse os argumentos nunha tupla
    cache = (A, W, b)
    
    return Z, cache

def sigmoid(Z):
    """
    Implementa a funcion sigmoide

    Arguments:
    Z -- array de numpy

    Returns:
    A -- funcion sigmoid(Z), igual dimensions que Z
    cache -- almacena Z, para su uso posterio na fase de retropropagacion
    """

    A = 1/(1+np.exp(-Z))
    
    cache = Z

    return A, cache


def relu(Z):
    """
    Implementa a funcion Relu

    Arguments:
    Z -- array de numpy

    Returns:
    A -- funcion relu(Z), igual dimensions que Z
    cache -- almacena Z, para su uso posterio na fase de retropropagacion
    """

    A = np.maximum(0,Z)

    cache = Z
    
    return A, cache

def propagar(A_prev, W, b, activacion):
    """
    Implementacion do modulo de propagacion LINEAR + ACTIVATION

    Argumentos:
    A -- saidas da capa anterior (ou datos de entrada): dimensions (tamano da capa anterior o dos datos de entrada)
    W -- matriz de pesos: dimensions (tamano da capa actual, tamano da capa previa)
    b -- vector cos bias (tamano da capa actual, 1)
    activacion -- funcion de activacion empleada: "sigmoid" or "relu". Se a entrada e incorrecta usa sigmoide
    Returns:
    A -- salida de la funcion de activacion  
    cache -- tupla que conten as caches:  "lineal_cache" y "activacion_cache"
    """
    
    if activacion == "sigmoide":
        
        Z, lineal_cache = combinacion_lineal(A_prev, W, b)
        A, activacion_cache = sigmoid(Z)
    
    elif activacion == "relu":

        Z, lineal_cache = combinacion_lineal(A_prev, W, b)
        A, activacion_cache = relu(Z) 
        
    else:
        
        Z, lineal_cache = combinacion_lineal(A_prev, W, b)
        A, activacion_cache = sigmoid(Z)    
    
    
    cache = (lineal_cache, activacion_cache)

    return A, cache

####


def cero_move(pub,move):

    move.linear.x=0
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=0

    pub.publish(move)

def alante(pub,move,vel):
    #print(vel)
    move.linear.x=vel[0]
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=vel[1]

    pub.publish(move)


def send_info(robot,W1,b1,fitness):
    W1_todo_peso=[]
    for j in range(len(W1)):
            W1str=[str(i) for i in W1[j]]
            W1_todo_peso.append(" ".join(W1str))
    W1_todo_peso=",".join(W1_todo_peso)

    b1_todo_peso=[]
    for j in range(len(b1)):
            b1str=[str(i) for i in b1[j]]
            b1_todo_peso.append(" ".join(b1str))
    b1_todo_peso=",".join(b1_todo_peso)

    robot_pesos="/".join([str(robot),W1_todo_peso,b1_todo_peso,fitness,"\n"])

    return robot_pesos


#def info_robot(vel,time,time_inicial,distancias):

def iniciar_nodo(argv):
    
    limit=None
    W1=[]
    b1=[]


    try:
        opts, args = getopt.getopt(argv,":r:t:w:b:")
    except getopt.GetoptError:
        print("Argumento no valido: usar -r (robot name)")
        opts=["-r","robot1"]
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
            W1=np.array(W1)

        elif opt in ("-b" "--bias"):
            x = arg
            b1 = [float(y) for y in x.split(",")]
            b1=np.array([b1]).T

        else:
            raise Exception("Flag no valida")

    rospy.init_node(robot)
    print(robot)
    print(W1)
    print("bias")
    print(b1)
    topic_robot="".join(["/",robot])
    sub=rospy.Subscriber( "/".join([topic_robot,"laser_front/scan"]) , LaserScan, robot_1_sensorfront)
    sub2=rospy.Subscriber("/".join([topic_robot,"laser_back/scan"]), LaserScan, robot_1_sensorback)
        
    move_r1=rospy.Publisher("/".join([topic_robot,"cmd_vel"]), Twist,queue_size=1)

    

    move=Twist()

    rate=rospy.Rate(10)

    cero_move(move_r1,move)

    datos_fedback=rospy.Publisher("/datos_fin", String,queue_size=1)

    inicial = time.time()
    if limit!=None:
        limit = inicial + limit

    while len(r1_dsf)==0 or len(r1_dsb)==0:
        #print("esperando")
        a=1

    X = np.array([ r1_dsf+r1_dsb ]).T
    print("X: " ,X)
    #print(X.shape[0])
    
    if len(W1)<=0:
        parametros = inicializar(X.shape[0],2,1)
        #Recuperamos los valores de la matriz de pesos (W1) y el vector con los bias (b1)
        W1 = parametros["W1"]
        b1 = parametros["b1"]
    
    
    while not rospy.is_shutdown():
        #print("probando si funciona")
        X = np.array([ r1_dsf+r1_dsb ]).T
        #print("X: " ,X)
        #print(r1_dsf)
        A = propagar(X, W1, b1, "sigmoide")[0]
        
        #print("A : ",A[0])
        alante(move_r1,move,A)
        # spin() simply keeps python from exiting until this node is stopped
        #rospy.spin()
        
        

        inicial = time.time()
        if limit!=None:
            if inicial>=limit:
                cero_move(move_r1,move)
                msg_send=send_info(robot,W1,b1,str(inicial-limit))
                print("send info",robot)
                datos_fedback.publish(msg_send)
                time.sleep(3)
                break
            
        rate.sleep()


if __name__ == '__main__':
    try:

        iniciar_nodo(sys.argv[1:])

    except rospy.ROSInterruptException:
        pass