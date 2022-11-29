#!/usr/bin/env python
from cmath import pi
from turtle import position
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from std_msgs.msg import String


import numpy as np 
# para trabajar con argumentos
import sys, getopt
import time


#robot 1 sensor front

r1_dsf=[]


#robot 1 sensor back

r1_dsb=[]

r1_positions=[]
desplacamiento=0

def robot_1_sensorfront(msg):

    global r1_dsf

    #rospy.loginfo("I heard %s",data.data) 

    r1_dsf=list(msg.ranges)
    
def robot_1_sensorback(msg):

    global r1_dsb

    r1_dsb=list(msg.ranges)

def magnitude(vector): 
    return np.sqrt(sum(pow(element, 2) for element in vector))


def robot_1_Odometry(msg):
    global r1_positions,desplacamiento
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    #if len(r1_positions)>=1:
    #    desplacamiento+=magnitude(np.array(r1_positions[-1])-np.array([x,y]))
    r1_positions.append([x,y])

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
    
    W1 =np.random.uniform(low=-1, high=1, size=(n_h, n_x)) #np.random.randn(n_h, n_x)
    #W1 = np.zeros((n_h, n_x))
    b1 = np.random.uniform(low=-1, high=1, size=(n_h, 1)) #np.random.randn(n_h, 1)
    
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

def t_hip(Z):
    """
    Implementa a funcion tangente hiperbolica

    Arguments:
    Z -- array de numpy

    Returns:
    A -- funcion t_hip(Z), igual dimensions que Z
    cache -- almacena Z, para su uso posterio na fase de retropropagacion
    """
    
    A = (1-np.exp(-Z))/(1+np.exp(-Z))
    
    
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

    elif activacion == "t_hip":
        Z, lineal_cache = combinacion_lineal(A_prev, W, b)
        A, activacion_cache = t_hip(Z) 

    else:
        
        Z, lineal_cache = combinacion_lineal(A_prev, W, b)
        A, activacion_cache = t_hip(Z)     
    
    
    cache = (lineal_cache, activacion_cache)

    return A, cache

####
def get_angle(angulo):
    new_angulo=angulo*2-1
    new_angulo=new_angulo*np.pi/3
    return new_angulo

def get_vel(vel):
    new_vel=(vel*2-1)/2.5
    return new_vel

def cero_move(pub,move):

    move.linear.x=0
    move.linear.y=0
    move.linear.z=0

    move.angular.x=0
    move.angular.y=0
    move.angular.z=0

    pub.publish(move)

angulos=[]
vels=[]

def alante(pub,move,vel):
    #print(vel)
    global angulos,vels
   
    move.linear.x=get_vel(vel[0])
    move.linear.y=0
    move.linear.z=0
    vels.append(move.linear.x)
    move.angular.x=0
    move.angular.y=0
    move.angular.z=get_angle(vel[1])
    angulos.append(abs(0-move.angular.z))
    pub.publish(move)


def send_info(robot,W1,b1,fitness):
    W1_todo_peso=[]
    # pasar de np a str pesos
    for j in range(len(W1)):
            W1str=[str(i) for i in W1[j]]
            W1_todo_peso.append(",".join(W1str))
    W1_todo_peso="_".join(W1_todo_peso)

    b1_todo_peso=[]
    # pasar de np a str bias
    for j in range(len(b1)):
            b1str=[str(i) for i in b1[j]]
            b1_todo_peso.append(" ".join(b1str))
    b1_todo_peso=",".join(b1_todo_peso)

    robot_pesos="/".join([str(robot).split("robot")[-1],W1_todo_peso,b1_todo_peso,fitness,"\n"])

    return robot_pesos

def cambio_de_tiempo(tiempo):
    tiempo=int(tiempo)
    return np.sum([np.e**(-i/1000) for i in range(0,tiempo)])

def dist_recorrida(p,previous_p):
    #print(p,previous_p)
    x,y=p
    previous_x,previous_y=previous_p
    #print
    d_increment = np.sqrt((x - previous_x) * (x - previous_x) +
                          (y - previous_y) * (y - previous_y))
    #print(d_increment)
    return d_increment

def funcion_fitness(robot,distancia_minimas,vels,angulos,tiempo,distancia_minim_front,choco):
    global dist_minima_front_l,dist_minima_front_r,r1_positions

    #print(robot,tiempo)
    #tiempo_n=tiempo
    #if tiempo>50:
    #    tiempo_n=50
    #tiempo_n=cambio_de_tiempo(tiempo_n)
    
    desplacamiento=np.sum(np.array([ dist_recorrida( r1_positions[i],r1_positions[i-1] ) for i in range(1, len(r1_positions) ) ]) )
    print(robot,tiempo,desplacamiento,choco,np.mean(angulos),np.mean(vels))
    #distancias=np.mean(dist_minima_front_l)+np.mean(dist_minima_front_r)+2.5*desplacamiento
    distancias=np.mean(distancia_minim_front)*15+np.mean(dist_minima_front_l)*5+np.mean(dist_minima_front_r)*5+100*desplacamiento+30*np.mean(distancia_minimas)
    #vel=3*np.mean(vels)+(3/np.mean(angulos))+len(angulos)/np.sum(angulos)
    vel=100*np.mean(vels)+(100/(np.mean(angulos)*10))+(100/(np.sum(angulos)))

    a=distancias + vel-(choco*200)
    #lo que hace es que el robot se mueva muchisimo mas lento para no colisionar

    #0.5*np.mean(distancia_minimas)+(np.mean(vels)/3)+(3/np.mean(angulos))+(time/np.sum(angulos))+desplacamiento+np.mean(distancia_minim_front)+(len(angulos)/np.sum(angulos))
    if np.isnan(a):
        a=0
    return a

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
    #print(W1)
    #print("bias")
    #print(b1)
    topic_robot="".join(["/",robot])
    sub=rospy.Subscriber( "/".join([topic_robot,"laser_front/scan"]) , LaserScan, robot_1_sensorfront)
    sub2=rospy.Subscriber("/".join([topic_robot,"laser_back/scan"]), LaserScan, robot_1_sensorback)
    sub3=rospy.Subscriber("/".join([topic_robot,"odom"]), Odometry, robot_1_Odometry)    
    move_r1=rospy.Publisher("/".join([topic_robot,"cmd_vel"]), Twist,queue_size=1)

    

    move=Twist()

    rate=rospy.Rate(5)

    cero_move(move_r1,move)

    datos_fedback=rospy.Publisher("/datos_fin", String,queue_size=1)

    inicial = time.time()
    ahora=time.time()
    if limit!=None:
        limit = inicial + limit
    else:
        limit = 380

    while (len(r1_dsf)==0 or len(r1_dsb)==0 ) or (limit!=None and ahora>=limit) : 
        #print("esperando")
        a=1
        ahora = time.time()
        if (limit!=None and ahora>=limit) :
            break
    if len(r1_dsf)==0:
        datos_fedback.publish(robot)
        
    else:
        X = np.array([ r1_dsf+r1_dsb ]).T/10
        #print("X: " ,X)
        #print(X.shape[0])
        
        if len(W1)<=0:
            parametros = inicializar(X.shape[0],2,1)
            #Recuperamos los valores de la matriz de pesos (W1) y el vector con los bias (b1)
            W1 = parametros["W1"]
            b1 = parametros["b1"]
        
        dist_minima=[]
        dist_minima_front=[]
        global dist_minima_front_l,dist_minima_front_r
        dist_minima_front_l=[]
        dist_minima_front_r=[]
        seguridad_dist=0.47

        try:
            print("esmos entrando en el bucle principal",robot)
            while not rospy.is_shutdown():
                #print("probando si funciona")
                X = np.array([ r1_dsf+r1_dsb ]).T
                dist_minima.append(np.min(X))
                dist_minima_front.append(np.min(np.array(r1_dsf)))
                dist_minima_front_l.append(np.min(np.array(r1_dsf[:len(r1_dsf)//2 ])))
                dist_minima_front_r.append(np.min(np.array(r1_dsf[len(r1_dsf)//2:] )))
                #print("X: " ,X)
                #print(r1_dsf)
                A = propagar(X, W1, b1, "sigmoide")[0]
                
                #print("A : ",A[0])
                alante(move_r1,move,A)
                # spin() simply keeps python from exiting until this node is stopped
                #rospy.spin()
                
                ahora = time.time()
                if dist_minima_front[-1]<=seguridad_dist or (limit!=None and ahora>=limit) or dist_minima[-1]<=seguridad_dist:
                        #print("robot",robot," distancia_minima: ",dist_minima[-1])
                        #print("X: " ,X)
                        cero_move(move_r1,move)

                        time.sleep(1)

                        colision=0
                        if dist_minima_front[-1]<=seguridad_dist or dist_minima[-1]<=seguridad_dist:
                            colision=1
                        msg_send=send_info(robot,W1,b1,str(funcion_fitness(robot,dist_minima,vels,angulos,ahora-inicial,dist_minima_front,colision) ))
                        #print("send info",robot)
                        #print(angulos)
                        datos_fedback.publish(msg_send)
                        
                        break
                    
                if ahora-inicial%10 ==0:
                    print(ahora-inicial)
                
                rate.sleep()
                
        except Exception as e: # work on python 3.x
            print('Failed to upload to ftp: '+ str(e))

if __name__ == '__main__':
    try:

        iniciar_nodo(sys.argv[1:])

    except rospy.ROSInterruptException:
        pass