#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import String

import os
#os.system('rostopic list')
import sys,time

def call_robot(robot_ind,time_eje,positions,pesos,bias_p):

        

        #print(pesos)
        os.system("".join(['rosrun p1 robot_red_neuronal.py -r ','robot',str(robot_ind), ' -t ',str(time_eje),' -w ',pesos,' -b ',bias_p]))
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

def spawn_robot_and_map(robot_ind,robots_por_linea):
    x=-10*((robot_ind-1)//robots_por_linea)
    y=-15*((robot_ind-1)%robots_por_linea)
    positions_robot=" ".join([" -x",str(x-1.5),"-y",str(y),"-z 0 "])
    positions_map=" ".join([" -x",str(x),"-y",str(y),"-z 0 "])
    print(positions_robot,positions_map)
    os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/robot/model.sdf -sdf -model',
    ' robot',str(robot_ind)," ",positions_robot] ))
    os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/default_maze/model.sdf -sdf -model',
    ' default_maze',str(robot_ind)," ",positions_map] ))
    return " ".join(["{ x:",str(x-1.5),", y:",str(y),", z: 0 }"])




def inicializar(n_x, n_h):
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
    # Definense as matrices y os vectores
    parametros={}
    W1 = np.random.rand(n_h, n_x)
    #W1 = np.zeros((n_h, n_x))
    b1 = np.random.rand(n_h, 1)
    
    # Gardanse os parametros nun diccionario
    parametros = {"W1": W1,
                  "b1": b1}

    return parametros

def inicilizar_pesos_robots(num_robots):
    num_sensores=8
    salida=2
    robot_pesos=[]
    for rob in range(num_robots):
        
        parametros = inicializar(num_sensores,salida)
        #Recuperamos los valores de la matriz de pesos (W1) y el vector con los bias (b1)
        W1 = parametros["W1"]
        b1 = parametros["b1"]
        W1_todo_peso=[]
        for j in range(len(W1)):
            W1str=[str(i) for i in W1[j]]
            W1_todo_peso.append(",".join(W1str))
        W1_todo_peso="_".join(W1_todo_peso)

        b1_todo_peso=[]
        for j in range(len(b1)):
            b1str=[str(i) for i in b1[j]]
            b1_todo_peso.append("_".join(b1str))
        b1_todo_peso=",".join(b1_todo_peso)

        robot_pesos.append("/".join([str(rob+1),W1_todo_peso,b1_todo_peso,"0",'\n']))
        #print(robot_pesos)
        time.sleep(1) 

    return robot_pesos


import operator

def get_best_fitness(dic_pesos_padres,ind_fitness):

    fitness=list(np.array(list(dic_pesos_padres.values()))[:,ind_fitness])
    dicionario=dict(zip(list(dic_pesos_padres.keys()),fitness))
    valores_ord = dict(sorted(dicionario.items(), key=operator.itemgetter(1)))
    return valores_ord

def pesos_str_to_np(z):
    W1=[]
    for x in z.split("_"):
        #print(x)
        #print(x.split(','))
        W1.append([float(y) for y in x.split(',')])
    return W1

def pesos_np_to_str(W1):
    W1_todo_peso=[]
    for j in range(len(W1)):
        W1str=[str(i) for i in W1[j]]
        W1_todo_peso.append(",".join(W1str))
    W1_todo_peso="_".join(W1_todo_peso)
    #print(W1_todo_peso)
    return W1_todo_peso

def cruce_aleatorio(padre_1,padre_2,num_div):
    
    padre_1=pesos_str_to_np(padre_1)
    padre_2=pesos_str_to_np(padre_2)
    #print(padre_1)
    
    hijo=[[],[]]
    if num_div==0:
        num_div=1
    cant_div=len(padre_1[0])//3
    resto=len(padre_1[0])-cant_div-1*3
    #print(resto)
    for ind in range(0,cant_div):
        if ind%2==0:
            hijo[0].extend(padre_1[0][ind*cant_div:(ind+1)*cant_div+1])
            hijo[1].extend(padre_1[1][ind*cant_div:(ind+1)*cant_div+1])
        else:
            hijo[0].extend(padre_2[0][ind*cant_div:(ind+1)*cant_div+1])
            hijo[1].extend(padre_2[1][ind*cant_div:(ind+1)*cant_div+1])

    if cant_div%2==0:
        hijo[0].extend(padre_2[0][(ind+1)*cant_div+2:(ind+1)*cant_div+resto+2])
        hijo[1].extend(padre_2[1][(ind+1)*cant_div+2:(ind+1)*cant_div+resto+2])
    else:
        hijo[0].extend(padre_1[0][(ind+1)*cant_div+2:(ind+1)*cant_div+resto+2])
        hijo[1].extend(padre_1[1][(ind+1)*cant_div+2:(ind+1)*cant_div+resto+2])

    return pesos_np_to_str(hijo)


def get_pesos_new_generation(dic_pesos_padres,dic_pesos_new):

    dic_ord_pdr_fitness=get_best_fitness(dic_pesos_padres,2)
    print(dic_ord_pdr_fitness.keys())
    for i in dic_pesos_padres.keys():
        #print(i)
        #hacemos por el momento para buscar padre
        if str(int(i)-1) in dic_pesos_padres:
            padre_2=dic_pesos_padres[str(int(i)-1)][0]
        else:
            padre_2=dic_pesos_padres[str(int(i)+1)][0]
        #print(i,padre_2)
        dic_pesos_new[i][0]=cruce_aleatorio(dic_pesos_padres[i][0],padre_2,np.random.randint(8))




#consigue los pesos del la generacion anterior del padre
def get_info_robot(dic_peso,line):
    line=line.split("/")
    #print(line)
    dic_peso[line[0]]=[line[1],line[2],line[3]]


lista_de_datos=[]


def feedback(msg):
    global lista_de_datos
    lista_de_datos.append(msg.data)
    #print(lista_de_datos)


positions=['{ x: -1.5, y: 0 ,z: 0 }','{ x: -1.5, y: -15 ,z: 0 }','{ x: -1.5, y: -30 ,z: 0 }']

def talker():
    global lista_de_datos
    rospy.init_node('control')

    pub1=rospy.Publisher("datos_fin", String,queue_size=1)

    sub=rospy.Subscriber("/datos_fin", String, feedback)

    rate = rospy.Rate(10) # 10hz
    ROBOTS = 20
    num_robot_por_lineas=4
    for i in range(4,ROBOTS+1):
        #print(-15(*(i-1)%3))
        positions.append(spawn_robot_and_map(i,num_robot_por_lineas))

    hilos_todos=[]
    lista_de_datos=[]
    #diccionario de pesos _de ejecucuion
    dic_peso={}
    #diccionrio pesos de los padres
    dic_p_padres={}
    # texto de padres
    file=os.path.realpath(__file__).split("/")
    file[-1]='robot.txt'
    file="/".join(file)
    #print(file)
    f = open (file,'w')
    textos=inicilizar_pesos_robots(ROBOTS)
    f.write("inicio\n")
    for line in textos:
        f.write(line)
    f.close()
    # archivo-entrada.py
    f = open (file,'r')
    mensaje = f.read()
    mensaje=mensaje.split("inicio")
    #print(mensaje.split("inicio"))
    #print(mensaje.split("\n"))
    f.close()
    for line in mensaje[-1].split("\n")[1:-1]:
        get_info_robot(dic_peso,line)

    time_eje=10
    cont=len(lista_de_datos)    
    
    while not rospy.is_shutdown():
        lista_de_datos=[]
        #f = open (file,'a')
        #f.write("generacion\n")
        for num_hilo in range(1,ROBOTS+1):
            hilo = threading.Thread(name='hilo%s' %num_hilo, 
                                    target=call_robot,args = (num_hilo,time_eje,positions,dic_peso[str(num_hilo)][0],dic_peso[str(num_hilo)][1]), )    
            hilo.start()
            hilos_todos.append(hilo)
            
            #time.sleep(1)
            #hilo.join() 
            #print("thread finished...exiting") 


        
        for hilo in hilos_todos:    

            hilo.join() 
        #if len(lista_de_datos)>=8:
        #    break
        rate.sleep()
        print("cantidad de datos:",len(lista_de_datos))
        f = open (file,'a')
        f.write("generacion\n")
        for line in lista_de_datos:
            f.write(line)
        f.close()
        f = open (file,'r')
        mensaje = f.read()
        mensaje=mensaje.split("inicio")
        mensaje=mensaje[-1].split("generacion")
        for line in mensaje[-1].split("\n")[1:-1]:
            get_info_robot(dic_p_padres,line)

        #print(dic_p_padres)
        if len(dic_p_padres)!=0:
            get_pesos_new_generation(dic_p_padres,dic_peso)
        #print(mensaje.split("inicio"))
        #print(mensaje.split("\n"))
        rate.sleep()
        f.close()
        

    
  
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
  