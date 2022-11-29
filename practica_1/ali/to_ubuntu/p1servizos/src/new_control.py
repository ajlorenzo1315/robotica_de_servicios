#!/usr/bin/env python

from ast import For
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import String

#reposition
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

import os
#os.system('rostopic list')
import sys,time

def call_robot(robot_ind,time_eje,pesos,bias_p):
        os.system("".join(['rosrun p1 robot_red_neuronal_new.py -r ','robot_clone_',str(robot_ind-1), ' -t ',str(time_eje),' -w ',pesos,' -b ',bias_p]))
        

import threading
import random


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
    W1 =np.random.uniform(low=-1, high=1, size=(n_h, n_x)) #np.random.randn(n_h, n_x)
    #W1 = np.zeros((n_h, n_x))
    b1 = np.random.uniform(low=-1, high=1, size=(n_h, 1)) #np.random.randn(n_h, 1)
    
    
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
    valores_ord = dict(sorted(dicionario.items(), key=operator.itemgetter(1),reverse=True))
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

def cruce_angulo_lineal(padre_1,padre_2,mutacion):



    padre_1=pesos_str_to_np(padre_1)
    padre_2=pesos_str_to_np(padre_2)
    a_min=-1
    a_max=1
    mutaciones=np.zeros((4,len(padre_1[0])))
    
    for i in range (0,4):
        muta=np.random.choice(mutacion)
        for j in range(0,muta+1):
            ind_mutados=np.random.choice(range(0,len(padre_1[0])),muta,False)
            cuanto_muta=np.random.uniform(low=-1, high=1, size=(1))[0]/10
            mutaciones[i][ind_mutados]=cuanto_muta
    
    hijo1=np.array([  padre_1[0]+mutaciones[0] ,  padre_2[1]+mutaciones[1] ])

    hijo2=np.array([  padre_2[0]+mutaciones[2] ,  padre_1[1]+mutaciones[3] ])

    hijo1=np.clip( hijo1 , a_min , a_max)
    hijo1=np.clip( hijo2 , a_min , a_max)

    return pesos_np_to_str(hijo1),pesos_np_to_str(hijo2)

def bias_cruce(padre_1,padre_2):
    b_padre_1=padre_1.split(",")
    b_padre_2=padre_2.split(",")

    hijo_1=",".join([b_padre_1[0],b_padre_2[1]])
    hijo_2=",".join([b_padre_2[0],b_padre_1[1]])
    return hijo_1,hijo_2

def cruce_ponderado(padre_1,padre_2,prob_mut,alpha=0.7):
    mutacion=[0,1,2,3,4,5,6,7,8]
    mutaciones=np.zeros((4,len(padre_1[0])))
    a_min=-1
    a_max=1
    for i in range (0,4):
        muta=np.random.choice(mutacion,p=prob_mut)
        for j in range(0,muta+1):
            ind_mutados=np.random.choice(range(0,len(padre_1[0])),muta,False)
            cuanto_muta=np.random.uniform(low=-1, high=1, size=(1))[0]/10
            mutaciones[i][ind_mutados]=cuanto_muta
    
    hijo1=np.array([  padre_1[0]*(alpha)+padre_2[0]*(1-alpha)+mutaciones[0] ,  padre_1[1]*(alpha)+padre_2[1]*(1-alpha)+mutaciones[1] ])

    hijo2=np.array([  padre_1[0]*(1-alpha)+padre_2[0]*(alpha)+mutaciones[2] ,  padre_1[1]*(1-alpha)+padre_2[1]*(alpha)+mutaciones[3] ])
    hijo1=np.clip( hijo1 , a_min , a_max)
    hijo1=np.clip( hijo2 , a_min , a_max)
    return pesos_np_to_str(hijo1),pesos_np_to_str(hijo2)

def mutar_hijo(hijo,prob_mut):
    mutacion=[0,1,2,3,4,5,6,7,8]
    mutaciones=np.zeros((2,len(hijo[0])))
    a_min=-1
    a_max=1
   
    for i in range (0,2):
        muta=np.random.choice(mutacion,p=prob_mut)
        for j in range(0,muta+1):
            ind_mutados=np.random.choice(range(0,len(hijo[0])),muta,False)
            cuanto_muta=np.random.uniform(low=-1, high=1, size=(1))[0]/10
            mutaciones[i][ind_mutados]=cuanto_muta
    
    hijo1=np.array([  hijo[0]+mutaciones[0] ,  hijo[1]+mutaciones[1] ])
    hijo1=np.clip( hijo1 , a_min , a_max)
    return pesos_np_to_str(hijo1)

def bias_ponderado(padre_1,padre_2,alpha=0.7):

    a_min=-1
    a_max=1

    b_padre_1=np.array([float(y) for y in padre_1.split(",")])
    b_padre_2=np.array([float(y) for y in padre_2.split(",")])

    b1= b_padre_1*(alpha)+b_padre_2*(1-alpha)
    b2= b_padre_1*(1-alpha)+b_padre_2*(alpha)

    b1=np.clip( b1 , a_min , a_max)
    b2=np.clip( b2 , a_min , a_max)

    hijo_1=",".join([str(y) for y in b1])
    hijo_2=",".join([str(y) for y in b2])

    return hijo_1,hijo_2

def propabilidades_padres_list(valores_ord):

    #padres_list=list(valores_ord.keys())
    
    #print(lista)
    #probabilidades(lista)
    padres_list=list(valores_ord.keys())
    #total=float(valores_ord[padres_list[0]])
    longitud=len(padres_list)
    lista=[len(padres_list)/i  for i in range(1,len(padres_list)+1)]
    #print(total)
    pobrabilidades=lista/np.sum(lista)
    #[longitud* (float(valores_ord[padres_list[i]])) /total*(np.e**-(i/longitud)) for i in range(len(padres_list))]
    return pobrabilidades,padres_list

def  generar_lista_de_probabilidades(elementos,pesos):
    resultado=[]
    for e,p in zip (elementos,pesos):
        resultado+=[e]*int(p*100)
    return resultado

def funcion_lerning_rate(ciclo):
    if ciclo==0:
        ciclo=0.000000000000000001
    ciclo=ciclo/5
    return 8*(np.e**-ciclo)

def mutaciones_probabilidades(ciclo):
    #elementos = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    lr=funcion_lerning_rate(ciclo)
    pesos2=lr*np.array([ 0.5, 0.2, 0.1, 0.1, 0.1, 0.05, 0.02, 0.01])
    pesos = np.append(np.array([2]),pesos2)
    pesos = pesos/np.sum(pesos)
    print("------------------------------------------")
    print(pesos)
    print("------------------------------------------")
    #resultado=generar_lista_de_probabilidades(elementos,pesos)
    return pesos

import random

def get_pesos_new_generation(dic_pesos_padres,dic_pesos_new,ciclo,filef):

    dic_ord_pdr_fitness=get_best_fitness(dic_pesos_padres,2)
    ff = open (filef,'a')
    ff.write("generacion\n")
    for key in dic_ord_pdr_fitness.keys():
        line="/".join([key,dic_ord_pdr_fitness[key]])
        ff.write(line)
    ff.close()
    #print("diccionario",dic_ord_pdr_fitness)
    porc_metodos=[0.05,0.15,0.4]
    pobrabilidades,padres_list=propabilidades_padres_list(dic_ord_pdr_fitness)
    print("------------------------------------------")
    print(pobrabilidades)
    print("------------------------------------------")
    print(padres_list)
    #resultado=generar_lista_de_probabilidades(padres_list,pobrabilidades)
    mutacion=mutaciones_probabilidades(ciclo)
    #print(dic_ord_pdr_fitness.keys())
    lista_hijos=list(dic_pesos_padres.keys())

    pasa_limpio=int(len(lista_hijos)*porc_metodos[0])
    pasa_mutado=int(pasa_limpio+len(lista_hijos)*porc_metodos[1])
    cruce_punto=int(pasa_mutado+len(lista_hijos)*porc_metodos[2])
    
    for i in range(pasa_limpio):
        dic_pesos_new[lista_hijos[i]][0]=dic_pesos_padres[lista_hijos[i]][0]
        dic_pesos_new[lista_hijos[i]][1]=dic_pesos_padres[lista_hijos[i]][1]

    for i in range(pasa_limpio,pasa_mutado):
        dic_pesos_new[lista_hijos[i]][0]=mutar_hijo(dic_pesos_padres[lista_hijos[i]][0],mutacion)
        dic_pesos_new[lista_hijos[i]][1]=dic_pesos_padres[lista_hijos[i]][1]
    
    for i in range(pasa_mutado,int(pasa_mutado+((cruce_punto-pasa_mutado)//2)) ):

        padre_1=np.random.choice(padres_list,p=pobrabilidades)
        padre_2=np.random.choice(padres_list,p=pobrabilidades)
        while padre_2==padre_1:
            padre_2=np.random.choice(padres_list,p=pobrabilidades)

        #print(padre_1,)
        padre_1p=dic_pesos_padres[padre_1][0]
        padre_2p=dic_pesos_padres[padre_2][0]
        
        #print(padre_1p,padre_2p)
        hijo1,hijo2=cruce_angulo_lineal(padre_1p,padre_2p,mutacion)
        #print(lista_hijos[i*2+1],lista_hijos[i*2])
        dic_pesos_new[ lista_hijos[i*2] ][0]=hijo1
        dic_pesos_new[ lista_hijos[i*2+1] ][0]=hijo2

        padre_1p=dic_pesos_padres[padre_1][1]
        padre_2p=dic_pesos_padres[padre_2][1]

        hijo1,hijo2=bias_cruce(padre_1p,padre_2p)
        dic_pesos_new[lista_hijos[i*2]][1]=hijo1
        dic_pesos_new[lista_hijos[i*2+1]][1]=hijo2

    for i in range(cruce_punto,len(lista_hijos)//2):

        padre_1=np.random.choice(padres_list,p=pobrabilidades)
        padre_2=np.random.choice(padres_list,p=pobrabilidades)
        while padre_2==padre_1:
            padre_2=np.random.choice(padres_list,p=pobrabilidades)

        #print(padre_1,)
        padre_1p=dic_pesos_padres[padre_1][0]
        padre_2p=dic_pesos_padres[padre_2][0]
        
        #print(padre_1p,padre_2p)
        hijo1,hijo2=cruce_ponderado(padre_1p,padre_2p,mutacion)
        #print(lista_hijos[i*2+1],lista_hijos[i*2])
        dic_pesos_new[ lista_hijos[i*2] ][0]=hijo1
        dic_pesos_new[ lista_hijos[i*2+1] ][0]=hijo2

        padre_1p=dic_pesos_padres[padre_1][1]
        padre_2p=dic_pesos_padres[padre_2][1]

        hijo1,hijo2=bias_ponderado(padre_1p,padre_2p)
        dic_pesos_new[lista_hijos[i*2]][1]=hijo1
        dic_pesos_new[lista_hijos[i*2+1]][1]=hijo2





#consigue los pesos del la generacion anterior del padre
def get_info_robot(dic_peso,line):
    line=line.split("/")
    #print(line)
    try:
        dic_peso[line[0]]=[line[1],line[2],float(line[3])]
    except:
        print(line)


lista_de_datos=[]


def feedback(msg):
    global lista_de_datos
    lista_de_datos.append(msg.data)
    #print(lista_de_datos)


def reestar_robot_position(model_name,position):
    state_msg = ModelState()
    state_msg.model_name = model_name
    state_msg.pose.position.x = position[0]
    state_msg.pose.position.y = position[1]
    state_msg.pose.position.z = 0.0
    state_msg.pose.orientation.x = 0
    state_msg.pose.orientation.y = 0
    state_msg.pose.orientation.z = 0
    state_msg.pose.orientation.w = 0
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        resp = set_state( state_msg )

    except rospy.ServiceException as e:
        print("Service call failed: %s" %e )

#positions=['{ x: -1.5, y: 0 ,z: 0 }','{ x: -1.5, y: -15 ,z: 0 }','{ x: -1.5, y: -30 ,z: 0 }']

#genes_mutados_elementos = ['0', '1', '2', '3', '4', '5', '6', '7', '8']
#pesos_genes_mutado = [2, 0.5, 0.2, 0.1, 0.1, 0.1, 0.05, 0.02, 0.01]


def talker():
    global lista_de_datos
    rospy.init_node('control')

    pub1=rospy.Publisher("datos_fin", String,queue_size=1)

    sub=rospy.Subscriber("/datos_fin", String, feedback)

    ROBOTS = 30
    print("INICIALIZAMOS PESOS")
    #diccionario de pesos _de ejecucuion
    dic_peso={}
    #diccionrio pesos de los padres
    dic_p_padres={}
    # texto de padres
    filef=os.path.realpath(__file__).split("/")
    filef[-1]='robot_fitness.txt'
    filef="/".join(filef)
    ff = open (filef,'w')
    ff.close()
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

    time_eje=180

    cont=len(lista_de_datos)    
    cont_ciclos=0
    rate = rospy.Rate(time_eje) # 10hz
    while not rospy.is_shutdown():
        lista_de_datos=[]
        hilos_todos=[]
        #f = open (file,'a')
        #f.write("generacion\n")
        for num_hilo in range(1,ROBOTS+1):
            hilo = threading.Thread(name='hilo%s' %num_hilo, 
                                    target=call_robot,args = (num_hilo,time_eje,dic_peso[str(num_hilo)][0],dic_peso[str(num_hilo)][1]))    
            #hilo.setDaemon(True)
            hilo.start()
            #hilo.raise_exception() 
            hilos_todos.append(hilo)
            
            #time.sleep(1)
            #hilo.join() 
            #print("thread finished...exiting") 

        for hilo in hilos_todos:    
            hilo.join() 
        #if len(lista_de_datos)>=8:
        #    break
        #for hilo in hilos_todos:
        #    if hilo.isAlive():

        cantidad_hijos_reejecutar=1

        while cantidad_hijos_reejecutar != 0:
            hijos_no_ejecutados=[]
            hijos_ejecutados=[]
            for line in lista_de_datos:
                if len(line)<10:
                    hijos_no_ejecutados.append(line.split("robot")[-1])
                else:
                    hijos_ejecutados.append(line)
            cantidad_hijos_reejecutar=len(hijos_no_ejecutados)
            lista_de_datos=hijos_ejecutados
            print(hijos_no_ejecutados)
            for str_num_hilo in hijos_no_ejecutados:
                num_hilo=int(str_num_hilo)
                hilo = threading.Thread(name='hilo%s' %num_hilo, 
                                        target=call_robot,args = (num_hilo,time_eje,dic_peso[str(num_hilo)][0],dic_peso[str(num_hilo)][1]))    
                
                hilo.start()
                
                hilos_todos.append(hilo)
                
            for hilo in hilos_todos:    
                hilo.join() 
            
        

        print("cantidad de datos:",len(lista_de_datos))
        f = open (file,'a')
        f.write("generacion\n")
        for line in lista_de_datos:
            f.write(line)
        f.close()
        #f = open (file,'r')
        #mensaje = f.read()
        #mensaje=mensaje.split("inicio")
        #mensaje=mensaje[-1].split("generacion")
        for line in lista_de_datos:
            get_info_robot(dic_p_padres,line)
    
        
        #print(dic_p_padres)
        if len(dic_p_padres)>0:

            get_pesos_new_generation(dic_p_padres,dic_peso,cont_ciclos,filef)
        #print(mensaje.split("inicio"))
        #print(mensaje.split("\n"))
        #print(dic_peso)
        #f.close()
        cont_ciclos+=1
        rate.sleep()

    
  
    
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
  