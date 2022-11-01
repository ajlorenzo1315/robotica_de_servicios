#!/usr/bin/env python

# SE ENTREGA!!!!!!!


from ast import For
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np 
from std_msgs.msg import String

import os
#os.system('rostopic list')
import sys, time, threading, operator, random

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
    time.sleep(1)

    os.system("".join(['rosservice call /gazebo/set_model_state ',"'",'{','model_state:','{','model_name:',' robot', str(robot_ind),
    ', pose: { position: ',positions[robot_ind-1], ', orientation: ' , '{' , 'x: 0, y: 0, z: 0, w: 0 } }',
    ', twist: { linear: {x: 0.0 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } ','}',"'" ]))
       
    #os.system("".join(['rosservice call gazebo/delete_model ','{','model_name:',' robot', str(robot_ind),'}' ]) )
    #os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/robot/model.sdf -sdf -model',' robot',str(robot_ind),' -y 0.2 -x -0.3'] ))
    time.sleep(1)

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

def spawn_robot_and_map(robot_ind, robots_por_linea):
    x = -10 * ((robot_ind - 1) // robots_por_linea)
    y = -15 * ((robot_ind - 1) % robots_por_linea)
    positions_robot = " ".join(["-x", str(x - 1.5), "-y", str(y), "-z 0 "])
    positions_map = " ".join(["-x", str(x), "-y", str(y), "-z 0 "])
    print(positions_robot,positions_map)
    os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/robot/model.sdf -sdf -model',
    ' robot',str(robot_ind)," ",positions_robot] ))
    os.system("".join(['rosrun gazebo_ros spawn_model -file `echo $GAZEBO_MODEL_PATH` ~/servicios/src/p1servizos/models/default_maze/model.sdf -sdf -model',
    ' default_maze',str(robot_ind)," ",positions_map] ))
    return " ".join(["{ x:", str(x-1.5), ", y:", str(y), ", z: 0 }"])


def inicializar(n_x, n_h):
    """
    Argumentos:
    n_x -- n neuronas de la capa de entrada
    n_h -- n neuronas de la capa oculta
    n_y -- n neuronas de la capa de salida
    semilla -- semilla  para realizar todos la misma inicializacion
    mult -- para que los valores sean float o anular el valor a cero
    
    Returns:
    parametros --  diccionario que conten:
                    W1 -- matriz de pesos (n_h, n_x)
                    b1 -- vector de bias (n_h, 1)
                    W2 -- matriz de pesos (n_y, n_h)
                    b2 -- vector de bias  (n_y, 1)
    """
    # Se definen las matrices y los vectores
    parametros = {}
    W1 = np.random.uniform(low = -1, high = 1, size = (n_h, n_x)) #np.random.randn(n_h, n_x)
    #W1 = np.zeros((n_h, n_x))
    b1 = np.random.uniform(low = -1, high = 1, size = (n_h, 1)) #np.random.randn(n_h, 1)
    
    # Se guardan los parametros en un diccionario
    parametros = {"W1": W1,
                  "b1": b1}

    return parametros


def inicilizar_pesos_robots(num_robots):
    num_sensores = 8
    salida = 2
    robot_pesos = []
    for rob in range(num_robots):
        parametros = inicializar(num_sensores,salida)
        # Recuperamos los valores de la matriz de pesos (W1) y el vector con los bias (b1)
        W1 = parametros["W1"]
        b1 = parametros["b1"]
        W1_todo_peso = []
        for j in range(len(W1)):
            W1str = [str(i) for i in W1[j]]
            W1_todo_peso.append(",".join(W1str))
        W1_todo_peso = "_".join(W1_todo_peso)

        b1_todo_peso = []
        for j in range(len(b1)):
            b1str = [str(i) for i in b1[j]]
            b1_todo_peso.append("_".join(b1str))
        b1_todo_peso = ",".join(b1_todo_peso)

        robot_pesos.append("/".join([str(rob+1),W1_todo_peso,b1_todo_peso,"0",'\n']))
        #print(robot_pesos)
        time.sleep(1) 

    return robot_pesos


def get_best_fitness(dic_pesos_padres, ind_fitness):
    fitness = list(np.array(list(dic_pesos_padres.values()))[:, ind_fitness])
    dicionario = dict(zip(list(dic_pesos_padres.keys()), fitness))
    valores_ord = dict(sorted(dicionario.items(), key = operator.itemgetter(1), reverse = True))
    return valores_ord


def pesos_str_to_np(z):
    W1 = []
    for x in z.split("_"):
        #print(x)
        #print(x.split(','))
        W1.append([float(y) for y in x.split(',')])
    return W1


def pesos_np_to_str(W1):
    W1_todo_peso = []
    for j in range(len(W1)):
        W1str = [str(i) for i in W1[j]]
        W1_todo_peso.append(",".join(W1str))
    W1_todo_peso = "_".join(W1_todo_peso)
    #print(W1_todo_peso)
    return W1_todo_peso


def cruce_aleatorio(padre_1,padre_2,num_div):
    padre_1 = pesos_str_to_np(padre_1)
    padre_2 = pesos_str_to_np(padre_2)
    #print(padre_1)
    
    hijo = [[],[]]
    if num_div == 0:
        num_div = 1

    cant_div = len(padre_1[0]) // 3
    resto = len(padre_1[0]) - cant_div - 1 * 3
    #print(resto)
    for ind in range(0, cant_div):
        if ind%2 == 0:
            hijo[0].extend(padre_1[0][ind * cant_div:(ind + 1) * cant_div + 1])
            hijo[1].extend(padre_1[1][ind * cant_div:(ind + 1) * cant_div + 1])
        else:
            hijo[0].extend(padre_2[0][ind * cant_div:(ind + 1) * cant_div + 1])
            hijo[1].extend(padre_2[1][ind * cant_div:(ind + 1) * cant_div + 1])

    if cant_div%2 == 0:
        hijo[0].extend(padre_2[0][(ind + 1) * cant_div + 2:(ind + 1) * cant_div + resto + 2])
        hijo[1].extend(padre_2[1][(ind + 1) * cant_div + 2:(ind + 1) * cant_div + resto + 2])
    else:
        hijo[0].extend(padre_1[0][(ind + 1) * cant_div + 2:(ind + 1) * cant_div + resto + 2])
        hijo[1].extend(padre_1[1][(ind + 1) * cant_div + 2:(ind + 1) * cant_div + resto + 2])

    return pesos_np_to_str(hijo)


def cruce_angulo_lineal(padre_1,padre_2,mutacion):
    padre_1 = pesos_str_to_np(padre_1)
    padre_2 = pesos_str_to_np(padre_2)

    mutaciones = np.zeros((4, len(padre_1[0])))
    print("\nMutacion",mutaciones.shape)

    for i in range (0, 4):
        muta = random.choice(mutacion)
        for j in range(0, muta + 1):
            ind_mutados = np.random.choice(range(0, len(padre_1[0])), muta,False)
            cuanto_muta = np.random.uniform(low=-1, high=1, size=(1))[0]/10
            mutaciones[i][ind_mutados] = cuanto_muta
    
    hijo1 = [padre_1[0] + mutaciones[0], padre_2[1] + mutaciones[1]]
    hijo2 = [padre_2[0] + mutaciones[2], padre_1[1] + mutaciones[3]]

    return pesos_np_to_str(hijo1), pesos_np_to_str(hijo2)


def bias_cruce(padre_1,padre_2):
    b_padre_1 = padre_1.split(",")
    b_padre_2 = padre_2.split(",")

    hijo_1 = ",".join([b_padre_1[0], b_padre_2[1]])
    hijo_2 = ",".join([b_padre_2[0], b_padre_1[1]])

    return hijo_1, hijo_2


def propabilidades_padres_list(valores_ord):

    #padres_list=list(valores_ord.keys())
    
    padres_list = list(valores_ord.keys())
    total = float(valores_ord[padres_list[0]])
    longitud = len(padres_list)
    #print(total)
    pobrabilidades = [longitud * (float(valores_ord[padres_list[i]])) / total * (np.e**-(i / longitud)) for i in range(len(padres_list))]
    
    return pobrabilidades, padres_list


def generar_lista_de_probabilidades(elementos, pesos):
    resultado = []
    for e, p in zip (elementos, pesos):
        resultado += [e] * int(p * 100)
    return resultado


def funcion_lerning_rate(ciclo):
    if ciclo == 0:
        ciclo = 0.000000000000000001
    ciclo = ciclo / 5
    return 8 * (np.e**-ciclo)


def mutaciones_probabilidades(ciclo):
    elementos = [0, 1, 2, 3, 4, 5, 6, 7, 8]
    lr = funcion_lerning_rate(ciclo)
    pesos2 = lr * np.array([0.5, 0.2, 0.1, 0.1, 0.1, 0.05, 0.02, 0.01])
    pesos = np.append(np.array([2]), pesos2)
    resultado = generar_lista_de_probabilidades(elementos,pesos)
    return resultado


def get_pesos_new_generation(dic_pesos_padres, dic_pesos_new, ciclo):

    dic_ord_pdr_fitness = get_best_fitness(dic_pesos_padres, 2)
    #print("diccionario",dic_ord_pdr_fitness)
    
    pobrabilidades, padres_list = propabilidades_padres_list(dic_ord_pdr_fitness)
    print("\n------------------------------------------")
    print(pobrabilidades)
    print("------------------------------------------")
    print(padres_list)
    resultado = generar_lista_de_probabilidades(padres_list, pobrabilidades)
    mutacion = mutaciones_probabilidades(ciclo)
    #print(dic_ord_pdr_fitness.keys())
    lista_hijos = list(dic_pesos_padres.keys())

    for i in range(len(lista_hijos) // 2):
        padre_1 = random.choice(resultado)
        padre_2 = random.choice(resultado)
        while padre_2 == padre_1:
            padre_2 = random.choice(resultado)

        #print(padre_1,)
        padre_1p = dic_pesos_padres[padre_1][0]
        padre_2p = dic_pesos_padres[padre_2][0]
        
        #print(padre_1p,padre_2p)
        hijo1, hijo2 = cruce_angulo_lineal(padre_1p, padre_2p, mutacion)
        #print(lista_hijos[i*2+1],lista_hijos[i*2])
        dic_pesos_new[lista_hijos[i * 2]][0] = hijo1
        dic_pesos_new[lista_hijos[i * 2 + 1]][0] = hijo2

        padre_1p = dic_pesos_padres[padre_1][1]
        padre_2p = dic_pesos_padres[padre_2][1]

        hijo1, hijo2 = bias_cruce(padre_1p, padre_2p)
        dic_pesos_new[lista_hijos[i * 2]][1] = hijo1
        dic_pesos_new[lista_hijos[i * 2 + 1]][1] = hijo2

        #print(i)
        #hacemos por el momento para buscar padre
        #if str(int(i)-1) in dic_pesos_padres:
        #    padre_2=dic_pesos_padres[str(int(i)-1)][0]
        #else:
        #    padre_2=dic_pesos_padres[str(int(i)+1)][0]
        #print(i,padre_2)
        #dic_pesos_new[i][0]=cruce_aleatorio(dic_pesos_padres[i][0],padre_2,np.random.randint(8))

    #print("hijos",dic_pesos_new)


# Consigue los pesos del la generacion anterior del padre
def get_info_robot(dic_peso, line):
    line = line.split("/")
    #print(line)
    dic_peso[line[0]] = [line[1], line[2], float(line[3])]


lista_de_datos = []


def feedback(msg):
    global lista_de_datos
    lista_de_datos.append(msg.data)
    #print(lista_de_datos)


positions = ['{ x: -1.5, y: 0 ,z: 0 }','{ x: -1.5, y: -15 ,z: 0 }','{ x: -1.5, y: -30 ,z: 0 }']

#genes_mutados_elementos = ['0', '1', '2', '3', '4', '5', '6', '7', '8']
#pesos_genes_mutado = [2, 0.5, 0.2, 0.1, 0.1, 0.1, 0.05, 0.02, 0.01]

def talker():
    global lista_de_datos
    rospy.init_node('control')

    pub1 = rospy.Publisher("datos_fin", String, queue_size=1)
    sub = rospy.Subscriber("/datos_fin", String, feedback)

    ROBOTS = 8
    num_robot_por_lineas = 4

    if ROBOTS > 3:
        for i in range(4, ROBOTS + 1):
            #print(-15(*(i-1)%3))
            positions.append(spawn_robot_and_map(i, num_robot_por_lineas))

    
    dic_peso = {} # Diccionario de pesos _de ejecucuion
    dic_p_padres = {} # Diccionrio pesos de los padres
    # Texto de padres
    file = os.path.realpath(__file__).split("/")
    file[-1] = 'robot.txt'
    file="/".join(file)
    #print(file)
    f = open (file,'w')
    textos = inicilizar_pesos_robots(ROBOTS)
    f.write("inicio\n")
    for line in textos:
        f.write(line)
    f.close()
    # archivo-entrada.py
    f = open (file,'r')
    mensaje = f.read()
    mensaje = mensaje.split("inicio")
    #print(mensaje.split("inicio"))
    #print(mensaje.split("\n"))
    f.close()
    
    for line in mensaje[-1].split("\n")[1:-1]:
        get_info_robot(dic_peso,line)

    time_eje = 60

    cont = len(lista_de_datos)    
    cont_ciclos = 0
    rate = rospy.Rate(time_eje) # 10hz
    while not rospy.is_shutdown():
        lista_de_datos = []
        hilos_todos = []
        #f = open (file,'a')
        #f.write("generacion\n")
        for num_hilo in range(1, ROBOTS + 1):
            hilo = threading.Thread(name = 'hilo%s' %num_hilo, 
                                    target = call_robot, args = (num_hilo, time_eje, positions, dic_peso[str(num_hilo)][0], dic_peso[str(num_hilo)][1]))    
            hilo.setDaemon(True)
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

        rate.sleep()

        print("\nCantidad de datos:", len(lista_de_datos))
        f = open (file,'a')
        f.write("Generacion\n")
        for line in lista_de_datos:
            f.write(line)
        f.close()
        time.sleep(1)
        f = open (file,'r')
        mensaje = f.read()
        mensaje = mensaje.split("Inicio")
        mensaje = mensaje[-1].split("Generacion")
        for line in mensaje[-1].split("\n")[1:-1]:
            get_info_robot(dic_p_padres, line)

        #print(dic_p_padres)
        if len(dic_p_padres) > 0:
            get_pesos_new_generation(dic_p_padres, dic_peso, cont_ciclos)
        #print(mensaje.split("inicio"))
        #print(mensaje.split("\n"))
        #print(dic_peso)
        f.close()
        cont_ciclos += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass