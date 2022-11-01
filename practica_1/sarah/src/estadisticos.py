#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
import numpy as np 
import pylab


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DISTANCIAS TOTAL PERCORRIDA POR EL ROBOT ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def dist_recorrida(x, y, topic_robot):
    first_run = True
    total_distance = 0

    if first_run:
        previous_x = x
        previous_y = y

    x = x
    y = y
    d_increment = np.sqrt((x - previous_x) * (x - previous_x) +
                          (y - previous_y)(y - previous_y))
    total_distance = total_distance + d_increment

    print("\nDISTANCIA TOTAL RECORRIDA: ", total_distance)

    rospy.Publisher("/".join([topic_robot, "odom"]), Odometry, queue_size=10) 
    previous_x = x
    previous_y = y
    first_run = False 

    return total_distance


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GRAFICA DISTANCIA RECORRIDA ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def grafica_dist_recorrida(total_distance):
    fig, ax1 = pylab.subplots()

    ax1.plot(range(1, 21), total_distance)
    ax1.axhline(32, ls='dashed')

    ax1.set_ylim(0, 35)

    ax1.set_xlabel('Iteraciones')
    ax1.set_ylabel('Distancia total')
    ax1.set_title('Evolucion distancia recorrida')
    #ax1.legend(('Mean', 'Best', 'Max'), 'best')

    fig.savefig('fitness_evo.png')
    pylab.show()



# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ DISTANCIA MEDIA/MINIMA DE OBSTACULOS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def dist_med_min_obstaculos(r1_dsf, r1_dsb):
    dist_media = []
    dist_minina_list = []
    dist_minima_front = []
    global dist_minima_front_l, dist_minima_front_r
    dist_minima_front_l = []
    dist_minima_front_r = []
    seguridad_dist = 0.45

    while not rospy.is_shutdown():
        X = np.array([r1_dsf + r1_dsb ]).T
        dist_minina_list.append(np.min(X))
        dist_minima_front.append(np.min(np.array(r1_dsf)))
        dist_minima_front_l.append(np.min(np.array(r1_dsf[:len(r1_dsf)//2 ])))
        dist_minima_front_r.append(np.min(np.array(r1_dsf[len(r1_dsf)//2:] )))

        distancias = dist_minima_front + dist_minima_front_l + dist_minima_front_r
        dist_media.append(sum(distancias)/len(distancias)) 
        
        if dist_minima_front[-1] <= seguridad_dist:
            dist_minima = dist_minina_list[-1]
            print("\nDISTANCIA MINIMA", dist_minima)

        print("\nDISTANCIA MEDIA A OBSTACULOS: ", dist_media)

    return dist_minima, dist_media

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GRAFICA DISTANCIAS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def grafica_distancias(dist_minima, dist_media):
    fig, ax1 = pylab.subplots()

    ax1.plot(range(1, 21), dist_minima)
    ax1.plot(range(1, 21), dist_media)
    ax1.axhline(32, ls='dashed')

    ax1.set_ylim(0, 35)

    ax1.set_xlabel('Iteraciones')
    ax1.set_ylabel('Distancia')
    ax1.set_title('Evolucion distancias')
    #ax1.legend(('Mean', 'Best', 'Max'), 'best')

    fig.savefig('dist_evo.png')
    pylab.show()

# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ EVOLUCION DEL FITNESS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def evolucion_fitness(fitness_dic):
    mean_fitness = []
    best_fitness = []
    best_weights = []

    for robot in len(fitness_dic):
        robot += 1 # Habra un total de robots segun el total del diccionario

    for fitness, weights in range(fitness_dic.iteritems()):
        # ZIP() combinamos el contenido de la fitness con cada robot
        a = zip(fitness, robot)

        _, best_robot = max(zip(fitness, robot))

    mean_fitness.append(np.mean(fitness))
    best_fitness.append(np.max(fitness))
    best_weights.append(np.array())

    return mean_fitness, best_fitness, best_weights


# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ GRAFICA FITNESS ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

def grafica_fitness(mean_fitness, best_fitness):
    fig, ax1 = pylab.subplots()

    ax1.plot(range(1, 21), mean_fitness)
    ax1.plot(range(1, 21), best_fitness)
    ax1.axhline(32, ls='dashed')

    ax1.set_ylim(0, 35)

    ax1.set_xlabel('Iteraciones')
    ax1.set_ylabel('Fitness')
    ax1.set_title('Evolucion Fitness')
    ax1.legend(('Mean', 'Best', 'Max'), 'best')

    fig.savefig('fitness_evo.png')
    pylab.show()

