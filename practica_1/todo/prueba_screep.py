#import subprocess
#from subprocess import call
#print(call("rostopic list", shell=True))  
import os
#os.system('rostopic list')
import sys

def call_robot(robot_ind,limit):
    for i in range(0,limit):
        os.system("".join(['rosrun p1 robot_comun.py -r ','robot',str(robot_ind), ' -t 60 ']))



import threading


ROBOTS = 3
hilos_todos=[]
for num_hilo in range(1,ROBOTS+1):
    hilo = threading.Thread(name='hilo%s' %num_hilo, 
                            target=call_robot,args = (num_hilo,3, ))    
    hilo.start()
    hilos_todos.append(hilo)
    #hilo.join() 
    print("thread finished...exiting") 

for hilo in hilos_todos:    

    hilo.join() 