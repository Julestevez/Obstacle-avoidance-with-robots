#Obstacle avoidance
#Potential Field

#************* Autonomous drones crossing in 2D and self avoiding among them ******
#************ We'll create a circular potential around each drone. Diameter is proportional to its horizontal speed*****
#******** drones transporting ropes will be included, so that it makes the obstacle avoidance more difficult *******

import numpy as np
import matplotlib.pyplot as plt
import random
import math


i=0
k=0.02 #constante de atracción
k_r = 0.6 #constante de repulsión
dt=2


#Potential field proportional to the speed
#Radius= 2 + (speed-1) #[m/s]


#initial point
#frame of 100x100m



#el drone 1 va de abajo a arriba y1=0, y1_end=100
#el drone 2 va de izquierda a derecha, x2=0, x2_end=100
#figure, axes = plt.subplots()

while True:

    #creation of up-down trajectories
    x_start=random.uniform(0, 40) #elijo aleatoriamente inicio y fin
    x_end=random.uniform(0, 40)
    x1=x_start
    x1_end=x_end
    y1=0
    y1_end=40
    #x1=np.linspace(x_start,x_end,50)
    #y1=np.linspace(0,100,50)

    #creation of left-right trajectories
    y_start=random.uniform(0, 40) #elijo aleatoriamente inicio y fin
    y_end=random.uniform(0,40)
    x2=0
    x2_end=40
    y2=y_start
    y2_end=y_end
    #x2=np.linspace(0,100,50)
    #y2=np.linspace(y_start,y_end,50)


    #MOVEMENT
    #attractive potential FIRST DRONE
    v_x_a1 = -k * (x1 - x1_end)
    v_y_a1 = -k * (y1 - y1_end)

    #attractive potential SECOND DRONE
    v_x_a2 = -k * (x2 - x2_end)
    v_y_a2 = -k * (y2 - y2_end)
  

    ro=5
    dist=math.sqrt((x2-x1)**2 + (y2-y1)**2)

    while y1<(y1_end-5) and x2<(x2_end-5):
        v_x_a1 = -k * (x1 - x1_end)
        v_y_a1 = -k * (y1 - y1_end)

        v_x_a2 = -k * (x2 - x2_end)
        v_y_a2 = -k * (y2 - y2_end)


        #repulsive potential
        if (abs(x2-x1)<8 and abs(y2-y1)<8):
            v_x_r1= -k_r * (1- dist/ro)*(x1-x2)/ro**3
            v_y_r1= -k_r * (1- dist/ro)*(y1-y2)/ro**3

            v_x_r2= -k_r * (1- dist/ro)*(x2-x1)/ro**3
            v_y_r2= -k_r * (1- dist/ro)*(y2-y1)/ro**3
        else:
            v_x_r1, v_y_r1= 0, 0
            v_x_r2, v_y_r2= 0, 0

        v_x_total1 = v_x_a1 + v_x_r1
        v_y_total1 = v_y_a1 + v_y_r1

        v_x_total2 = v_x_a2 + v_x_r2
        v_y_total2 = v_y_a2 + v_y_r2

        x1 = x1 + v_x_total1*dt
        y1 = y1 + v_y_total1*dt

        x2 = x2 + v_x_total2*dt
        y2 = y2 + v_y_total2*dt



        plt.clf()
        axes = plt.gca()
        axes.set_xlim([0,40])
        axes.set_ylim([0,40])
        plt.plot(x1,y1,'oy',markersize=20)
        plt.plot(x2,y2,'ob',markersize=20)
        #circle1=plt.Circle((x1[i],y1[i]),3,color='blue')
        #circle2=plt.Circle((x2[i],y2[i]),3,color='red')
        #axes.add_artist(circle1)
        #axes.add_artist(circle2)
        #legend(loc=0)
       
        




        """#### agrego formación de 3 ###
        alfa = 3.14 # the follower drone must be behind the leader
        temp = x2[i]-x2[i-1]
        beta= math.atan((y2[i]-y2[i-1])/temp)
        #if (y2[i]-y2[i-1]<0):
        #    beta=3.14/2+beta
        landaX=10*0.85*math.cos(alfa)
        landaY=10*0.85*math.sin(alfa)
        x_f1=x2[i] + landaX*math.cos(beta) - landaY*math.sin(beta)
        y_f1=y2[i] + landaX*math.sin(beta) + landaY*math.cos(beta)

        #new follower (falta crear su beta)
        x_f2=x_f1 + landaX*math.cos(beta) - landaY*math.sin(beta)
        y_f2=y_f1 + landaX*math.sin(beta) + landaY*math.cos(beta)

        plt.plot(x_f1,y_f1,'ob')
        plt.plot(x_f2,y_f2,'og')"""


        plt.pause(0.1)
  
plt.show()

    

"""def Follower(States,Old_states):
            
    alfa=150*3.14/180

    temp= (States[0]-Old_states[0])
    if temp==0: 
        temp=0.1
    
    beta = math.atan((States[1]-Old_states[1])/(temp))
    landaX=60*0.85*math.cos(alfa)
    landaY=60*0.85*math.sin(alfa)
    x_pos_desired=States[0] + landaX*math.cos(beta) - landaY*math.sin(beta)
    y_pos_desired=States[1] - landaX*math.sin(beta) - landaY*math.cos(beta) """






