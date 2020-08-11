#Obstacle avoidance
#Potential Field

#************* Autonomous drones crossing in 2D and self avoiding among them ******
#************ We'll create a circular potential around each drone. Diameter is proportional to its horizontal speed*****
#******** drones transporting ropes will be included, so that it makes the obstacle avoidance more difficult *******

import numpy as np
import matplotlib.pyplot as plt
import random

i=0

#Potential field proportional to the speed
#Radius= 2 + (speed-1) #[m/s]


#initial point
#frame of 100x100m

start_point= [0,0]
end_point= [100,100]
#speed = 1

x=np.linspace(0,100,100)
y=np.linspace(0,100,100)

#aleatory number
#dots crossing vertically 
x_start=random.uniform(0, 100)
x_end=random.uniform(0, 100)

#dots crossing horizontally
y_start=random.uniform(0, 100)
y_end=random.uniform(0,100)

#down-up paths
x1=np.linspace(x_start,x_end,50)
y1=np.linspace(0,100,50)

#left-right paths
x2=np.linspace(0,100,50)
y2=np.linspace(y_start,y_end,50)


figure, axes = plt.subplots()

while True:

    #creation of up-down trajectories
    x_start=random.uniform(0, 100)
    x_end=random.uniform(0, 100)
    x1=np.linspace(x_start,x_end,50)
    y1=np.linspace(0,100,50)

    #creation of left-right trajectories
    y_start=random.uniform(0, 100)
    y_end=random.uniform(0,100)
    x2=np.linspace(0,100,50)
    y2=np.linspace(y_start,y_end,50)

    for i in range(50):
        plt.clf()
        axes = plt.gca()
        axes.set_xlim([0,100])
        axes.set_ylim([0,100])
        plt.plot(x1[i],y1[i],'oy')
        plt.plot(x2[i],y2[i],'ob')
        circle1=plt.Circle((x1[i],y1[i]),3,color='blue')
        circle2=plt.Circle((x2[i],y2[i]),3,color='red')
        axes.add_artist(circle1)
        axes.add_artist(circle2)
        plt.pause(0.1)
  
plt.show()

    







