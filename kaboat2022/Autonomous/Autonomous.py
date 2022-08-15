#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

goalPos = [8.5, 0]
Pos = [0, 0]
Psi = 0
BoatWidth = 1

endRange = 1
avoidRange = 3
maxRange = 20

Gain_Psi = 1
Gain_Distance = 1


lidardata = [0] * 360

ax = plt.subplot(111, polar=True)

data_range = np.linspace(0,2*np.pi, 360)
plt.gcf().set_facecolor((0, 0.5, 1, 0.1))

def sigmoid(x):
    return 1/(exp(-x) + 1)

def callback(data1, data2, data3):
    global lidardata, Pos, Psi
    lidardata = data1.data
    Psi = data2.data
    Pos[0] = data3.data[0]
    Pos[1] = data3.data[1]



def animate(i):
    # print(lidardata)
    lidarinv = [0] * 360
    ld = np.array(lidardata)
    for i in range(61, 300):
        ld[i] = 0
    for i in range(360):
        if(ld[i] == 0):
            lidarinv[i] = maxRange

    ld = np.flip(ld)



    
    Goal_Distance = np.sqrt(np.power(goalPos[0] - Pos[0], 2) + np.power(goalPos[1] - Pos[1], 2))
    Goal_Psi = np.arctan2(goalPos[0] - Pos[0], goalPos[1] - Pos[1]) * 180 / np.pi - Psi
    Goal_Psi = int(Goal_Psi)
    
    if(Goal_Psi < -180): Goal_Psi += 360
    elif(Goal_Psi > 180): Goal_Psi -= 360

    ##### Ax Setting #####
    ax.clear()
    ax.set_theta_zero_location('N') 
    ax.set_theta_direction(-1)
    ax.axis([-np.pi/2, np.pi/2, 0, maxRange])

    ##### Lidar Plot #####
    ax.plot(data_range, ld, 'o', color='r', markersize = 1)
    ax.fill(data_range,ld, color = [0,0,1,0.1])
    ax.fill(data_range,lidarinv,'0.9')


    ##### Boat Plot #####
    ax.plot(0,0,'s', color = [0,0,1,0.5], markersize = 10)
    
    ##### Goal Plot #####
    # ax.plot([0, Goal_Psi * np.pi / 180], [0, Goal_Distance], color = "green", markersize = 5)
    ax.plot(Goal_Psi * np.pi / 180, Goal_Distance,'s', color = "green", markersize = 10)

    #######################################################################################################
    # safeZone[i] > 0  : Safe
    # safeZone[i] == 0 : Danger

    safeZone = [avoidRange] * 360
    
    for i in range(-60, 61):
        if(ld[i]!=0 and ld[i] < avoidRange):
            safeZone[i] = 0

    temp = np.array(safeZone)
    for i in range(-60, 61):
        if(safeZone[i] > safeZone[i + 1]): ### 
            for j in range(floor(i + 1 - np.arctan2(BoatWidth/2, ld[i + 1]) * 180 / np.pi),i + 1):
                temp[j] = 0
        if(safeZone[i] < safeZone[i + 1]): ### 
            for j in range(i, ceil(i + np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi) + 1):
                temp[j] = 0
    safeZone = temp

    #######################################################################################################
    
    thetaList = []
    thetaList = [[-61, 1000 * abs(-61-Goal_Psi)], [61, 1000 * abs(61-Goal_Psi)]]
    for i in range(-60, 61):
        if(safeZone[i] > 0):
            # cost = (1 / Gain_Psi) * np.log(abs(i - Goal_Psi) + 1) + (1 / Gain_Distance) * sigmoid(1-(ld[i]/avoidRange))
            cost = (1 / Gain_Psi) * sigmoid(abs(i - Goal_Psi)/60) + (1 / Gain_Distance) * sigmoid(1 - ld[i]/avoidRange)
            thetaList.append([i, cost])
    
    thetaList = sorted(thetaList, key = lambda x : x[1])
    Psi_d = thetaList[0][0]

    if(ld[Goal_Psi] > Goal_Distance):
        isSafe = True
        for j in range((floor(Goal_Psi  - np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi)),(ceil(Goal_Psi + np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi))+1):
            if(ld[j] < Goal_Distance):
                isSafe = False
        if(isSafe):
            Psi_d = Goal_Psi
                    

    
    ax.fill(data_range, safeZone, color = [0,1,0,0.5])
    ax.plot([0,Psi_d * np.pi / 180], [0,5], color = [1,0,1,1])

    msg = Float32()
    msg.data = Psi_d + Psi

    if((goalPos[0]-Pos[0])*(goalPos[0]-Pos[0]) + (goalPos[1]-Pos[1])*(goalPos[1]-Pos[1]) < endRange * endRange):
        msg.data = -10000
        

    pub.publish(msg)
    print(msg)


if __name__=="__main__":
    rospy.init_node('LidarPython11',anonymous = False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    sub1 = message_filters.Subscriber("/LidarData", Float64MultiArray)
    sub2 = message_filters.Subscriber("/IMUData", Float32)
    sub3 = message_filters.Subscriber("/UTM", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)
    ani = FuncAnimation(plt.gcf(), animate, interval = 50)
    plt.show()
