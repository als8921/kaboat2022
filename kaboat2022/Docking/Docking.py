#!/usr/bin/env python3

import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

Docking_theta = -80
Docking_Delta = 2
Docking_Angle = 0

goalPos = [7.82, 1.74]
end_range = 2
DockFinal = []
DockPos = []

Pos = [0, 0]
Psi = 0
BoatWidth = 0.8

obs_range = 0.5
maxRange = 20

lidardata = [0] * 360

ax = plt.subplot(111, polar=True)

data_range = np.linspace(0,2*np.pi, 360)
plt.gcf().set_facecolor((0, 0.5, 1, 0.1))
def sigmoid(x):
    return 1/(exp(-x) + 1)

def callback(data1, data2, data3, data4):
    global lidardata, Pos, Psi, Docking_Angle
    lidardata = data1.data
    Psi = data2.data
    Pos[0] = data3.data[0]
    Pos[1] = data3.data[1]
    Docking_Angle = data4.data



def animate(i):
    global DockFinal, DockPos

    lidarinv = [0] * 360
    ld = np.array(lidardata)
    # ld = np.flip(ld)
    for i in range(61, 300):
        ld[i] = 0
    for i in range(360):
        if(ld[i] == 0):
            lidarinv[i] = maxRange

    obs = []
    for i in range(-60, 61):
        if(ld[i] - ld[i+1] > obs_range):
            obs.append(i+1)
        elif(ld[i] - ld[i+1] <-obs_range):
            obs.append(i)

    Docklist = []
    for i in range(len(obs) - 1):
        if(abs(obs[i] - obs[i + 1]) > 7 and abs(ld[obs[i]] - ld[obs[i + 1]]) < 3):
            Docklist.append([(obs[i] + obs[i + 1])/2, (ld[obs[i]] + ld[obs[i + 1]])/2])

    Docklist = np.array(Docklist)
    ax.clear()
#################


    if(DockFinal != []):
        theta, r = DockFinal
        print(DockPos)
        y_e = (Pos[0] - DockPos[0] + 5 * np.sin(Docking_theta * np.pi / 180)) * np.cos(Docking_theta * np.pi / 180) - (Pos[1] - DockPos[1] + 5 * np.cos(Docking_theta * np.pi / 180)) * np.sin(Docking_theta * np.pi / 180)
        DockPsi = Docking_theta - np.arctan(y_e / Docking_Delta) * 180 / np.pi
        ax.plot([0, (DockPsi-Psi) * np.pi / 180], [0,100], color = "green")
        
        if((DockPos[0]-Pos[0])**2 + (DockPos[1]-Pos[1])**2 < end_range ** 2):
            DockPsi = -10000
            
        msg = Float32()
        msg.data = DockPsi
        pub.publish(msg)

        
    else:  
        if((goalPos[0]-Pos[0])**2 + (goalPos[1]-Pos[1])**2 < end_range ** 2):
            if(Docklist!=[]):

                Docklist = sorted(Docklist, key = lambda x:abs(x[0]-Docking_Angle))
                print(Docklist)
                DockFinal = Docklist[0]
                theta, r = DockFinal
                DockPos = (Pos[0]+r*np.sin((Psi+theta)*np.pi/180), Pos[1]+r*np.cos((Psi+theta)*np.pi/180))


    ##### Ax Setting #####
    # ax.clear()
    ax.set_theta_zero_location('N') 
    ax.set_theta_direction(-1)
    ax.axis([-np.pi/2, np.pi/2, 0, maxRange])

    ##### Lidar Plot #####
    ax.plot(data_range, ld, 'o', color='r', markersize = 1)
    ax.fill(data_range,ld, color = [0,0,1,0.1])
    ax.fill(data_range,lidarinv,'0.9')



    ##### Docking Zone Plot #####
    for i in obs:
        ax.plot(i * np.pi / 180, ld[i], "ro")


    for i, j in Docklist:
        ax.plot([i * np.pi / 180], [j], "bo", markersize = 10)
        

    ##### Boat Plot #####
    ax.plot(0,0,'s', color = [0,0,1,0.5], markersize = 10)
    
   
    



if __name__=="__main__":
    rospy.init_node('LidarPython11',anonymous = False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    pub1 = rospy.Publisher("thetaList", Float64MultiArray, queue_size=10)
    sub1 = message_filters.Subscriber("/LidarData", Float64MultiArray)
    sub2 = message_filters.Subscriber("/IMUData", Float32)
    sub3 = message_filters.Subscriber("/UTM", Float64MultiArray)
    sub4 = message_filters.Subscriber("/vision", Float32)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3, sub4],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)
    ani = FuncAnimation(plt.gcf(), animate, interval = 100)
    plt.show()