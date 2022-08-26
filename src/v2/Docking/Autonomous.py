#!/usr/bin/env python3
import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

Docking_theta = -10
Docking_Delta = 2.5
Docking_Angle = -10000

DockFinal = []
# DockFinal = [-16.33, 18.14]
# DockFinal = [-13.23, 19.24]
# DockFinal = [-10.30, 20.33]


DockPos = [[-16.33, 18.14], [-13.23, 19.24], [-10.30, 20.33]]
DockCnt = [0,0,0]

# goalPos = [-10.72, 8.917]
goalPos = [-11.53, 11.69]
# goalPos = [-8.53, 11.69]

Pos = [0, 0]
Psi = 0
Past_Psi_d = 0

BoatWidth = 0.9

endRange = 2
avoidRange = 4
maxRange = 30

Gain_Psi = 1
Gain_Distance = 8
Gain_Diff = 0

dt = 50 #ms
lidardata = [0] * 360

ax = plt.subplot(111, polar=True)

data_range = np.linspace(0,2*np.pi, 360)
plt.gcf().set_facecolor((0, 0.5, 1, 0.1))

def CostFuncAngle(x):
    x = abs(x)
    if(x <= 10):
        return 0.01 * x
    else:
        return 0.1*(x - 10)

def CostFuncDistance(x):
    if(x > 7):
        return 0
    else:
        return 10 - 10/7 * x

def CostFuncDiff(x):
    return abs((x + Psi - Past_Psi_d)%180)/dt

def sigmoid(x):
    return 1/(exp(-x) + 1)

def callback(data1, data2, data3):
    global lidardata, Pos, Psi
    lidardata = data1.data
    Psi = data2.data
    Pos[0] = data3.data[0]
    Pos[1] = data3.data[1]



def animate(i):
    global Past_Psi_d, DockFinal, Docking_Angle, DockCnt
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
    thetaList = [[-61, 10000 * abs(-61-Goal_Psi)], [61, 10000 * abs(61-Goal_Psi)]]
    for i in range(-60, 61):
        if(safeZone[i] > 0):
            # cost = (1 / Gain_Psi) * sigmoid(abs(i - Goal_Psi)/60) + (1 / Gain_Distance) * sigmoid(1 - ld[i]/avoidRange)
            # cost = (1 / Gain_Psi) * sigmoid(abs(i - Goal_Psi)/60) 
            cost = Gain_Psi * CostFuncAngle(i - Goal_Psi) + Gain_Distance * CostFuncDistance(ld[i]) + Gain_Diff * CostFuncDiff(ld[i])
            thetaList.append([i, cost])
    
    thetaList = sorted(thetaList, key = lambda x : x[1])
    Psi_d = thetaList[0][0]

    if(ld[Goal_Psi] > Goal_Distance):
        isSafe = True
        for i in range((floor(Goal_Psi  - np.arctan2(BoatWidth/2, ld[Goal_Psi]) * 180 / np.pi)),(ceil(Goal_Psi + np.arctan2(BoatWidth/2, ld[Goal_Psi]) * 180 / np.pi))+1):
            if(ld[i] < Goal_Distance):
                isSafe = False
        if(isSafe):0, 0
    ax.fill(data_range, safeZone, color = [0,1,0,0.5])
    ax.plot([0,Psi_d * np.pi / 180], [0,5], color = [1,0,1,1])

    msg = Float32()
    msg.data = Psi_d + Psi
    Past_Psi_d = msg.data

    print(DockFinal)
    if(DockFinal != []):
        print(DockFinal)
        y_e = (Pos[0] - DockFinal[0]) * np.cos(Docking_theta * np.pi / 180) - (Pos[1] - DockFinal[1]) * np.sin(Docking_theta * np.pi / 180)
        DockPsi = Docking_theta - np.arctan(y_e / Docking_Delta) * 180 / np.pi
        ax.plot([0, (DockPsi-Psi) * np.pi / 180], [0,100], color = "green")
        Psi_d = DockPsi
        msg.data = Psi_d


        if((DockFinal[0]-Pos[0])**2 + (DockFinal[1]-Pos[1])**2 < 1):
            msg.data = -10000
        
        
    print(DockCnt)
    if(DockFinal == []):
        if((goalPos[0]-Pos[0])**2 + (goalPos[1]-Pos[1])**2 < 10 ** 2):
            try:
                Docking_Angle = rospy.wait_for_message("/vision", Float32, timeout=0.1).data
            except:
                pass
            if Docking_Angle!=-10000:
                Docklist = []
                for x, y in DockPos:
                    Docklist.append(np.arctan2(x-Pos[0], y-Pos[1]) * 180 / np.pi - Psi)

                # print(Docklist)
                Docklist = [abs(i-Docking_Angle) for i in Docklist]
                DockCnt[np.argmin(Docklist)] += 1
                for i in range(3):
                    if(DockCnt[i] > 20):
                        DockFinal = DockPos[i]
                # DockFinal = DockPos[np.argmin(Docklist)]
            else:
                msg.data = -10




        
    pub.publish(msg)
    # print(msg)


if __name__=="__main__":
    rospy.init_node('Autonomous',anonymous = False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    sub1 = message_filters.Subscriber("/LidarData", Float64MultiArray)
    sub2 = message_filters.Subscriber("/IMUData", Float32)
    sub3 = message_filters.Subscriber("/UTM", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2, sub3],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)
    ani = FuncAnimation(plt.gcf(), animate, interval = dt)
    plt.show()
