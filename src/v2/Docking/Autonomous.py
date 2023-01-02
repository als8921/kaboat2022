#!/usr/bin/env python3
# 최종 Docking 미션을 수행하였던 코드

import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

Docking_theta = -10     # Docking 진입 각도
Docking_Delta = 2.5     # Docking Line Of Sight Delta값
Docking_Angle = -10000  # 현재 선수각 기준 목표 도형까지의 각도 (값이 들어오지 않았을 때 초기 값 : -10000)

DockFinal = []

# 최종 Docking 목적지의 UTM 좌표를 구하여 입력해 주었음
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
    ####################    자율운항 코드 끝    ####################

    # 최종 도착지가 지정되지 않았을 때
    # print(DockFinal)
    if(DockFinal != []):
        # print(DockFinal)
        y_e = (Pos[0] - DockFinal[0]) * np.cos(Docking_theta * np.pi / 180) - (Pos[1] - DockFinal[1]) * np.sin(Docking_theta * np.pi / 180)
        DockPsi = Docking_theta - np.arctan(y_e / Docking_Delta) * 180 / np.pi
        ax.plot([0, (DockPsi-Psi) * np.pi / 180], [0,100], color = "green")
        Psi_d = DockPsi
        msg.data = Psi_d


        if((DockFinal[0]-Pos[0])**2 + (DockFinal[1]-Pos[1])**2 < 1):
            msg.data = -10000
        
        
    # print(DockCnt)
    # 최종 도착지가 지정되었을 때
    if(DockFinal == []):
        # 자율운항 목적지에 도착하였을 때 도킹모드를 실행
        if((goalPos[0]-Pos[0])**2 + (goalPos[1]-Pos[1])**2 < 10 ** 2):
            # Vision 코드에서 Docking Angle 값이 넘어오는 것을 기다림
            try:
                Docking_Angle = rospy.wait_for_message("/vision", Float32, timeout=0.1).data
            except:
                pass

            # Docking Angle이 정해졌을 때
            if Docking_Angle!=-10000:
                Docklist = []
                # 내 위치를 기준으로 도킹 지점들의 각도를 DockList에 저장
                for x, y in DockPos:
                    Docklist.append(np.arctan2(x-Pos[0], y-Pos[1]) * 180 / np.pi - Psi)

                # 도킹 지점들의 각도와 Vision 코드에서 받은 도형 까지의 각도 차이가 가장 적은 점을 선택
                Docklist = [abs(i-Docking_Angle) for i in Docklist]
                DockCnt[np.argmin(Docklist)] += 1   # 각 차이가 가장 적은 각의 Count 값을 증가시킴

                # 특정횟수 이상 선택되었을 경우 최종 목적지를 설정 (데이터가 잘못 선택되는 경우를 막기 위해)
                for i in range(3):
                    if(DockCnt[i] > 20):
                        DockFinal = DockPos[i]
            # 감지되지 않았을 경우 왼쪽으로 계속 회전을 하는 명령을 주어 탐색을 기다림
            # 파도 발생기 때문에 배가 오른쪽으로 회전을 하여 왼쪽으로 강제로 명령을 주었음                            
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
