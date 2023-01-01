#!/usr/bin/env python3
# Lidar 데이터와 GPS, IMU (위치 정보)를 이용하여 지금 상황에서 최적의 희망 선수각을 구하는 알고리즘
# Lidar, GPS, IMU, GoalPosition => Psi_d

import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

# goalPos : 도착지의 UTM 좌표 (ref_GPS 를 기준)
# goalPos = [27, 3]
# goalPos = [-27.3, 11.5]
# goalPos =[-29.66, 16.2]
goalPos = [-28, 11.87]

# 배의 위치 정보 초기화
Pos = [0, 0]
Psi = 0
Past_Psi_d = 0

# 배의 폭 (m)
BoatWidth = 0.85

endRange = 1    # 마지막 목적지의 도착 범위
avoidRange = 4  # 배의 안전구역을 설정하기 위한 거리 (값 조절이 중요함)
maxRange = 5    # 시각화를 하기 위해 설정한 최대 거리

# 각 Cost값을 결정하기 위한 Gain값
Gain_Psi = 1
Gain_Distance = 8
Gain_Diff = 0

dt = 50 #ms
lidardata = [0] * 360   # 라이다 데이터 초기화

ax = plt.subplot(111, polar=True)

data_range = np.linspace(0,2*np.pi, 360)
plt.gcf().set_facecolor((0, 0.5, 1, 0.1))

# 각도에 대한 Cost를 구하는 함수
def CostFuncAngle(x):
    x = abs(x)
    if(x <= 10):
        return 0.01 * x
    else:
        return 0.1*(x - 10)

# 거리에 대한 Cost를 구하는 함수
def CostFuncDistance(x):
    if(x > 7):
        return 0
    else:
        return 10 - 10/7 * x

# 선수각과 이전 Psi_d에 대한 Cost를 구하는 함수 (만들어 놓고 사용하지 않음)
# 만든 이유 : 값이 급격하게 변하는 것을 막기 위해 값이 적게 변하는 쪽으로 선택하게 만드려는 목적
def CostFuncDiff(x):
    return abs((x + Psi - Past_Psi_d)%180)/dt

# ROS를 통해 데이터를 받아오는 부분
def callback(data1, data2, data3):
    global lidardata, Pos, Psi
    lidardata = data1.data
    Psi = data2.data
    Pos[0] = data3.data[0]
    Pos[1] = data3.data[1]

# Matplotlib 시각화
def animate(i):
    global Past_Psi_d, goalPos
    lidarinv = [0] * 360
    ld = np.array(lidardata)
    for i in range(61, 300):
        ld[i] = 0
    for i in range(360):
        if(ld[i] == 0):
            lidarinv[i] = maxRange

    # 실제 라이다는 데이터의 인덱스가 반대라서 뒤집어주는 과정
    # 유니티 시뮬레이터에서는 flip하지 않음
    ld = np.flip(ld)

    Goal_Distance = np.sqrt(np.power(goalPos[0] - Pos[0], 2) + np.power(goalPos[1] - Pos[1], 2))    # 도착지 까지의 거리
    Goal_Psi = np.arctan2(goalPos[0] - Pos[0], goalPos[1] - Pos[1]) * 180 / np.pi - Psi             # 도착지 까지의 각도
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
    # 안전구역을 설정해 주는 부분

    safeZone = [avoidRange] * 360
    
    # 거리가 avoidRange 보다 가까우면 위험 부분으로 판단하여 안전구역에서 제외
    for i in range(-60, 61):
        if(ld[i]!=0 and ld[i] < avoidRange):
            safeZone[i] = 0

    temp = np.array(safeZone)

    # 배의 크기를 고려하여 안전한 각도를 설정해 주기 (수식 이해 필요함)
    for i in range(-60, 61):
        if(safeZone[i] > safeZone[i + 1]): ### 
            for j in range(floor(i + 1 - np.arctan2(BoatWidth/2, ld[i + 1]) * 180 / np.pi),i + 1):
                temp[j] = 0
        if(safeZone[i] < safeZone[i + 1]): ### 
            for j in range(i, ceil(i + np.arctan2(BoatWidth/2, ld[i]) * 180 / np.pi) + 1):
                temp[j] = 0
    safeZone = temp

    #######################################################################################################
    
    # thetaList에 라이다 범위의 밖인 -60, 60 도를 추가 (Cost 값을 높게 만들어 선택 우선순위를 후순위로)
    thetaList = [[-61, 100000 * abs(-61-Goal_Psi)], [61, 100000 * abs(61-Goal_Psi)]] 

    # SafeZone에서 안전한 각도 중에 최선의 각도를 찾기 위한 Cost값 설정하기   
    for i in range(-60, 61):
        if(safeZone[i] > 0):
            cost = Gain_Psi * CostFuncAngle(i - Goal_Psi) + Gain_Distance * CostFuncDistance(ld[i]) + Gain_Diff * CostFuncDiff(ld[i])
            thetaList.append([i, cost])
    
    # 정렬을 하여 Cost 값이 가장 적은 각을 가져오기
    thetaList = sorted(thetaList, key = lambda x : x[1])
    Psi_d = thetaList[0][0]
    
    print(ld[Goal_Psi], Goal_Distance)


    # 도착지 까지 장애물이 없을 경우 최대속도를 내기 위한 코드 
    # (잘 작동이 안되서 이걸 보완하여 전략으로 설정했으면 좋겠음)
    if(ld[Goal_Psi] > Goal_Distance):
        isSafe = True
        for i in range((floor(Goal_Psi  - np.arctan2(BoatWidth/2, ld[Goal_Psi]) * 180 / np.pi)),(ceil(Goal_Psi + np.arctan2(BoatWidth/2, ld[Goal_Psi]) * 180 / np.pi))+1):
            if(ld[i] < Goal_Distance):
                print(ld[i])
                isSafe = False
        if(isSafe):
            Psi_d = Goal_Psi + 10000
                    

    
    print(Psi_d)
    ax.fill(data_range, safeZone, color = [0,1,0,0.5])
    if(Psi_d > 9000):
        ax.plot([0,(Psi_d-10000) * np.pi / 180], [0,5], color = [1,1,1,1])
    else:
        ax.plot([0,Psi_d * np.pi / 180], [0,5], color = [1,1,1,1])

    msg = Float32()
    msg.data = Psi_d + Psi
    Past_Psi_d = msg.data

    # 도착처리
    if((goalPos[0]-Pos[0])*(goalPos[0]-Pos[0]) + (goalPos[1]-Pos[1])*(goalPos[1]-Pos[1]) < endRange * endRange):
        # goalPos = [-9.5, -24]
        msg.data = -10000
        

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
