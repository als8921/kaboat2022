#!/usr/bin/env python3
# Docking을 구현 첫 번째 아이디어
# 라이다 데이터의 변화가 커졌을 때 코너점으로 판단하여 도킹해야하는 지점의 랜드마크로 설정하여 특정 지점으로 목적지를 정해주기
# 실제 대회에서는 이 방법을 사용하지 않고 다른 전략을 사용함

import rospy
import message_filters
import numpy as np
from math import floor, ceil, exp
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from std_msgs.msg import Float64MultiArray, Float32

Docking_theta = -80 # 최종 Docking 하는 지점의 진입 각도
Docking_Delta = 2   # Line Of Sight의 Delta 값
Docking_Angle = 0   # 카메라에서 도형을 인식한 현재 선수각을 기준으로 해당 도형까지의 각도

goalPos = [7.82, 1.74]  # 1차 자율운항모드 도착점
end_range = 2   # 도착처리하는 범위
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

    # 장애물과 다음 장애물 사이의 거리가 특정 거리 이상일 때 그 점을 도킹 지점의 랜드마크로 설정
    obs = []
    for i in range(-60, 61):
        if(ld[i] - ld[i+1] > obs_range):
            obs.append(i+1)
        elif(ld[i] - ld[i+1] <-obs_range):
            obs.append(i)

    Docklist = []
    for i in range(len(obs) - 1):
        # 두 지점의 각도가 7도 이상, 두 지점의 거리차(깊이)가 3(m) 이하 일 때 그 지점을 DockList에 등록
        if(abs(obs[i] - obs[i + 1]) > 7 and abs(ld[obs[i]] - ld[obs[i + 1]]) < 3):
            Docklist.append([(obs[i] + obs[i + 1])/2, (ld[obs[i]] + ld[obs[i + 1]])/2])

    Docklist = np.array(Docklist)
    ax.clear()
#################

    # 최종 도킹 지점이 결정되었을 때
    if(DockFinal != []):
        theta, r = DockFinal
        print(DockPos)
        # 최종 목적지가 결정되었을 때 Line Of Sight 알고리즘을 이용하여 그 지점까지 이동하는 각도를 정하기
        y_e = (Pos[0] - DockPos[0] + 5 * np.sin(Docking_theta * np.pi / 180)) * np.cos(Docking_theta * np.pi / 180) - (Pos[1] - DockPos[1] + 5 * np.cos(Docking_theta * np.pi / 180)) * np.sin(Docking_theta * np.pi / 180)
        DockPsi = Docking_theta - np.arctan(y_e / Docking_Delta) * 180 / np.pi
        ax.plot([0, (DockPsi-Psi) * np.pi / 180], [0,100], color = "green")
        
        # 최종 도킹 목적지에 도착하였을 때
        if((DockPos[0]-Pos[0])**2 + (DockPos[1]-Pos[1])**2 < end_range ** 2):
            DockPsi = -10000
            
        msg = Float32()
        msg.data = DockPsi
        pub.publish(msg)

    # 최종 도킹 지점이 결정되지 않았을 때
    else:  
        # 1차 자율운항을 통과하고 마커를 탐색하는 구역으로 들어왔을 때
        if((goalPos[0]-Pos[0])**2 + (goalPos[1]-Pos[1])**2 < end_range ** 2):
            # DockList가 결정되었을 때
            if(Docklist!=[]):

                Docklist = sorted(Docklist, key = lambda x:abs(x[0]-Docking_Angle)) # DockList중 DockingAngle과 가장 각도가 가까운 각을 찾기 위한 정렬
                print(Docklist)
                DockFinal = Docklist[0] # 최종 각도를 가장 각도차이가 적은 각으로 정하기
                theta, r = DockFinal
                DockPos = (Pos[0]+r*np.sin((Psi+theta)*np.pi/180), Pos[1]+r*np.cos((Psi+theta)*np.pi/180))  # GPS, IMU, 도착점의 각도, 도착점까지의 거리를 이용하여 최종 목적지의 UTM 값을 계산


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