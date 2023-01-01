#!/usr/bin/env python3
#UTM 데이터를 바탕으로 다음 목적지 좌표까지 도달하는 선수각을 구하는 알고리즘
#(UTM, WayPointUTM) => Psi_d

import rospy
import math
from std_msgs.msg import Float32, Float64MultiArray

### Parameter ###
end_range = 3   #도착 거리 
# Delta = 1     # LOS Guidance를 사용하지 않고, 현재 위치와 다음 좌표까지의 각을 희망 선수각으로 설정하는 알고리즘을 선택함

WP = [0,0,
    -9.538178448099643, -2.0647230963222682, 
    -11.706345309619792, 13.135687047615647, 
    -3.344654312590137, -3.980027507059276, 
    -6.0551088334061205, 6.4590605539269745, 
    -3.6087476636166684, -12.516516923904419]
#AutoPilot Waypoint List

#k번째 Waypoint 도착
k = 0

def update(data):
    global k

    x, y = data.data[0], data.data[1]

    if (k <= len(WP) - 4):
        ######################LOS Guidance (대회에서 사용안함)######################
        # pi_p = math.atan2(WP[k + 2] - WP[k], WP[k + 3] - WP[k + 1])
        # y_e = (x - WP[k]) * math.cos(pi_p) - (y - WP[k + 1]) * math.sin(pi_p)
        # pi_p = pi_p * 180 / math.pi
        # psi_d = pi_p - math.atan(y_e / Delta) * 180 / math.pi
        ############################################################################

        #Psi_d 결정
        psi_d = math.atan2(WP[k+2] - x, WP[k+3] - y) * 180.0 / math.pi
        
        #원 범위안에 들어왔는지 도착 확인
        if(math.pow(WP[k + 2] - x, 2) + math.pow(WP[k + 3] - y, 2) < end_range * end_range):
            k += 2


    #Psi_d값으로 -10000의 특정한 값을 보내주어 이 값을 받았을 때 멈추도록 구현함
    else:
        psi_d = -10000

    #-180~180
    if psi_d > 180:
        psi_d -= 360

    pubdata = Float32()
    pubdata.data = psi_d
    pub.publish(pubdata)



if __name__=="__main__":
    rospy.init_node("LOS_Guidance", anonymous=False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    rospy.Subscriber("/UTM", Float64MultiArray, update)
    rospy.spin()