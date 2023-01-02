#!/usr/bin/env python3
# 대회장 측에서 주어지는 GPS 좌표와 배에서 사용한 GPS에 오차가 있었기 때문에 WayPoint를 GUI를 통하여 직접 전달해 주었음
# 이런 센서문제에 즉각 대응할 수 있는 방안이 필요함
# 22 대회에서는 유니티로 현재 자세 값을 알 수 있는 UI를 만들어 지도의 점을 선택하여 그 점을 Autopilot의 Waypoint로 전달해주는 방법을 사용하였음

import rospy
import message_filters
import math
from std_msgs.msg import Float32, Float64MultiArray

### Parameter ###
end_range = 3
##
WP_Past = []
k = 0

def update(data1, data2):
    global k, WP_Past, cnt
    WP=data2.data

    # Waypoint가 바뀌었을 때 처음부터 다시 실행될 수 있도록
    if(WP_Past != WP):
        k = 0
        print("WP Changed")

    WP_Past = WP

    x, y = data1.data[0], data1.data[1]

    # 현재 위치를 기준으로 다음 점까지의 각도를 실시간으로 구해 따라가도록 설정하였음
    # Line Of Sight 알고리즘을 사용하는 것 보다 더 좋다고 판단하여 수정하였음
    if (k <= len(WP) -4):
        psi_d = math.atan2(WP[k+2] - x, WP[k+3] - y) * 180.0 / math.pi
        
        # Waypoint에 도착하면 다음 Waypoint로 넘겨주는 부분
        if(math.pow(WP[k + 2] - x, 2) + math.pow(WP[k + 3] - y, 2) < end_range * end_range):
            k += 2
            cnt.data+=1

    else:
        psi_d = -10000


    if psi_d > 180:
        psi_d -= 360
    pubdata = Float32()
    pubdata.data = psi_d
    pub.publish(pubdata)
    # print(psi_d)



if __name__=="__main__":
    rospy.init_node("LOS_Guidance", anonymous=False)
    pub = rospy.Publisher("/Psi_d", Float32, queue_size=10)
    sub1 = message_filters.Subscriber("/UTM", Float64MultiArray)
    sub2 = message_filters.Subscriber("/Waypoint", Float64MultiArray)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, 0.1, allow_headerless=True)
    mf.registerCallback(update)
    rospy.spin()