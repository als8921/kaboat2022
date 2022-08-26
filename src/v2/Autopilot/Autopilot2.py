#!/usr/bin/env python3

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

    if(WP_Past != WP):
        k = 0
        print("WP Changed")

    WP_Past = WP

    x, y = data1.data[0], data1.data[1]

    if (k <= len(WP) -4):
        psi_d = math.atan2(WP[k+2] - x, WP[k+3] - y) * 180.0 / math.pi
        
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