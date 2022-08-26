#!/usr/bin/env python3
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import utm

##### Setting #####
ref_GPS = [35.0695, 128.579]

def callback(data):
    msg = Float64MultiArray()
    biasX, biasY, _, _ = utm.from_latlon(ref_GPS[0],ref_GPS[1])
    WayPoint = np.reshape(data.data, (-1, 2))
    WP = []
    for i, j in WayPoint:
        x, y, _, _ = utm.from_latlon(i, j)
        x -= biasX
        y -= biasY
        WP.append(x)
        WP.append(y)

    msg.data = WP
    pub.publish(msg)
    # print(WP)
        

if __name__=="__main__":
    rospy.init_node("WayPoint_Node")
    pub = rospy.Publisher("/Waypoint", Float64MultiArray, queue_size=10)
    rospy.Subscriber("Waypoint_GPS", Float64MultiArray, callback)
    rospy.spin()