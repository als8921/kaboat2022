#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import utm

##### Setting #####
ref_GPS = [35.0695, 128.579]

# WayPoint = np.zeros([4,2])
# WayPoint[0] = [36.3687312, 127.3452688]
# WayPoint[1] = [36.368870, 127.345281]
# WayPoint[2] = [36.3688497, 127.3455597]
# WayPoint[3] = [36.368698, 127.345534]


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
    print(WayPoint)
        

if __name__=="__main__":
    rospy.init_node("WayPoint_Node")
    pub = rospy.Publisher("/Waypoint", Float64MultiArray, queue_size=10)
    rospy.Subscriber("Waypoint_GPS", Float64MultiArray, callback)
    rospy.spin()