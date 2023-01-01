#!/usr/bin/env python3
# 120도의 Laserscan 데이터를 360칸의 배열에 넣어 보내주는 코드

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64MultiArray

def callback(data):
    temp = [0] * 360
    ld = Float64MultiArray()
    for i, j in enumerate(data.ranges):
        temp[i-60] = j
    ld.data = temp
    pub.publish(ld)


if __name__ == '__main__':
    rospy.init_node('Lidar_talker', anonymous=False)
    pub = rospy.Publisher('/LidarData', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/LidarLaserScan', LaserScan, callback)
    rospy.spin()