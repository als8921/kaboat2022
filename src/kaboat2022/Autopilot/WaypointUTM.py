#!/usr/bin/env python3
# Waypoint GPS 리스트를 UTM 좌표로 변환해 주는 코드 (결과 값을 복사하여 사용하였음)
import numpy as np
import rospy
from std_msgs.msg import Float64MultiArray
import utm

##### Setting #####
WP = [35.069481, 128.57889, 35.069618, 128.578871, 35.069464, 128.578968, 35.069558, 128.578930]
WP = [35.069387, 128.578961]
ref_GPS = [35.0695, 128.579]

biasX, biasY, _, _ = utm.from_latlon(ref_GPS[0],ref_GPS[1])
WayPoint = np.reshape(WP, (-1, 2))
WP = []
for i, j in WayPoint:
    x, y, _, _ = utm.from_latlon(i, j)
    x -= biasX
    y -= biasY
    WP.append(x)
    WP.append(y)

print(WP)