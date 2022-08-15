#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32, Float64MultiArray

### Parameter ###
end_range = 3.5
# Delta = 1
##
WP =  [-0.04401118226815015, -10.424458644818515, -8.247453809482977, -9.83531837631017, -2.661407724430319, -4.092080892063677, -7.6999332686536945, -9.72672996437177, -0.9562331881606951, -10.531507750973105]


k = 0

def update(data):
    global k

    x, y = data.data[0], data.data[1]

    if (k <= len(WP) -4):
        # pi_p = math.atan2(WP[k + 2] - WP[k], WP[k + 3] - WP[k + 1])
        # y_e = (x - WP[k]) * math.cos(pi_p) - (y - WP[k + 1]) * math.sin(pi_p)
        # pi_p = pi_p * 180 / math.pi
        # psi_d = pi_p - math.atan(y_e / Delta) * 180 / math.pi
        psi_d = math.atan2(WP[k+2] - x, WP[k+3] - y) * 180.0 / math.pi
        
        if(math.pow(WP[k + 2] - x, 2) + math.pow(WP[k + 3] - y, 2) < end_range * end_range):
            k += 2

    else:
        psi_d = -10000


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