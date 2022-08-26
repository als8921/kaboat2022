#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import Float32, Float64MultiArray

### Parameter ###
end_range = 3
# Delta = 1
##
WP = [0,0,-9.538178448099643, -2.0647230963222682, -11.706345309619792, 13.135687047615647, -2.9344654312590137, -3.980027507059276, -6.3551088334061205, 6.4590605539269745, -3.6087476636166684, -12.516516923904419]
# WP = [0,0,
#     -9.538178448099643, -2.0647230963222682, 
#     -11.706345309619792, 13.135687047615647, 
#     -3.344654312590137, -3.980027507059276, 
#     -6.0551088334061205, 6.4590605539269745, 
#     -3.6087476636166684, -12.516516923904419]

k = 0

def update(data):
    global k

    x, y = data.data[0], data.data[1]

    if (k <= len(WP) - 4):
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