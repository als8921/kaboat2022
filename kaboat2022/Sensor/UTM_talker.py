#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import NavSatFix
import utm

# ref_GPS = [rospy.get_param("ref_LAT"), rospy.get_param("ref_LON")]
ref_GPS = [35.0695, 128.579]
msg = Float64MultiArray()
biasX, biasY, _, _ = utm.from_latlon(ref_GPS[0],ref_GPS[1])

def callback(data):
    WP = []
    lat, lon = data.latitude, data.longitude
    x, y, _, _ = utm.from_latlon(lat, lon)
    x -= biasX
    y -= biasY

    msg.data =[x, y]
    pub.publish(msg)
    print(msg.data)
        

if __name__=="__main__":
    rospy.init_node("GPStoUTM_Node")
    pub = rospy.Publisher("/UTM", Float64MultiArray, queue_size=10)
    rospy.Subscriber("/ublox_gps/fix",NavSatFix, callback)
    rospy.spin()