#!/usr/bin/env python3
import rospy
import message_filters
import time
import numpy as np
from std_msgs.msg import Float64MultiArray, Float32


##### Parameter #####
P_gain = 24
D_gain = 1

tau_X = 500

max_tau_N = 450

speed_Constant = 1.06 #1.05 --> 1.01(more speed on rotate)
##########

tau_N = 0
psi_error_past = 0
timepast = time.time()

minThrust = 1000    
maxThrust = 2000

isEnd = False
def callback(data1, data2):
    global psi_error_past
    global timepast, tau_X, isEnd
    dt = time.time() - timepast
    psi = data1.data
    psi_d = data2.data

    if(psi_d==-10000):
        isEnd = True
    else: isEnd = False

    psi_error = psi_d - psi
    if(psi_error > 180): psi_error -= 360
    elif(psi_error < -180): psi_error += 360

    psi_error_dot = (psi_error-psi_error_past)/dt
    psi_error_past = psi_error

    tau_N = P_gain * psi_error + D_gain * psi_error_dot


    idle_L = 1500 - tau_X * np.power(speed_Constant, -abs(psi_error))
    idle_R = 1500 + tau_X * np.power(speed_Constant, -abs(psi_error))

    if(tau_N > max_tau_N) : tau_N = max_tau_N
    elif(tau_N < -max_tau_N): tau_N = -max_tau_N

    pl, pr = 0, 0
    if(tau_N > 0):
        pr = int(-tau_N)
        pl = int(450 * np.arctan(pr / 450))

    elif(tau_N < 0):
        pl = int(-tau_N)
        pr = int(450 * np.arctan(pl / 450))

    pl += idle_L
    pr += idle_R

    Lpwm, Rpwm = pl,pr


    if(Rpwm > maxThrust): Rpwm = maxThrust
    elif(Rpwm < minThrust) : Rpwm = minThrust

    if(Lpwm > maxThrust): Lpwm = maxThrust
    elif(Lpwm < minThrust) : Lpwm = minThrust    

    if(isEnd):
        Lpwm, Rpwm = 1500, 1500

    print(Lpwm, Rpwm)

    PWM_data = Float64MultiArray()
    PWM_data.data = [Lpwm, Rpwm]
    pub.publish(PWM_data)
    timepast = time.time()

    

if __name__ == '__main__':
    rospy.init_node('PWM_Control', anonymous=False)
    pub = rospy.Publisher('PWM', Float64MultiArray, queue_size=10)
    sub1 = message_filters.Subscriber("/IMUData", Float32)
    sub2 = message_filters.Subscriber("/Psi_d", Float32)
    mf = message_filters.ApproximateTimeSynchronizer([sub1, sub2],10,0.1,allow_headerless=True)
    mf.registerCallback(callback)
    rospy.spin()
