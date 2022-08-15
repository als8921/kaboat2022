#!/usr/bin/env python
import rospy
import message_filters
import time
from std_msgs.msg import Float64MultiArray, Float32

psi_error_past = 0

timepast = time.time()

P_gain = 8
D_gain = 0

tau_N = 0
tau_X = 100

minThrust = 1300
maxThrust = 1700

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


    # if(abs(psi_error) > 50):
    #     tau_X = 0
    # else:
    #     tau_X = 100
        
    tempThrust = tau_X + abs(tau_N * 0.5) + 1500
    if(tempThrust > maxThrust):
        if(tau_N > 0):
            Lpwm, Rpwm = maxThrust, maxThrust - tau_N
        if(tau_N < 0):
            Lpwm, Rpwm = maxThrust + tau_N, maxThrust
    else:
        Rpwm = tau_X - tau_N / 2 + 1500
        Lpwm = tau_X + tau_N / 2 + 1500

    


    if(Rpwm > maxThrust): Rpwm = maxThrust
    elif(Rpwm < minThrust) : Rpwm = minThrust

    if(Lpwm > maxThrust): Lpwm = maxThrust
    elif(Lpwm < minThrust) : Lpwm = minThrust    

    Lpwm = 3000 - Lpwm

    # if(psi_error > 30):
    #     Lpwm, Rpwm = 1150, 1050
    # elif(psi_error < -30):
    #     Lpwm, Rpwm = 1950, 1850


    if(isEnd):
        Lpwm, Rpwm = 1500, 1500

    PWM_data = Float64MultiArray()
    PWM_data.data = [Lpwm, Rpwm]
    print("PSI : ", data1.data," PSI_D : ",data2.data, " Lpwm : ", Lpwm, " Rpwm : ", Rpwm)
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