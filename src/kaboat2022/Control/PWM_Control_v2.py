#!/usr/bin/env python3
# Psi 값과 Psi_d 값이 주어졌을 때 최종 PWM 출력 값을 결정해 줌
# (Psi_d, Psi) => PWM

##### 실험을 통해 수식을 세워 마지막 Thruster 출력부를 구현하였음 #####
##### 대회에서 좋은 성적을 거둘수 있었던 가장 좋은 아이디어라고 생각 #####
import rospy
import message_filters
import time
import numpy as np
from std_msgs.msg import Float64MultiArray, Float32

##### Parameter #####
P_gain = 19
D_gain = 0

tau_X = 300

max_tau_N = 450

speed_Constant = 1.06 #1.05 --> 1.01(more speed on rotate)
##########

tau_N = 0
psi_error_past = 0
timepast = time.time()

minThrust = 1050
maxThrust = 1950

isEnd = False
def callback(data1, data2):
    global psi_error_past
    global timepast, tau_X, isEnd
    dt = time.time() - timepast
    psi = data1.data
    psi_d = data2.data

    # psi_d가 -10000일 때 도착처리
    if(psi_d==-10000):
        isEnd = True
    else: isEnd = False

    # 기본 전진속도를 늘리기 위해 넣은 부분 (제대로 사용하진 못함)
    if(psi_d > 9000):
        psi_d -=10000
        tau_X = 600
        
    psi_error = psi_d - psi
    if(psi_error > 180): psi_error -= 360
    elif(psi_error < -180): psi_error += 360

    psi_error_dot = (psi_error-psi_error_past)/dt
    psi_error_past = psi_error


    # 각도의 Error 값을 이용하여 PD제어 (대회에서 D게인을 거의 0으로 설정하고 사용하였음)
    tau_N = P_gain * psi_error + D_gain * psi_error_dot

    # 각도의 Error 값이 적으면 회전을 줄이고, Error 값이 크면 회전을 키우는 수식을 따로 세워사용함
    # PID 제어를 대체하는 수식을 세운 부분
    idle_L = 1500 - tau_X * np.power(speed_Constant, -abs(psi_error))
    idle_R = 1500 + tau_X * np.power(speed_Constant, -abs(psi_error))

    if(tau_N > max_tau_N) : tau_N = max_tau_N
    elif(tau_N < -max_tau_N): tau_N = -max_tau_N

    pl, pr = 0, 0
    # 실험적인 부분
    if(tau_N > 0):
        pr = int(-tau_N)
        pl = int(450 * np.arctan(pr / 450))

    elif(tau_N < 0):
        pl = int(-tau_N)
        pr = int(450 * np.arctan(pl / 450))

    pl += idle_L
    pr += idle_R

    Lpwm, Rpwm = pl,pr


    ##### Saturation #####
    if(Rpwm > maxThrust): Rpwm = maxThrust
    elif(Rpwm < minThrust) : Rpwm = minThrust

    if(Lpwm > maxThrust): Lpwm = maxThrust
    elif(Lpwm < minThrust) : Lpwm = minThrust    


    ##### 도착 처리 #####
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
