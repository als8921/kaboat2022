#!/usr/bin/env python3
# Psi 값과 Psi_d 값이 주어졌을 때 최종 PWM 출력 값을 결정해 줌
# (Psi_d, Psi) => PWM

# 단순히 각도 차이를 이용하여 전진부(Tau_X), 회전부(Tau_N)를 결정 -> 각도 조정이 어려워 문제를 해결하기 위해 V2 Version을 개발
import rospy
import message_filters
import time
from std_msgs.msg import Float64MultiArray, Float32

psi_error_past = 0

timepast = time.time()

P_gain = 8
D_gain = 0

# 전진부와 회전부를 구분 (전진부의 기본속도를 Tau_X로 정해줌)
tau_N = 0
tau_X = 100

# Thruster 과부하를 막기 위한 최댓값 제한
minThrust = 1300
maxThrust = 1700

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

    psi_error = psi_d - psi
    if(psi_error > 180): psi_error -= 360
    elif(psi_error < -180): psi_error += 360

    psi_error_dot = (psi_error-psi_error_past)/dt
    psi_error_past = psi_error

    tau_N = P_gain * psi_error + D_gain * psi_error_dot


    # 기본 값 Tau_X에서 Tau_N 만큼의 차이를 내기 위해 양쪽 Thrust값에 -/+를 해주는 부분
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

    # 왼쪽 모터는 도는 방향이 반대라서 1000 ~ 2000의 데이터를 2000 ~ 1000으로 변환
    Lpwm = 3000 - Lpwm

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