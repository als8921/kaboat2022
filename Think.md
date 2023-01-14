
## 뒤에서 부터 하나씩
1. 모터를 돌리기 (아두이노)
ML.writeMicroseconds(TL);
MR.writeMicroseconds(TR);
이 부분이 모터에 신호를 주는 부분 
TL, TR 부분에 1000에서 2000의 수를 정해주기

2. TL, TR값을 1200, 1800 줘보는거 -> 모터 도는 것을 확인

3. 모드

	3-1. 조종기를 사용해서 TL, TR값을 정해줌 -> ROS 필요없음

	3-2. 파이썬에서 판단한 알고리즘 대로 움직여야함 -> ROS로 값을 보내줌

4. 값을 어떻게 받아올 것인가? -> ino 파일에서 알고리즘을 다짜기???
-> 불편함, C,Cpp(OOP)을 다 알아야함 -> 파이썬으로 파일을 여러개로 분리하여 코드를 짤 수 있겠다.

5. 아두이노에서

> ML.writeMicroseconds(TL);<br>
> MR.writeMicroseconds(TR);

TL, TR 값만 받아오자 -> 파이썬에서 넘겨주면 되겠다.

```
void servo_cb( const std_msgs::Float64MultiArray&msg) 
//msg로 std_msgs::Float64MultiArray 형태의 데이터를 받아옴
{
      //Relay에 신호를 주어 스위치 ON
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);

      //1000 ~ 2000 사이의 값을 writeMicroseconds
      ML.writeMicroseconds(msg.data[0]);//왼쪽 Thruster 출력부
      MR.writeMicroseconds(msg.data[1]);//오른쪽 Thruster 출력부
      //ROS 에서 받아온 데이터를 그대로 모터에 출력
}
```
* 1000 -> 역회전
* 1500 -> 정지
* 2000 -> 정회전

```
sudo apt install ros-melodic-rosserial-python //ROSSERIAL INSTALL
```

아두이노에는 네트워크가 안됨 -> 시리얼 통신을 통해서 ROS 데이터를 주고 받을 수 있음
-> 포트 설정등을 위한 코드를 실행시켜줘야함

```
rosrun rosserial_python serial_node.py
```


## ROS Publish in Terminal
```
rostopic pub /PWM std_msgs/Float64MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data: [1600, 1500]" -r 10

```


ROS /PWM 1600, 1600 -> Serial 통신을 통해 아두이노의 Subscribe로 전달 -> 콜백함수가 돌면서 모터 회전

-> PWM값으로 장난을 쳐보고 싶다.

```
import rospy 
import time
from std_msgs.msg import Float64MultiArray

rospy.init_node("testPwmNode")
pwmPublisher = rospy.Publisher("/PWM", Float64MultiArray, queue_size = 10)
pwmData = Float64MultiArray()

for i in range(100):
    pwmData.data = [1500+2*i, 1500-2*i]
    pwmPublisher.publish(pwmData)
    time.sleep(0.02)

for i in range(100, 0, -1):
    pwmData.data = [1500+2*i, 1500-2*i]
    pwmPublisher.publish(pwmData)
    time.sleep(0.02)


pwmData.data = [1500, 1500]
pwmPublisher.publish(pwmData)
```

PWM값을 주었을 때 그 명령대로 움직이는 것을 확인


현재 선수각, 희망 선수각 -> PWM을 정해보자!!

PID제어를 통해서 Error 값을 줄이는 방향으로