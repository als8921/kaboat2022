#if (ARDUINO >= 100)
 #include <ArduinoHardware.h>
#else
 #include <WProgram.h>
#endif
//아두이노 기본적인 라이브러리 Include

#include <Servo.h> //모터 제어를 위한 라이브러리
#include <ros.h>  //ROS
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle  nh;

Servo ML, MR;

int Relay1 = 2;
int Relay2 = 3;

void servo_cb( const std_msgs::Float64MultiArray&msg) 
{
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);
      
      ML.writeMicroseconds(msg.data[0]);
      MR.writeMicroseconds(msg.data[1]);
}
ros::Subscriber<std_msgs::Float64MultiArray> sub("PWM", servo_cb);



void setup()      
{
      ArduinoHardware().setBaud(57600);   //BaudRate를 57600으로 설정
      ML.attach(5, 1050, 1950);     //모터의 출력 제한 Saturation
      MR.attach(6, 1050, 1950);     //모터의 출력 제한 Saturation
      MR.writeMicroseconds(1500);   //정지 명령 1500
      ML.writeMicroseconds(1500);   //정지 명령 1500
      
      pinMode(Relay1, OUTPUT);      //릴레이 핀 세팅 (2번 핀을 OUTPUT 모드로 설정)
      pinMode(Relay2, OUTPUT);      //릴레이 핀 세팅 (3번 핀을 OUTPUT 모드로 설정)
      
      nh.initNode();                //ROS Node
      nh.subscribe(sub);            //Subscriber
}

void loop()
{    
  nh.spinOnce();
}
