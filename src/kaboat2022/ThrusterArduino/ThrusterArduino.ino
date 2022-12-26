#if (ARDUINO >= 100)
 #include <ArduinoHardware.h>
#else
 #include <WProgram.h>
#endif
//아두이노 기본적인 라이브러리 Include

#include <Servo.h> //모터 제어를 위한 라이브러리
#include <ros.h>  //ROS
#include <std_msgs/Float64MultiArray.h>   //ROS msg

//BLDC_Motor_pinL : PIN5
//BLDC_Motor_pinR : PIN6

ros::NodeHandle  nh;    //노드 객체 생성

int channel1;     //조종기
int channel2;     //조종기
int channel3;     //조종기 모드 제어 부분
Servo ML, MR;     //ESC 모터 드라이버를 제어하기 위한 객체 생성

//Relay 핀 설정
int Relay1 = 2; //PIN2
int Relay2 = 3; //PIN3

int count = 0;


//모터에 신호를 전달해 주는 부분 (콜백 함수로 계속 실행됨)
void servo_cb( const std_msgs::Float64MultiArray&msg) 
{
      //Relay에 신호를 주어 스위치 ON
      digitalWrite(Relay1, HIGH);
      digitalWrite(Relay2, HIGH);

      //1000 ~ 2000 사이의 값을 writeMicroseconds
      ML.writeMicroseconds(msg.data[0]);//왼쪽 Thruster 출력부
      MR.writeMicroseconds(msg.data[1]);//오른쪽 Thruster 출력부
}
//Subscriber 객체 생성 (topic_name : /PWM, callback : servo_cb)
ros::Subscriber<std_msgs::Float64MultiArray> sub("PWM", servo_cb);


//기본 세팅 함수 (처음 한 번만 실행됨)
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
      int channel1, channel2, channel3;
      
      //컨트롤러의 값 가져오기
      if (pulseIn(10,HIGH) > 0)
      {
            //On/Off, 전/후진, 회전
            channel1 = pulseIn(8,HIGH);
            channel2 = pulseIn(9,HIGH);
            channel3 = pulseIn(10,HIGH);
      }

      //Serial.println(channel1); //1240 1610 2000
      // Serial.println(channel2); //937 1360 1720
      //Serial.println(channel3);------ 0:off, 1000 : manual control, 2000 : automode
      

      //컨트롤러에서 들어오는 값을 변환
      int c1, c2;
      c1 = map(channel1, 1220, 2010, -100, 100); // -10 ~ 10
      c2 = map(channel2, 920, 1725, -100, 100); // 0~20
      
      //컨트롤러에 노이즈가 있어 특정 경계안에서 움직이지 않도록 설정
      if(c1 > -15 && c1 < 15) c1 = 0;
      if(c2 > -5 && c2 < 25) c2 = 0;

      //컨트롤러가 종료되었을 때 제어 멈춤
      if (channel3 == 0)
      {
            MR.writeMicroseconds(1500);
            ML.writeMicroseconds(1500);
            digitalWrite(Relay1, LOW);
            digitalWrite(Relay2, LOW);
            Serial.println("shut down");
      }

      //수동 컨트롤 모드
      else if(channel3 < 1500)      
      {
            int TL, TR;
            digitalWrite(Relay1, HIGH);
            digitalWrite(Relay2, HIGH);

            //c1, c2에 맞는 출력부 수식 -500 ~ 500
            TR = -(c2 * 5 + 7 *c1);
            TL = c2 * 5 - 7 *c1;

            //-500 ~ 500 데이터를 1000 ~ 2000 의 데이터로 변환
            TL+=1500;
            TR+=1500;
            
            //출력부 전달 부분
            TL = constrain(TL,1050,1950);
            TR = constrain(TR,1050,1950);
            Serial.print(TL);
            Serial.print("  ");
            Serial.println(TR);
            MR.writeMicroseconds(TR);
            ML.writeMicroseconds(TL);
      }
      //자동 모드일 경우 SubScribe를 받아 사용
      else
      {
            Serial.println("AutoMode");
            nh.spinOnce();
      }
}
