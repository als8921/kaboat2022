# KABOAT

프로그래밍 개발 환경

Ubuntu 18.04

ROS Melodic

macOS

Unity

---

-   하드웨어 설계 및 구성
-   시뮬레이터 구현
-   Rviz 시각화
-   대회 전략 및 알고리즘
-   구현

# 1. 하드웨어 구성

---

## Main Sensor

-   GPS
    -   Ublox ZED-F9P
-   IMU
    -   Xsens MTI-30
-   LIDAR
    -   Robosense RS-LIDAR-M1
-   CAMERA
    -   Logitech Webcam

## MCU

-   Arduino Uno R3

## Main PC

-   Intel NUC10i7FNH

# 소프트웨어 구성

---

## Sensor Package

1. Ublox ZED-F9P

    [https://github.com/ros-agriculture/ublox_f9p](https://github.com/ros-agriculture/ublox_f9p)

2. Xsens MTI-30

    [https://github.com/esteve/xsens_ros_mti_driver](https://github.com/esteve/xsens_ros_mti_driver)

3. RS-LIDAR-M1

    [https://github.com/RoboSense-LiDAR/ros_rslidar](https://github.com/RoboSense-LiDAR/ros_rslidar)

# 파일 구성

```
├── README.md
└── src
    ├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
    ├── kaboat2022
    │   ├── Autonomous
    │   │   └── Autonomous.py               자율운항알고리즘
    │   ├── Autopilot
    │   │   ├── LOS_Guidance.py             희망 선수각
    │   │   ├── PWM_Control.py              희망 선수각으로 PWM 제어 V1
    │   │   ├── PWM_Control_v2.py           희망 선수각으로 PWM 제어 V2
    │   │   └── WaypointUTM.py              Waypoint GPS 좌표를 UTM 좌표로 변환
    │   ├── CMakeLists.txt
    │   ├── Docking
    │   │   └── Docking.py                  도형 식별
    │   ├── Sensor
    │   │   ├── CameraOpen.py               카메라 테스트
    │   │   ├── IMU_Pub.py                  Quaternion 값을 선수각으로 변환
    │   │   ├── Lidar_Pub.py                -60 ~ 60 도 데이터를 360도의 데이터로 변환
    │   │   └── UTM_talker.py               현재 GPS 값을 UTM 좌표로 변환
    │   ├── ThrusterArduino
    │   │   └── ThrusterArduino.ino         PWM 값이 결정되었을 때 아두이노에 명령 전달
    │   ├── launch
    │   │   ├── Autonomous.launch           자율운항모드 실행
    │   │   ├── Autopilot.launch            오토파일럿 V1
    │   │   ├── Autopilot_v2.launch         오토파일럿 V2
    │   │   └── sensor.launch               센서를 모두 작동
    │   ├── package.xml
    │   └── src
    │       └── kaboat2022_sensor.launch
    └── v2
        ├── Autopilot
        │   ├── Autopilot2.py               오토파일럿 V2
        │   ├── PWM_Control_v2.py           희망 선수각으로 PWM 제어 V2
        │   └── WaypointUTM.py              Waypoint GPS 좌표를 UTM 좌표로 변환
        ├── Autopilot.launch                오토파일럿
        ├── CMakeLists.txt
        ├── Docking
        │   ├── Autonomous.py               자율운항 + 도킹 모드
        │   └── Vision.py                   선택된 도형을 인식하고 식별
        ├── Docking.launch                  도킹 모드 실행
        ├── UTMConverter.py                 UTM 좌표로 변환하는 테스트 코드
        └── package.xml
```
