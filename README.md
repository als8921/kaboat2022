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

├── README.md
└── src
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
├── kaboat2022
│   ├── Autonomous
│   │   └── Autonomous.py
│   ├── Autopilot
│   │   ├── LOS_Guidance.py
│   │   ├── PWM_Control.py
│   │   ├── PWM_Control_v2.py
│   │   └── WaypointUTM.py
│   ├── CMakeLists.txt
│   ├── Docking
│   │   └── Docking.py
│   ├── Sensor
│   │   ├── CameraOpen.py
│   │   ├── IMU_Pub.py
│   │   ├── Lidar_Pub.py
│   │   └── UTM_talker.py
│   ├── ThrusterArduino
│   │   └── ThrusterArduino.ino
│   ├── launch
│   │   ├── Autonomous.launch
│   │   ├── Autopilot.launch
│   │   ├── Autopilot_v2.launch
│   │   └── sensor.launch
│   ├── package.xml
│   └── src
│   └── kaboat2022_sensor.launch
└── v2
├── Autopilot
│   ├── Autopilot2.py
│   ├── PWM_Control_v2.py
│   └── WaypointUTM.py
├── Autopilot.launch
├── CMakeLists.txt
├── Docking
│   ├── Autonomous.py
│   └── Vision.py
├── Docking.launch
├── UTMConverter.py
└── package.xml
