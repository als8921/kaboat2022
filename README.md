# KABOAT

프로그래밍 개발 환경

Ubuntu 18.04

ROS Melodic

macOS

Unity

---

-   하드웨어 설계 및 구성
-   시뮬레이터 구현
-   하드웨어 인 더 루프 시스템
    -   HILS
        하드웨어 인 더 루프(Hardware-In-the-Loop)라 함은 통합 시스템의 프로그램이 기본 하드웨어(CPU 또는 컨트롤
        러 하드웨어)에서 실행되고 있으며, 시뮬레이션이 별도의 기계에서 실행되고 있는 것을 말한다. 두 개의 시스템 사
        이의 인터페이스는 시험을 위해 개발되었다. 시뮬레이션은 중앙 제어 시스템의 프로그래밍을 검증하고 고무된 결과
        를 문서화하는 물리적 실제 설계 동적 시스템을 포함하기에 충분한 정확도가 있어야 한다. 시뮬레이션 프로그램에
        서 수학적 모델로 나타난다
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
