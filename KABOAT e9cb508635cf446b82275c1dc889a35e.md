# KABOAT

프로그래밍 개발 환경

Ubuntu 18.04

ROS Melodic

macOS

Unity

---

- 하드웨어 설계 및 구성
- 시뮬레이터 구현
- 하드웨어 인 더 루프 시스템
    - HILS
        
        하드웨어 인 더 루프(Hardware-In-the-Loop)라 함은 통합 시스템의 프로그램이 기본 하드웨어(CPU 또는 컨트롤
        러 하드웨어)에서 실행되고 있으며, 시뮬레이션이 별도의 기계에서 실행되고 있는 것을 말한다. 두 개의 시스템 사
        이의 인터페이스는 시험을 위해 개발되었다. 시뮬레이션은 중앙 제어 시스템의 프로그래밍을 검증하고 고무된 결과
        를 문서화하는 물리적 실제 설계 동적 시스템을 포함하기에 충분한 정확도가 있어야 한다. 시뮬레이션 프로그램에
        서 수학적 모델로 나타난다
        
- Rviz 시각화
- 대회 전략 및 알고리즘
- 구현

# 1. 하드웨어 구성

---

## Main Sensor

- GPS
    - Ublox ZED-F9P
- IMU
    - Xsens MTI-30
- LIDAR
    - Robosense RS-LIDAR-M1
- CAMERA
    - Logitech Webcam

## MCU

- Arduino Uno R3

## Main PC

- Intel NUC10i7FNH

# 소프트웨어 구성

---

## Sensor Package

1. Ublox ZED-F9P
    
    [https://github.com/ros-agriculture/ublox_f9p](https://github.com/ros-agriculture/ublox_f9p)
    
2. Xsens MTI-30
    
    [https://github.com/esteve/xsens_ros_mti_driver](https://github.com/esteve/xsens_ros_mti_driver)
    
3. RS-LIDAR-M1
    
    [https://github.com/RoboSense-LiDAR/ros_rslidar](https://github.com/RoboSense-LiDAR/ros_rslidar)
    

# 시뮬레이터 구성

---

GPS, IMU, LIDAR 센서를 구성하고 ROS Message로 Publish 하였다.

- GPS → Float64MultiArray
- IMU → Float32
- LIDAR
    - 2D LIDAR → LaserScan
    - 3D LIDAR → PointCloud

## GPS

UnityEngine.GameObject 클래스의 transform.position 값을 사용하였다.

y 방향 운동은 일어나지 않고 x, z 방향 운동만 하도록 설정하였다.

유니티에서 위도, 경도 값을 사용하지 않아 미터 단위로 실제 사이즈와 비슷하게 설정하였다.

<aside>
🛰️ POS = new double[] { Boat.transform.position.x, Boat.transform.position.z };

</aside>

## IMU

UnityEngine.GameObject 클래스의 transform.position, transform.rotation 값을 사용하였다.

roll, pitch 방향 회전은 일어나지 않고 yaw 방향 회전만 하도록 설정하였다.

<aside>
🧭 PSI = (Boat.transform.rotation.eulerAngles.y > 180) ? Boat.transform.rotation.eulerAngles.y - 360 : Boat.transform.rotation.eulerAngles.y;

</aside>

## LIDAR

```csharp
public class LidarSensor : MonoBehaviour
{
    float Distance = Lidar.distance;
    public RaycastHit hit;
    public float distanceData = 0;
    void Update()
    {
        Debug.DrawRay(transform.position, transform.forward * Distance, Color.red);

        if (Physics.Raycast(transform.position, transform.forward, out hit, Distance))
        {
            distanceData = hit.distance;
        }
        else
            distanceData = 0;
    }
}
```

Raycast의 충돌 거리 데이터를 통해 LIDAR를 구현하였다.

### 2D LIDAR

| Horizontal Fov | 360° |
| --- | --- |
| Resolution | 1° |

```csharp
namespace RosSharp.RosBridgeClient
{
    public class LidarLaserScan : UnityPublisher<MessageTypes.Sensor.LaserScan>
    {
        private MessageTypes.Sensor.LaserScan Data;
        public GameObject LidarObject;
        protected override void Start()
        {
            base.Start();
            Data = new MessageTypes.Sensor.LaserScan();
            Data.angle_min = 0;
            Data.angle_max = 359 * Mathf.PI / 180.0f;
            Data.angle_increment = 1 * Mathf.PI / 180.0f;
            Data.time_increment = 0.02f;
            Data.range_min = 1;
            Data.range_max = 200;
        }

        private void FixedUpdate()
        {
            double[] data = LidarObject.GetComponent<Lidar>().lidarData;
            for (int i = 0; i < 360; i++)
                Data.ranges[i] = (float)data[i];
            Publish(Data);
        }
    }
}
```

360개의 거리 데이터를 Sensor.LaserScan 데이터로 Publish 하도록 설정하였다.

### 3D LIDAR

| Horizontal Fov | 120° (-60° ~ 60°) |
| --- | --- |
| Vertical FoV | 25° (-12° ~ 12°) |
| Resolution | 1° |

```csharp
namespace RosSharp.RosBridgeClient
{
    public class LidarPCLPublish : UnityPublisher<MessageTypes.Sensor.PointCloud>
    {
        public GameObject Lidar3DObject;
        double[,] LidarData;
        float time = 0;
        MessageTypes.Sensor.PointCloud Data;

        protected override void Start()
        {
            base.Start();
            Data = new MessageTypes.Sensor.PointCloud();
        }

        private void FixedUpdate()
        {
            time += Time.deltaTime;
            if(time > 0.1f)
            {
                LidarData = Lidar3DObject.GetComponent<Lidar3D>().lidarData;
                MessageTypes.Geometry.Point32[] data = new MessageTypes.Geometry.Point32[3025];
                List<MessageTypes.Geometry.Point32> pclData = new List<MessageTypes.Geometry.Point32>();
                for (int i = 0; i < 3025; i++)
                {
                    double d = LidarData[i / 121, i % 121];
                    double a = (i / 121) - 12;
                    double b = (i % 121) - 60;
                    double x = d * Mathf.Cos((float)b * Mathf.PI / 180.0f) * Mathf.Cos((float)a * Mathf.PI / 180.0f);
                    double y = -d * Mathf.Sin((float)b * Mathf.PI / 180.0f);
                    double z = -d * Mathf.Cos((float)b * Mathf.PI / 180.0f) * Mathf.Sin((float)a * Mathf.PI / 180.0f);
                    if (x != 0 || y != 0 || z != 0)
                    {
                        pclData.Add(new MessageTypes.Geometry.Point32((float)x, (float)y, (float)z));
                    }
                }
                Data.points = pclData.ToArray();
                Data.header.frame_id = "map";
                Publish(Data);
                time = 0;
            }
        }
    }
}
```

3025개의 Point 데이터를 Sensor.PointCloud 데이터로 Publish 한다.

## 시뮬레이터 동역학 모델

별도로 동역학 모델에 대한 부분을 구현하지 않고, 유니티에 내장되어 있는 함수를 사용하여 보트의 운동을 구현하였다.

운동을 구현하기 위해 2가지 방법으로 첫 번째 모터 출력부 두 곳에서 AddRelativeForce 메서드를 사용하여 두 지점에서의 힘을 정해주는 방법, 두 번째 무게중심 축에서 선수 방향으로 AddRelativeForce, y축 방향으로 AddTorque 메서드로 TauX, TauN을 정해주는 방법을 구상하였고, 시뮬레이터에서는 두 번째 방법을 사용하였다.

양쪽 모터의 PWM값을 사용하여