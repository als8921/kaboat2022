#!/usr/bin/env python
# 선택된 도형과 색깔의 위치를 찾아내 그 지점까지 각도를 구해주는 코드
# (Shape, Color) => (Angle)
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

# 카메라를 ROS 메세지로 전달해 주는 ROS 패키지를 사용
bridge = CvBridge()

# cross : minArea 100
minArea = 100

#1 = red, 2 = blue, 3 = green
color = 1
#triangle = 1, cross = 2,  circle = 3
shape = 1


# 색깔을 정하는 임계값을 정해줌
min_blue = np.array([110,50,0], dtype=np.uint8)
max_blue = np.array([135, 255, 255], dtype=np.uint8)

# 빨간색의 범위는 0과 255 를 모두 포함하여 2개로 나누어 줌
min_red_1 = np.array([0, 50, 0], dtype=np.uint8)
max_red_1 = np.array([10, 255, 255], dtype=np.uint8)

min_red_2 = np.array([170, 50, 0], dtype=np.uint8)
max_red_2 = np.array([180, 255, 255], dtype=np.uint8)

min_green = np.array([40, 30, 0], dtype=np.uint8)
max_green = np.array([80, 255, 255], dtype=np.uint8)

order = 0

WIDTH = 640
HEIGHT = 480

# Label을 달아주는 함수
def setLabel(frame, pts, label):
    (x, y, w, h) = cv2.boundingRect(pts)
    pt1 = (x, y)
    pt2 = (x+w, y+h)
    cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
    cv2.putText(frame, label, (pt1[0], pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def colorFilter(frame, color, shape):
    global order

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_frame = frame.copy
    ##### 색깔 필터 #####
    if color == 1:
        mask1 = cv2.inRange(hsv, min_red_1, max_red_1)
        mask2 = cv2.inRange(hsv, min_red_2, max_red_2)
        mask = mask1 | mask2
    elif color == 2:
        mask = cv2.inRange(hsv, min_blue, max_blue)
    elif color == 3:
        mask = cv2.inRange(hsv, min_green, max_green)

    
    mask_resize=cv2.resize(mask,(640,480))
    color_img = cv2.bitwise_and(frame, frame, mask = mask)
    color_img = cv2.resize(color_img, (640, 480))
    frame=cv2.resize(frame, (640, 480))
    hsv_frame = cv2.medianBlur(color_img, 5)
    #gray = cv2.cvtColor(color_img, cv2.COLOR_BGR2GRAY)
    _,binary=cv2.threshold(mask_resize,1,255,cv2.THRESH_BINARY)
    test_binary = binary.copy()
    test_binary = cv2.resize(binary, (640, 480))
    edges = cv2.Canny(mask_resize, 0, 255)
    test_edges = edges.copy()
    test_edges = cv2.resize(test_edges, (640, 360))


    # 도형의 윤곽선을 찾는 과정
    _, contours,_ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # 화면의 Center값
    center_x = int(WIDTH/2)
    center_y = int(HEIGHT/2)

    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
    
    for cont in contours:
        # 도형의 면적이 minArea보다 작으면 pass
        if cv2.contourArea(cont) < minArea:
            continue

        approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True)*0.02, True)
        vtc = len(approx)

        # 도형의 가운데 점 찾기
        M = cv2.moments(cont, False)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        order = -10000

        # 삼각형
        if vtc == 3 and shape == 1:
            order = cx - center_x   # 화면의 가운데 점에서 도형의 중심까지의 x 거리
            cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
            cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
            setLabel(frame, cont, "Triangle")

        # 십자가
        elif vtc == 12 and shape == 2:
            order = cx - center_x   # 화면의 가운데 점에서 도형의 중심까지의 x 거리
            cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
            cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
            setLabel(frame, cont, "Cross")

        # 원
        elif shape == 3:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)

            # 원의 찌그러진 정도 파악
            ratio = 4 * math.pi * area / (length * length)
            if ratio > 0.85:
                order = cx - center_x   # 화면의 가운데 점에서 도형의 중심까지의 x 거리
                cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
                cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                setLabel(frame, cont, "Circle")
    
    # cv2.imshow('edges', edges)
    # cv2.imshow('color_mask', color_img)
    # cv2.imshow('binary', test_binary)
    # cv2.imshow('edges', test_edges)
    cv2.imshow('video', frame)

    # 떨어진 x 픽셀값이 결정되었을 경우
    if(order != -10000):
        # 화면의 중심에서 떨어진 x 값이 커질수록 각도가 커지는 시야각 원리를 이용한 식
        theta = math.atan2(order,493)*180/np.pi 
    else:
        theta = -10000

    msg = Float32()
    msg.data = theta
    pub.publish(msg)



def update(data):
    frame = bridge.imgmsg_to_cv2(data, "bgr8")
    colorFilter(frame, color, shape)
    if cv2.waitKey(1) == ord("q"):
        return


if __name__=="__main__":
    rospy.init_node("Vision_node")
    pub = rospy.Publisher("/vision", Float32, queue_size=10)
    rospy.Subscriber("/usb_cam/image_raw", Image, update)
    rospy.spin()