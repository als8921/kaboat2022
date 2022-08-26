#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import math
import numpy as np

bridge = CvBridge()

# cap = cv2.VideoCapture('IMG_0222.MOV')

minArea = 500
#1 = red, 2 = blue, 3 = green
color = 3
#triangle = 1, cross = 2,  circle = 3
shape = 3


WIDTH = 1920
HEIGHT = 1080

# cv2.namedWindow("color_mask")
# cv2.moveWindow("color_mask", 0, 0)
# cv2.namedWindow("binary")
# cv2.moveWindow("binary", 640, 0)
# cv2.namedWindow("edges")
# cv2.moveWindow("edges", 1280, 0)

min_blue = np.array([110,50,0], dtype=np.uint8)
max_blue = np.array([130, 255, 255], dtype=np.uint8)

min_red_1 = np.array([0, 100, 0], dtype=np.uint8)
max_red_1 = np.array([10, 255, 255], dtype=np.uint8)

min_red_2 = np.array([170, 100, 0], dtype=np.uint8)
max_red_2 = np.array([180, 255, 255], dtype=np.uint8)

min_green = np.array([40, 30, 0], dtype=np.uint8)
max_green = np.array([80, 255, 255], dtype=np.uint8)

order = []


def setLabel(frame, pts, label):
    (x, y, w, h) = cv2.boundingRect(pts)
    pt1 = (x, y)
    pt2 = (x+w, y+h)
    cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
    cv2.putText(frame, label, (pt1[0], pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def colorFilter(frame, color, shape):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    hsv_frame = frame.copy
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
    _, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    center_x = int(WIDTH/2)
    center_y = int(HEIGHT/2)
    #a = float(center_x)
    #b = float(center_y)
    cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
    
    for cont in contours:
        if cv2.contourArea(cont) < minArea:
            continue

        approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True)*0.02, True)
        vtc = len(approx)
        M = cv2.moments(cont, False)
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])

        if vtc == 3 and shape == 1:
            cont2black = cv2.countNonZero(mask)
            if cont2black >= 200:
                order.append(cx - center_x)
                cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
                cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                setLabel(frame, cont, "Triangle")

        elif vtc == 12 and shape == 2:
            cont2black = cv2.countNonZero(mask)
            if cont2black >= 200:
                order.append(cx - center_x)
                cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
                cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                setLabel(frame, cont, "RedCross")

        elif shape == 3:
            length = cv2.arcLength(cont, True)
            area = cv2.contourArea(cont)
            ratio = 4 * math.pi * area / (length * length)
            if ratio > 0.85:
                cont2black = cv2.countNonZero(mask)
                if cont2black >= 200:
                    order.append(cx - center_x)
                    cv2.circle(frame, (cx, cy),  3, (255, 255, 255), -1)
                    cv2.putText(frame, "({:.2f}, {:.2f})".format(float(cx) - center_x, center_y - float(cy)), (cx - 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0))
                    setLabel(frame, cont, "Circle")
        

    cv2.imshow('edges', edges)
    cv2.imshow('color_mask', color_img)
    cv2.imshow('binary', test_binary)
    cv2.imshow('edges', test_edges)
    cv2.imshow('video', frame)


def cb(data):
    cap = bridge.imgmsg_to_cv2(data, 'bgr8')
    # cap.resize(WIDTH, HEIGHT)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    # print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    # print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    colorFilter(cap, color, shape)
    if cv2.waitKey(1) == ord("q"):
        return
    
    print("Processing")


# while True:
#     ret, frame = cap.read()
#     if ret == True:

#         colorFilter(frame, color, shape)
       

#         if cv2.waitKey(1) == ord("q"):
#             break
#     else:
#         continue

# cap.release()
# cv2.destroyAllWindows

rospy.init_node("Test", anonymous=False)
rospy.Subscriber('usb_cam/image_raw', Image, cb)
rospy.spin()