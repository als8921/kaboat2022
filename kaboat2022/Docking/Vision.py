#!/usr/bin/env python3

import rospy
import cv2
import math
import numpy as np
from cv2 import VideoCapture
from std_msgs.msg import Float32
from sensor_msgs.msg import Image

### Parameter###
ObjectShape = 3
# 3 : Triangle
# 4 : Rectangle
# 10 : Star
# 0 : Circle

ObjectColor = "red"
#red, blue, green
##################


minArea = 50

WIDTH = 640
HEIGHT = 480

min_blue = np.array([100,50,50], dtype=np.uint8)
max_blue = np.array([150, 255, 255], dtype=np.uint8)

min_red = np.array([135, 100, 100], dtype=np.uint8)
max_red = np.array([179, 255, 255], dtype=np.uint8)
max_green = np.array([80, 255, 255], dtype=np.uint8)




# min_red = np.array([0, 0, 0], dtype=np.uint8)
# max_red = np.array([255, 255, 255], dtype=np.uint8)
# min_blue = np.array([0, 0, 0], dtype=np.uint8)
# max_blue = np.array([255, 255, 255], dtype=np.uint8)
# min_green = np.array([0, 0, 0], dtype=np.uint8)
# max_green = np.array([255, 255, 255], dtype=np.uint8)

cap = VideoCapture(-1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))


def setLabel(frame, pts, label):
    (x, y, w, h) = cv2.boundingRect(pts)
    pt1 = (x, y)
    pt2 = (x+w, y+h)
    cv2.rectangle(frame, pt1, pt2, (0, 255, 0), 2)
    cv2.putText(frame, label, (pt1[0], pt1[1]-3), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))

def colorFilter(frame, pts, min_color, max_color, list, center_x, center_y, label):
    (x, y, w, h) = cv2.boundingRect(pts)
    pt1 = (x, y)
    pt2 = (x+w, y+h)
    newframe = frame[int(y+h/2-10):int(y+h/2+10), int(x+w/2-10):int(x+w/2+10)]
    new_hsv = cv2.cvtColor(newframe, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(new_hsv, min_color, max_color)
    cont2black = cv2.countNonZero(mask)
    if cont2black >= 200:
        list.append(x+w/2 - center_x)
        cv2.circle(frame, (int(x+w/2), int(y+h/2)), 3, (0, 0, 255), -1)
        cv2.putText(frame, "(%d, %d)"%(int(x+w/2) - center_x, center_y - int(y+h/2)), (int(x+w/2-10), int(y+h/2-10)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255))
        setLabel(frame, pts, label)

def main():
    while True:
        try :
            ret, frame = cap.read()
            if ret == True:
                msg = Image()
                msg.data = frame
                pub1.publish(msg)

            
                colormask = cv2.inRange(frame, min_red, max_red)
                colorframe = cv2.bitwise_and(frame, frame, mask = colormask)
                cv2.imshow('color', colorframe)
                frame = cv2.medianBlur(frame, 5)
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                binary = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,cv2.THRESH_BINARY, 51, 10)
                edges = cv2.Canny(binary, 0, 200)

                _,contours,_ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                center_x = int(WIDTH/2)
                center_y = int(HEIGHT/2)
                a = float(center_x)#*2.54/141.211
                b = float(center_y)#*2.54/141.211
                cv2.circle(frame, (center_x, center_y), 3, (0, 0, 255), -1)
                order = []

                for cont in contours:
                    if cv2.contourArea(cont) < minArea:
                        continue

                    approx = cv2.approxPolyDP(cont, cv2.arcLength(cont, True)*0.02, True)
                    vtc = len(approx)
                    M = cv2.moments(cont, False)
                    cx = int(M['m10']/M['m00'])
                    cy = int(M['m01']/M['m00'])

                    if vtc == ObjectShape:
                        # colorFilter(frame, cont, min_blue, max_blue, order, center_x, center_y, str(ObjectShape))
                        colorFilter(frame, cont, min_red, max_red, order, center_x, center_y, str(ObjectShape))

                    if ObjectShape == 0:
                        length = cv2.arcLength(cont, True)
                        area = cv2.contourArea(cont)
                        ratio = 4 * math.pi * area / (length * length)
                        if ratio > 0.85:
                            colorFilter(frame, cont, min_green, max_green, order, center_x, center_y, str(ObjectShape))
                
                theta = -10000
                if(len(order) > 0):
                    theta = np.arctan(order[0]/616)*180/np.pi
                    print(theta)

                msg = Float32()

                msg.data = theta
                pub.publish(msg)
                cv2.imshow('edge', edges)
                cv2.imshow('camera', frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                if rospy.is_shutdown():
                    break


            else:
                continue
        except:
            pass

if __name__=="__main__":
    rospy.init_node("Vision", anonymous=False)
    pub = rospy.Publisher("/vision", Float32, queue_size=10)
    pub1 = rospy.Publisher("/Camera", Float32, queue_size=10)
    main()

    cap.release()
    cv2.destroyAllWindows
