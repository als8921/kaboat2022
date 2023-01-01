#!/usr/bin/env python3
# 카메라를 연결하고 openCV 패키지로 여는 코드

import cv2
import numpy as np

### Parameter
minArea = 800

WIDTH = 800
HEIGHT = 600

center_x = int(WIDTH/2)
center_y = int(HEIGHT/2)

cap = cv2.VideoCapture(-1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)


def main():
    while True:
        ret, frame = cap.read()



        cv2.line(frame, (center_x, center_y), (center_x + 300, center_y), (255,0,0))
        cv2.imshow('camera', frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

            

    cap.release()
    cv2.destroyAllWindows

if __name__=="__main__":
    main()