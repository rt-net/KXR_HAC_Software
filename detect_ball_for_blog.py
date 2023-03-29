import math
import time
import cv2
import numpy as np
import calib_img

balllower = np.array([5,200,180])
ballupper = np.array([25,255,255])
BallSizeTH = 10

def main():
    frame = calib_img.main() #キャリブレーション後の画像を読み込む
    resultimg = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
    frame_mask = cv2.inRange(hsv, balllower, ballupper)
    area = cv2.countNonZero(frame_mask)
    #print(area)
    imgcog = cv2.moments(frame_mask, False)
    if area > BallSizeTH:
        ball = True
        try:
            x,y= int(imgcog["m10"]/imgcog["m00"]) , int(imgcog["m01"]/imgcog["m00"])
            cv2.circle(resultimg, (x,y), 4, 100, 2, 4)
            return frame_mask, area, x, y
        except:
            return resultimg, area, 0, 0
    else:
        ball = False
        return resultimg, area, 0, 0
        