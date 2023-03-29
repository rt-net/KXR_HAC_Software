import math
import time
import cv2
import numpy as np
import calib_img

LineareaTH = 1000
fieldlower = np.array([165,50,50])
fieldupper = np.array([180,255,255])

def main():
    frame = calib_img.main() #キャリブレーション後の画像を読み込む
    resultimg = frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
    frame_mask = cv2.inRange(hsv, fieldlower, fieldupper)   #エッジ赤線以外をマスク
        
    ksize=3 #フィルターサイズ
    Blur = cv2.medianBlur(frame_mask,ksize) #ぼかしフィルタ
    Linecog = cv2.moments(Blur, False) #線の重心座標を得る

    Linearea = cv2.countNonZero(Blur) #線の面積を得る
    
    angle = 0
    
    if Linearea > LineareaTH:

        xcog,ycog= int(Linecog["m10"]/Linecog["m00"]) , int(Linecog["m01"]/Linecog["m00"]) #線の重心座標を代入
        cv2.circle(resultimg, (xcog,ycog), 4, 100, 2, 4) #重心を円で示す

        fieldedges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジ検出
        kernel = np.ones((2,2),np.uint8) #膨張フィルタ
        dilation = cv2.dilate(fieldedges,kernel,iterations = 1) #膨張

        Lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換
        line_length = 1000 #描画する線の長さ

        x1total, x2total, y1total, y2total = 0, 0, 0, 0 #線のパラメータの平均を得るための合計値の初期値
        
        try:
            for line in Lines:
                rho = line[0][0]
                theta = line[0][1]
                a = math.cos(theta)
                b = math.sin(theta)
                x0 = a * rho
                y0 = b * rho
                
                x1 = int(x0 - line_length * b)
                x2 = int(x0 + line_length * b)
                y1 = int(y0 + line_length * a)
                y2 = int(y0 - line_length * a)
                
                try:
                    a = (y2-y1)/(x2-x1)
                    b = y1-a*x1
                except:
                    a = 0
                    
                if a >= 0:
                    if y2 <= y1:
                        y2dash = y2
                        y2 = y1
                        y1 = y2dash
                    if x2 <= x1:
                        x2dash = x2
                        x2 = x1
                        x1 = x2dash
                if a < 0:
                    if y2 <= y1:
                        y2dash = y2
                        y2 = y1
                        y1 = y2dash
                    if x1 <= x2:
                        x1dash = x1
                        x1 = x2
                        x2 = x1dash
                
                x1total = x1 + x1total
                x2total = x2 + x2total
                y1total = y1 + y1total
                y2total = y2 + y2total
            
            cv2.line(
                resultimg, 
                (int(x1total/len(Lines)), int(y1total/len(Lines))), 
                (int(x2total/len(Lines)), int(y2total/len(Lines))), 
                (0, 255, 255), thickness=2, lineType=cv2.LINE_4 ) #見えている線の合成の描画
            
            angle = math.atan(-(y2total-y1total)/(x2total-x1total))*180/3.14
            if angle < 0:
                angle = -(angle + 90)
            else:
                angle = -(angle - 90)
            b = y1total - a*x1total

        except:
            a = 0
            #print("No line detected by hough")
        
        
        return frame_mask, angle, xcog, ycog
    
    else:
        #print("No line")
        return resultimg, 0, 0, 0
