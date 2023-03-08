import math
import time
import cv2
import numpy as np
import calib_img


fieldcornerleft = cv2.imread("fieldcornerleft.jpg")
fieldcornerleft = cv2.resize(fieldcornerleft, (66, 66))
fieldcornerright = cv2.imread("fieldcornerright.jpg")
fieldcornerright = cv2.resize(fieldcornerright, (66, 66))
fieldinnercornerleft = cv2.imread("fieldinnercornerleft.png")
fieldinnercornerleft = cv2.resize(fieldinnercornerleft, (66, 66))

def main():
    frame = calib_img.main() #キャリブレーション後の画像を読み込む
    resultimg = frame
    
    # 処理対象画像に対して、テンプレート画像との類似度を算出する
    matchleft = cv2.matchTemplate(frame, fieldcornerleft, cv2.TM_CCOEFF_NORMED)
    matchright = cv2.matchTemplate(frame, fieldcornerright, cv2.TM_CCOEFF_NORMED)
    matchinnerleft = cv2.matchTemplate(frame, fieldinnercornerleft, cv2.TM_CCOEFF_NORMED)

    # 閾値に基づき類似度の高い部分を検出する
    threshold = 0.8
    matchfindleft = np.where(matchleft >= threshold)
    matchfindright = np.where(matchright >= threshold)
    matchfindinnerleft = np.where(matchinnerleft >= threshold)
    
    h, w = 66,66 #コーナー部のサイズ(mm)

    if len(matchfindleft[0]) != 0: #左コーナーが検出されているとき
        leftcornery = sum(matchfindleft[0])/len(matchfindleft[0])
        leftcornerx = sum(matchfindleft[1])/len(matchfindleft[1])
        cv2.rectangle(resultimg, (int(leftcornerx), int(leftcornery)), (int(leftcornerx)+w, int(leftcornery)+h), (0, 255, 0), 2)
        #print("left corner detected")
        return resultimg, 1, leftcornerx, leftcornery #結果画像と1を返す
    if len(matchfindright[0]) != 0: #右コーナーが検出されているとき
        rightcornery = sum(matchfindright[0])/len(matchfindright[0])
        rightcornerx = sum(matchfindright[1])/len(matchfindright[1])
        cv2.rectangle(resultimg, (int(rightcornerx), int(rightcornery)), (int(rightcornerx)+w, int(rightcornery)+h), (0, 255, 255), 2)
        #print("right corner detected")
        return resultimg, 2, rightcornerx, rightcornery #結果画像と2を返す
    if len(matchfindinnerleft[0]) != 0: #左コーナーが検出されているとき
        leftcornery = sum(matchfindinnerleft[0])/len(matchfindinnerleft[0])
        leftcornerx = sum(matchfindinnerleft[1])/len(matchfindinnerleft[1])
        cv2.rectangle(resultimg, (int(leftcornerx), int(leftcornery)), (int(leftcornerx)+w, int(leftcornery)+h), (255, 0, 0), 2)
        #print("left corner detected")
        return resultimg, 3, leftcornerx, leftcornery
    else:
        return resultimg, 0, 0, 0 #結果画像と0を返す

    