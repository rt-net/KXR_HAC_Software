import math
import time
import cv2
import numpy as np

delay = 1

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
ret, im = cap.read()
print(type(cap))
print(cap.isOpened())

fieldlower = np.array([165,50,50])
fieldupper = np.array([180,255,255])
balllower = np.array([5,200,180])
ballupper = np.array([25,255,255])
goallower = np.array([0,0,254])
goalupper = np.array([1,1,255])
LineareaTH = 3000
BallSizeTH = 100

def decode_fourcc(v):
    v = int(v)
    return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])

WIDTH = 320
HEIGHT = 240
FPS = 5

# フォーマット・解像度・FPSの設定
#cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
cap.set(cv2.CAP_PROP_FPS, FPS)

# フォーマット・解像度・FPSの取得
fourcc = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
fps = cap.get(cv2.CAP_PROP_FPS)
print("fourcc:{} fps:{} width:{} height:{}".format(fourcc, fps, width, height))

TMP_FOLDER_PATH = "./tmp/"
MTX_PATH = TMP_FOLDER_PATH + "mtx.csv"
DIST_PATH = TMP_FOLDER_PATH + "dist.csv"

def loadCalibrationFile(mtx_path, dist_path):
    try:
        mtx = np.loadtxt(mtx_path, delimiter=',')
        dist = np.loadtxt(dist_path, delimiter=',')
    except Exception as e:
        raise e
    return mtx, dist

def main():
    ret, frame = cap.read()
    mtx, dist = loadCalibrationFile(MTX_PATH, DIST_PATH)
    frame_undistort = cv2.undistort(frame, mtx, dist, None) # 内部パラメータを元に画像補正
    lane_shape = np.float32([(46, 0), (252, 24), (15, 240), (315, 220)])
    img_shape = np.float32([(0,0),(345,0),(0,395),(345,395)])
    M = cv2.getPerspectiveTransform(lane_shape, img_shape)
    BEV = cv2.warpPerspective(frame_undistort, M, (345, 395)) #実際の寸法(mm)に合わせて変換pixel=mm
    return BEV
