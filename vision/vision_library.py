import math
import time
import cv2
import numpy as np
import vision.parameters

def decode_fourcc(v): #デコード用関数 動画コーデック(FourCC)はfloatで表現されているためこの関数が必要
    v = int(v) #intにキャスト(型変換)
    return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)]) 
    #動画コーデックをビット演算し連結することでフォーマットを得る
    #元の戻り値の動画コーデックvは4バイト　フォーマットの4つの1バイト文字の連結
    #ビットシフトでそれぞれの文字コードを末尾に取り出し、0bFF(11111111)との&演算で文字コードを得る
    
def load_calibration_file(mtx_path, dist_path): #キャリブレーションパラメータの読み込み
    try:
        mtx = np.loadtxt(mtx_path, delimiter=',')
        dist = np.loadtxt(dist_path, delimiter=',')
    except Exception as e:
        raise e
    return mtx, dist #パラメータ配列を返す

class VisionLibrary:
    def __init__(self):             
        print("[カメラ初期化中]")
            
        self.cap = cv2.VideoCapture(0) #デバイス番号を0で指定しインスタンス生成
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #カメラ画像取得の際のバッファを設定
        print(type(self.cap)) #カメラ画像の取得元クラス表示
        print("カメラ設定正常: ",self.cap.isOpened()) #カメラ画像が読み込まれているか表示(Trueが正常)

        # フォーマット・解像度・FPSの設定
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V')) #フォーマット指定
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, vision.parameters.camera_width) #幅指定
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, vision.parameters.camera_height) #高さ指定
        self.cap.set(cv2.CAP_PROP_FPS, vision.parameters.camera_FPS) #FPS指定

        # フォーマット・解像度・FPSの取得
        fourcc = decode_fourcc(self.cap.get(cv2.CAP_PROP_FOURCC)) #動画コーデックをデコードしたものを代入
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) #幅を代入
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) #高さを代入
        fps = self.cap.get(cv2.CAP_PROP_FPS) #FPSを代入
        print("fourcc:{} fps:{} width:{} height:{}".format(fourcc, fps, width, height)) #フォーマット、解像度、FPSを表示
        
        #歪み補正パラメータが配置されたディレクトリパス指定
        TMP_FOLDER_PATH = "./tmp/" 
        self.MTX_PATH = TMP_FOLDER_PATH + "mtx.csv" 
        self.DIST_PATH = TMP_FOLDER_PATH + "dist.csv"
        
        print("[カメラ初期化完了]")
        
    def calibrate_img(self):      
        ret, frame = self.cap.read() #カメラ画像の読み込み　画像の配列は2つめの戻り値frameに格納 retは画像が読み込めたかのbool値が入る
        mtx, dist = load_calibration_file(self.MTX_PATH, self.DIST_PATH) #キャリブレーションパラメータ配列を得る
        frame_undistort = cv2.undistort(frame, mtx, dist, None) # パラメータを元に画像補正
        lane_shape = np.float32([vision.parameters.BEV_top_left, 
                                 vision.parameters.BEV_top_right, 
                                 vision.parameters.BEV_bottom_left, 
                                 vision.parameters.BEV_bottom_right]) #鳥瞰図変換のパラメータ　画角内に4つの角が収まる長方形を置いたときの角の場所
        img_shape = np.float32([(0,0),
                                (vision.parameters.BEV_width,0),
                                (0,vision.parameters.BEV_height),
                                (vision.parameters.BEV_width, vision.parameters.BEV_height)]) #上記長方形の大きさ
        BEV_transform_parameter = cv2.getPerspectiveTransform(lane_shape, img_shape) #鳥瞰図変換パラメータMを得る
        self.BEV = cv2.warpPerspective(frame_undistort, BEV_transform_parameter, (vision.parameters.BEV_width, vision.parameters.BEV_height)) #実際の寸法(mm)に合わせて鳥瞰図変換pixel=mm
        
        return self.BEV #歪み補正、鳥瞰図変換後の画像を返す
        
    def detect_edge(self): #エッジ検出の関数
        frame = self.calibrate_img() #キャリブレーション後画像の読み込み
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, vision.parameters.field_lower, vision.parameters.field_upper)   #エッジ赤線をマスク

        blur = cv2.medianBlur(frame_mask,vision.parameters.blur_filter_size) #ぼかしフィルタ
        line_center_of_gravity = cv2.moments(blur, False) #線の重心座標を得る

        line_pixel_area = cv2.countNonZero(blur) #線の面積を得る
        
        angle = 0 #エッジ角度の初期値
        self.edge_a = 0 
        self.edge_b = 0 #a、bどちらも0を初期値
        
        if line_pixel_area > vision.parameters.line_pixel_area_threshhold:#見えるエッジの面積がエッジ 存在判定の閾値を超えた時      
            self.center_of_gravity_x,self.center_of_gravity_y= int(line_center_of_gravity["m10"]/line_center_of_gravity["m00"]), int(line_center_of_gravity["m01"]/line_center_of_gravity["m00"]) #線の重心座標を代入
    
            field_edges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジ検出
            
            dilation_filter = np.ones((2,2),np.uint8) #膨張フィルタ
            dilation = cv2.dilate(field_edges,dilation_filter,iterations = 1) #膨張
            
            lines = []
            theta_list = []
            lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換
            line_length = 1000 #描画する線の長さ
            rho_total = 0
            theta_total = 0

            if type(lines) == np.ndarray: #エッジが検出されている時
                for line in lines: #検出された全ての線のパラメータθについてのリストを作成
                    theta_list.append(line[0][1])
                theta_std_dev = np.nanstd(theta_list) #θの標準偏差を得る
                
                #線のパラメータを不連続なρとθで表すためパラメータ平均値の算出に場合分けが必要
                #画角垂直方向に線がある時θは0付近とπ付近になり得るため標準偏差に基づいて場合分けを行う
                if theta_std_dev < 1: #θの標準偏差が1未満である時
                    for line in lines: #Hough変換で得た配列Linesについて繰り返す
                        rho = line[0][0]
                        theta = line[0][1]
                                            
                        theta_total = theta_total+theta   
                        rho_total = rho_total+rho
                else: #θの標準偏差が1以上の時
                    for line in lines: #Hough変換で得た配列Linesについて繰り返す
                        rho = line[0][0]
                        theta = line[0][1]
                                            
                        if theta > math.pi/2 : #線の傾きが負の時
                            rho = -rho #その場合でのrhoの符号は-
                            theta = math.pi-theta #パラメータθを1~pi/2の値に変換
                            theta_total = theta_total-theta
                        else:
                            theta_total = theta_total+theta
                            
                        rho_total = rho_total+rho
                    
                rho_average = rho_total/(len(lines))
                theta_average = theta_total/(len(lines))
                                                    
                a = math.cos(theta_average)
                b = math.sin(theta_average)
                x0 = a * rho_average
                y0 = b * rho_average
                
                self.x1_average = (x0 - line_length * b)
                self.x2_average = (x0 + line_length * b)
                self.y1_average = (y0 + line_length * a)
                self.y2_average = (y0 - line_length * a)           
                
                self.edge_a = (self.y2_average-self.y1_average)/(self.x2_average-self.x1_average) #傾きを導出
                self.edge_b = self.y1_average/(len(lines)) - self.edge_a*self.x1_average/(len(lines)) #切片を導出
                
                self.angle = int(math.degrees(math.atan(-(self.y2_average-self.y1_average)/(self.x2_average-self.x1_average)))) #角度の計算(度)

                #線の角度について機体前後方向と平行が0度になるように計算
                if self.angle < 0:
                    self.angle = -(self.angle + 90) 
                else:
                    self.angle = -(self.angle - 90)
                
            else: #エッジが検出されなかったとき
                self.edge_a = 0 
                self.edge_b = 0 #a、bどちらも0を代入    
                        
        else: #エッジの色が存在しない時
            self.edge_a = 0 
            self.edge_b = 0 #a、bどちらも0を代入 
                
        if self.edge_a == 0 and self.edge_b == 0:
            self.found_edge = False
        else:
            self.found_edge = True
            
        return self.edge_a, self.edge_b #エッジ角度、エッジ切片を返す
    
    def detect_ball(self): #ボール検出の関数
        frame = self.calibrate_img() #キャリブレーション後画像の読み込み
        #self.resultimg = frame #結果表示用画像の作成
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, vision.parameters.ball_lower, vision.parameters.ball_upper) #ボール色をマスク
        area = cv2.countNonZero(frame_mask) #ボールのマスクの画素数を取得
        ballcog = cv2.moments(frame_mask, False) #ボールの重心座標を取得
        
        #もしボール画素数が存在判定の閾値を超えていたら
        if area > vision.parameters.ball_pixel_area_threshold:
            self.ball_x,self.ball_y = int(ballcog["m10"]/ballcog["m00"]) , int(ballcog["m01"]/ballcog["m00"]) #ボールの重心座標を得る
        else:
            self.ball_x = 0
            self.ball_y = 0
        
        if self.ball_x == 0 and self.ball_y == 0:
            self.found_ball = False
        else:
            self.found_ball = True
        
        return self.ball_x, self.ball_y

    def detect_corner(self): #コーナー検出の関数
        frame = self.calibrate_img() #キャリブレーション後画像の読み込み        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame = cv2.inRange(hsv, vision.parameters.field_lower, vision.parameters.field_upper)   #エッジ赤線をマスク
        
        # 処理対象画像の各画素に対して、テンプレート画像との類似度を算出する
        match_left = cv2.matchTemplate(frame, vision.parameters.field_corner_left, cv2.TM_CCOEFF_NORMED)
        match_right = cv2.matchTemplate(frame, vision.parameters.field_corner_right, cv2.TM_CCOEFF_NORMED)
       
        # 閾値に基づき類似度の高い画素を得る　値はlist
        left_pattern_pixel_position = np.where(match_left >= vision.parameters.pattern_match_threshhold)
        right_pattern_pixel_position = np.where(match_right >= vision.parameters.pattern_match_threshhold)
        
        if left_pattern_pixel_position[0].any(): #左コーナーが検出されているとき
            self.corner_y = sum(left_pattern_pixel_position[0])/len(left_pattern_pixel_position[0]) #左コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_x = sum(left_pattern_pixel_position[1])/len(left_pattern_pixel_position[1]) #左コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = 1 #コーナー種別を1に設定
        elif right_pattern_pixel_position[0].any(): #右コーナーが検出されているとき
            self.corner_y = sum(right_pattern_pixel_position[0])/len(right_pattern_pixel_position[0]) #右コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_x = sum(right_pattern_pixel_position[1])/len(right_pattern_pixel_position[1]) #右コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = 2 #コーナー種別を2に設定
        else: #何も検出されなかったとき
            #座標、コーナー種別を0に設定
            self.corner_y = 0 
            self.corner_x = 0
            self.corner_type = 0
            
        return self.corner_type, self.corner_x, self.corner_y #コーナー座標、種別を返す
        
    def display_resultimg(self):#結果画像の表示用関数
        result = self.calibrate_img() #キャリブレーション後画像の読み込みと結果表示画像の作成
        
        if self.found_edge == True: #エッジが存在するとき 存在しないときエッジの値はどちらも0
            #見えている線の合成の描画
            if self.corner_type == 0: #コーナーが見えていないとき(コーナーが存在するとエッジの直線近似の前提が崩れる)
                if True:#self.center_of_gravity_x or self.center_of_gravity_y:
                    cv2.line(result, 
                            (int(self.x1_average), int(self.y1_average)), 
                            (int(self.x2_average), int(self.y2_average)), 
                            (0, 255, 255), thickness=2, lineType=cv2.LINE_4 )
                    
                    #線の角度(度)の画像への書き込み
                    cv2.putText(result,
                        text=str(self.angle),
                        org=(self.center_of_gravity_x+10, self.center_of_gravity_y+30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)

        if self.found_ball == True:     
            #ボール重心の描画
            cv2.circle(result, (self.ball_x,self.ball_y), 4, 100, 2, 4) #ボール重心座標にマーク
            cv2.putText(result,
                        text=str(self.ball_x),
                        org=(self.ball_x+10, self.ball_y+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)

            cv2.putText(result,
                        text=str(self.ball_y),
                        org=(self.ball_x+10, self.ball_y+30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)            
            
        if self.corner_type != 0: #コーナーが存在する時 corner_type == 0 の時は存在しない
            cv2.rectangle(result, 
                            (int(self.corner_x), int(self.corner_y)), 
                            (int(self.corner_x)+vision.parameters.corner_width, int(self.corner_y)+vision.parameters.corner_height), 
                            (255, 255, 0), 2) 
            if self.corner_type == 1:
                cv2.putText(result,
                            text="Left",
                            org=(int(self.corner_x+10), int(self.corner_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            elif self.corner_type == 2:
                cv2.putText(result,
                            text="Right",
                            org=(int(self.corner_x+10), int(self.corner_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            elif self.corner_type == 3:
                cv2.putText(result,
                            text="Left Inner",
                            org=(int(self.corner_x+10), int(self.corner_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
        return result