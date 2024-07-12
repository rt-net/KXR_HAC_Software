import math
import time
import sys
import os

import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import parameterfile

def decode_fourcc(v): #デコード用関数 動画コーデック(FourCC)はfloatで表現されているためこの関数が必要
    v = int(v) #intにキャスト(型変換)
    return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)]) 
    #動画コーデックをビット演算し連結することでフォーマットを得る
    #元の戻り値の動画コーデックvは4バイト　フォーマットの4つの1バイト文字の連結
    #ビットシフトでそれぞれの文字コードを末尾に取り出し、0xFF(11111111)との&演算で文字コードを得る
    
def load_calibration_file(mtx_path, dist_path): #キャリブレーションパラメータの読み込み
    try:
        mtx = np.loadtxt(mtx_path, delimiter=',')
        dist = np.loadtxt(dist_path, delimiter=',')
    except Exception as e:
        raise e
    return mtx, dist #キャリブレーションパラメータ配列を返す

class VisionLibrary:
    def __init__(self):             
        print("[Initializing camera...]")
        self.cap = cv2.VideoCapture(0) #デバイス番号を0で指定しインスタンス生成
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #カメラ画像取得の際のバッファを設定
        print("     ", type(self.cap)) #カメラ画像の取得元クラス表示
        print("     Camera setting OK: ", self.cap.isOpened()) #カメラ画像が読み込まれているか表示(Trueが正常)

        # フォーマット・解像度・FPSの設定
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V')) #フォーマット指定　使用するカメラに合わせる
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, parameterfile.CAMERA_FRAME_WIDTH) #幅指定　使用するカメラに合わせる
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, parameterfile.CAMERA_FRAME_HEIGHT) #高さ指定　使用するカメラに合わせる
        self.cap.set(cv2.CAP_PROP_FPS, parameterfile.CAMERA_FPS) #FPS指定　高いと処理が間に合わないため，低めに設定

        # フォーマット・解像度・FPSの取得
        fourcc = decode_fourcc(self.cap.get(cv2.CAP_PROP_FOURCC)) #動画コーデックをデコードしたものを取得
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) #設定された幅を取得
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) #設定された高さを取得
        fps = self.cap.get(cv2.CAP_PROP_FPS) #設定されたFPSを取得
        print("     fourcc:{} fps:{} width:{} height:{}".format(fourcc, fps, width, height)) #取得したフォーマット、解像度、FPSを表示
        print("[Camera initialized successfully]")
        
        #歪み補正パラメータが配置されたディレクトリパス指定
        self.MTX_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tmp', "mtx.csv")
        self.DIST_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'tmp', "dist.csv")
        
        #各要素の検出状態の初期化
        self.is_found_edge = False #フィールドのエッジが見えているかどうかのT/F
        self.is_found_ball = False #ボールが見えているかどうかのT/F
        self.corner_type = "NONE" #見えているコーナーの種類（最初はNONE）
        self.corner_pixel_coordinate_x = 0
        self.corner_pixel_coordinate_y = 0
        
    def calibrate_img(self):
        """calibrate image using undistort and bird eye view transform
        Returns:
        --------
        calibrated image data
        """
        ret, frame = self.cap.read() #カメラ画像の読み込み　画像の配列は2つめの戻り値frameに格納 retは画像が読み込めたかのbool値が入る
        mtx, dist = load_calibration_file(self.MTX_PATH, self.DIST_PATH) #キャリブレーションパラメータ配列を得る
        frame_undistort = cv2.undistort(frame, mtx, dist, None) # パラメータを元に画像補正
        lane_shape = np.float32([parameterfile.BEV_TOP_LEFT, 
                                 parameterfile.BEV_TOP_RIGHT, 
                                 parameterfile.BEV_BOTTOM_LEFT, 
                                 parameterfile.BEV_BOTTOM_RIGHT]) #鳥瞰図変換のパラメータ　画角内に4つの角が収まる長方形を置いたときのそれぞれの角の場所（すでにparameterfileに入力済み）
        img_shape = np.float32([(0,0),
                                (parameterfile.BEV_FRAME_WIDTH_MM,0),
                                (0,parameterfile.BEV_FRAME_HEIGHT_MM),
                                (parameterfile.BEV_FRAME_WIDTH_MM, parameterfile.BEV_FRAME_HEIGHT_MM)]) #上記、鳥瞰図変換のパラメータ取得に用いた長方形の大きさ
        BEV_transform_parameter = cv2.getPerspectiveTransform(lane_shape, img_shape) #鳥瞰図変換用のパラメータを得る
        self.BEV_img = cv2.warpPerspective(frame_undistort, 
                                       BEV_transform_parameter, 
                                       (parameterfile.BEV_FRAME_WIDTH_MM, parameterfile.BEV_FRAME_HEIGHT_MM)) #実際の寸法(mm)に合わせて鳥瞰図変換pixel=mm
        
        return self.BEV_img #歪み補正、鳥瞰図変換後の画像を返す
       
    def detect_ball(self): #ボール検出の関数
        """detect ball and its coordinate using color identification
        Returns:
        --------
        ball coordinate x, y
        """
        frame = self.BEV_img #キャリブレーション後画像の読み込み         
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, parameterfile.BALL_COLOR_MIN, parameterfile.BALL_COLOR_MAX) #ボール色をマスク
        ball_pixel_area = cv2.countNonZero(frame_mask) #ボールのマスクの画素数を取得
        ball_center_of_gravity = cv2.moments(frame_mask, False) #ボールの重心座標を取得
        
        #もしボール画素数が存在判定の閾値を超えていたら
        if ball_pixel_area > parameterfile.BALL_PIXEL_AREA_THRESHOLD:
            self.ball_pixel_coordinate_x,self.ball_pixel_coordinate_y = int(ball_center_of_gravity["m10"]/ball_center_of_gravity["m00"]), int(ball_center_of_gravity["m01"]/ball_center_of_gravity["m00"]) #ボールの重心座標を得る
        else:
            self.ball_pixel_coordinate_x = 0
            self.ball_pixel_coordinate_y = 0
        
        if self.ball_pixel_coordinate_x == 0 and self.ball_pixel_coordinate_y == 0:
            self.is_found_ball = False #ボールが見つかっていないとき、ball_pixel_coordinate_xとball_pixel_coordinate_yには0が格納される
        else:
            self.is_found_ball = True
        
        return self.ball_pixel_coordinate_x, self.ball_pixel_coordinate_y #BEV画像中のボールのx,y座標を返す
    
    def detect_ball_wide(self): #ボール検出の関数（BEV画像ではなく、カメラ画像そのままの広い画角でボールを検出する）
        """detect ball and its coordinate using color identification
        use wide view angle
        Returns:
        --------
        ball coordinate x, y
        """
        ret, frame = self.cap.read() #カメラ画像の読み込み　画像の配列は2つめの戻り値frameに格納 retは画像が読み込めたかのbool値が入る
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, parameterfile.BALL_COLOR_MIN, parameterfile.BALL_COLOR_MAX) #ボール色をマスク
        ball_pixel_area = cv2.countNonZero(frame_mask) #ボールのマスクの画素数を取得
        ball_center_of_gravity = cv2.moments(frame_mask, False) #ボールの重心座標を取得
        
        if ball_pixel_area > parameterfile.BALL_PIXEL_AREA_THRESHOLD_WIDE:
            self.ball_pixel_coordinate_x_wide,self.ball_pixel_coordinate_y_wide = int(ball_center_of_gravity["m10"]/ball_center_of_gravity["m00"]) , int(ball_center_of_gravity["m01"]/ball_center_of_gravity["m00"]) #ボールの重心座標を得る
        else:
            self.ball_pixel_coordinate_x_wide = 0
            self.ball_pixel_coordinate_y_wide = 0
        
        if self.ball_pixel_coordinate_x_wide == 0 and self.ball_pixel_coordinate_y_wide == 0:
            self.is_found_ball = False #ボールが見つかっていないとき、ball_pixel_coordinate_x_wideとball_pixel_coordinate_y_wideには0が格納される
        else:
            self.is_found_ball = True
        
        return self.ball_pixel_coordinate_x_wide, self.ball_pixel_coordinate_y_wide #カメラ画像中のボールのx,y座標を返す    

    def detect_corner(self): #コーナー検出の関数
        """detect corner and its type using pattern matching
        Returns:
        --------
        corner type, coordinate x, y
        """
        frame = self.BEV_img #キャリブレーション後画像の読み込み        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask_low = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_LOW, parameterfile.FIELD_COLOR_MAX_LOW)   #エッジ赤線をマスク
        frame_mask_high = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_HIGH, parameterfile.FIELD_COLOR_MAX_HIGH)   #エッジ赤線をマスク
        frame_mask = frame_mask_high | frame_mask_low
        
        # 処理対象画像の各画素に対して、テンプレート画像との類似度を算出する
        match_left = cv2.matchTemplate(frame_mask, parameterfile.LEFT_CORNER_TEMPLATE, cv2.TM_CCOEFF_NORMED)
        match_right = cv2.matchTemplate(frame_mask, parameterfile.RIGHT_CORNER_TEMPLATE, cv2.TM_CCOEFF_NORMED)
       
        # 閾値に基づき類似度の高い画素を得る　値はlist
        left_corner_pixel_coordinate = np.where(match_left >= parameterfile.TEMPLATE_MATCH_THRESHOLD)
        right_corner_pixel_coordinate = np.where(match_right >= parameterfile.TEMPLATE_MATCH_THRESHOLD)
        
        if left_corner_pixel_coordinate[0].any(): #左コーナーが検出されているとき
            self.corner_pixel_coordinate_y = sum(left_corner_pixel_coordinate[0])/len(left_corner_pixel_coordinate[0]) #左コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_pixel_coordinate_x = sum(left_corner_pixel_coordinate[1])/len(left_corner_pixel_coordinate[1]) #左コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = "RIGHT" #コーナー種別を1に設定
        elif right_corner_pixel_coordinate[0].any(): #右コーナーが検出されているとき
            self.corner_pixel_coordinate_y = sum(right_corner_pixel_coordinate[0])/len(right_corner_pixel_coordinate[0]) #右コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_pixel_coordinate_x = sum(right_corner_pixel_coordinate[1])/len(right_corner_pixel_coordinate[1]) #右コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = "LEFT" #コーナー種別を2に設定
        else: #何も検出されなかったとき
            #座標を0に設定、コーナー種別はNONE
            self.corner_pixel_coordinate_y = 0 
            self.corner_pixel_coordinate_x = 0
            self.corner_type = "NONE"
            
        return self.corner_type, self.corner_pixel_coordinate_x, self.corner_pixel_coordinate_y #コーナー座標、種別を返す
    
    def detect_corner_wide(self): #コーナー検出の関数（BEV画像ではなく、カメラ画像そのままの広い画角でボールを検出する）
        """detect corner and its type using pattern matching
        use wide view angle
        Returns:
        --------
        corner type, coordinate x, y
        """
        ret, frame = self.cap.read()     
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask_low = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_LOW, parameterfile.FIELD_COLOR_MAX_LOW)   #エッジ赤線をマスク
        frame_mask_high = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_HIGH, parameterfile.FIELD_COLOR_MAX_HIGH)   #エッジ赤線をマスク
        frame_mask = frame_mask_high | frame_mask_low
        
        # 処理対象画像の各画素に対して、テンプレート画像との類似度を算出する
        match_left = cv2.matchTemplate(frame_mask, parameterfile.LEFT_CORNER_TEMPLATE_WIDE, cv2.TM_CCOEFF_NORMED)
        match_right = cv2.matchTemplate(frame_mask, parameterfile.RIGHT_CORNER_TEMPLATE_WIDE, cv2.TM_CCOEFF_NORMED)
       
        # 閾値に基づき類似度の高い画素を得る　値はlist
        left_corner_pixel_coordinate = np.where(match_left >= parameterfile.TEMPLATE_MATCH_THRESHOLD_WIDE)
        right_corner_pixel_coordinate = np.where(match_right >= parameterfile.TEMPLATE_MATCH_THRESHOLD_WIDE)
        
        if left_corner_pixel_coordinate[0].any(): #左コーナーが検出されているとき
            self.corner_pixel_coordinate_y = sum(left_corner_pixel_coordinate[0])/len(left_corner_pixel_coordinate[0]) #左コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_pixel_coordinate_x = sum(left_corner_pixel_coordinate[1])/len(left_corner_pixel_coordinate[1]) #左コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = "RIGHT_WIDE" #コーナー種別を1に設定
        elif right_corner_pixel_coordinate[0].any(): #右コーナーが検出されているとき
            self.corner_pixel_coordinate_y = sum(right_corner_pixel_coordinate[0])/len(right_corner_pixel_coordinate[0]) #右コーナーのy座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_pixel_coordinate_x = sum(right_corner_pixel_coordinate[1])/len(right_corner_pixel_coordinate[1]) #右コーナーのx座標を取得 類似度が閾値を上回る箇所の座標の平均を取る
            self.corner_type = "LEFT_WIDE" #コーナー種別を2に設定
        else:
            self.corner_type = "NONE"
            
        return self.corner_type, self.corner_pixel_coordinate_x, self.corner_pixel_coordinate_y #コーナー座標、種別を返す
    
    def detect_edge_using_numpy_calc(self): #エッジ検出の関数
        """detect field edge using numpy calculation
        Returns:
        --------
        edge angle, slope, intercept
        """
        frame = self.BEV_img #キャリブレーション後画像の読み込み
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask_low = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_LOW, parameterfile.FIELD_COLOR_MAX_LOW)   #エッジ赤線をマスク
        frame_mask_high = cv2.inRange(hsv, parameterfile.FIELD_COLOR_MIN_HIGH, parameterfile.FIELD_COLOR_MAX_HIGH)   #エッジ赤線をマスク
        frame_mask = frame_mask_high | frame_mask_low

        blur = cv2.medianBlur(frame_mask,parameterfile.BLUR_FILTER_SIZE) #ぼかしフィルタ
        line_center_of_gravity = cv2.moments(blur, False) #線の重心座標を得る

        line_pixel_area = cv2.countNonZero(blur) #線の面積を得る
        
        self.angle = 0 #エッジ角度の初期値
        self.slope = 0 #エッジの傾き
        self.intercept = 0 #エッジの切片
        
        if line_pixel_area > parameterfile.EDGE_PIXEL_AREA_THRESHOLD:#見えるエッジの面積がエッジ 存在判定の閾値を超えた時      
            self.center_of_gravity_x,self.center_of_gravity_y= int(line_center_of_gravity["m10"]/line_center_of_gravity["m00"]), int(line_center_of_gravity["m01"]/line_center_of_gravity["m00"]) #線の重心座標を代入
    
            field_edges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジを強調する
            
            dilation_filter = np.ones((2,2),np.uint8) #2×2の膨張フィルタを用意
            dilation = cv2.dilate(field_edges,dilation_filter,iterations = 1) #エッジを膨張させる
            
            lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換で直線を検出
            LINE_LENGTH = 1000 #描画する線の長さを指定
            rho_total = 0 #ρの合計を初期化
            theta_total = 0 #θの合計を初期化

            if type(lines) == np.ndarray: #エッジが検出されている時
                theta_std_dev = np.nanstd(lines, 0)
                theta_std_dev = theta_std_dev[0][1] #θの標準偏差を得る
                
                #線のパラメータを不連続なρとθで表すためパラメータ平均値の算出に場合分けが必要
                #画角垂直方向に線がある時θは0付近とπ付近になり得るため標準偏差に基づいて場合分けを行う
                line_parameter_list = lines[:,0]
                rho = line_parameter_list[:,0] #ハフ変換で検出した線のパラメータからρを抜き出す
                theta = line_parameter_list[:,1] #ハフ変換で検出した線のパラメータからθを抜き出す
                
                if theta_std_dev < 1: #θの標準偏差が1未満である時
                    theta_list = theta
                    rho_list = rho
                else: #θの標準偏差が1以上の時                 
                    theta_list = np.where(theta>math.pi/2, theta-math.pi, theta)
                    rho_list = np.where(theta>math.pi/2, -rho, rho)
                
                rho_average = np.average(rho_list)
                theta_average = np.average(theta_list) 
                                       
                a = math.cos(theta_average)
                b = math.sin(theta_average)
                x0 = a * rho_average
                y0 = b * rho_average
                
                self.x1_average = (x0 - LINE_LENGTH * b)
                self.x2_average = (x0 + LINE_LENGTH * b)
                self.y1_average = (y0 + LINE_LENGTH * a)
                self.y2_average = (y0 - LINE_LENGTH * a)           
                
                self.slope = (self.y2_average-self.y1_average)/(self.x2_average-self.x1_average) #傾きを導出
                self.intercept = self.y1_average - self.slope*self.x1_average #切片を導出
                
                self.angle = int(math.degrees(math.atan(-(self.y2_average-self.y1_average)/(self.x2_average-self.x1_average)))) #角度の計算(度)

                #線の角度について機体前後方向と平行が0度になるように計算
                if self.angle < 0:
                    self.angle = -(self.angle + 90) 
                else:
                    self.angle = -(self.angle - 90)
                
        if self.slope == 0 and self.intercept == 0:
            self.is_found_edge = False#エッジが見つかっていないとき、edge_slopeとedge_interceptには0が格納される
        else:
            self.is_found_edge = True
            
        return self.angle, self.slope, self.intercept #エッジ角度、エッジ切片を返す
    
    def detect_goal(self):
        """detect goal using numpy calculation
        Returns:
        --------
        goal angle, slope, intercept
        """
        frame = self.BEV_img #キャリブレーション後画像の読み込み        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, parameterfile.GOAL_COLOR_MIN, parameterfile.GOAL_COLOR_MAX)   #エッジ赤線をマスク
        self.goal = frame_mask
                
        blur = cv2.medianBlur(frame_mask,parameterfile.BLUR_FILTER_SIZE_GOAL) #ぼかしフィルタ
        line_center_of_gravity = cv2.moments(blur, False) #線の重心座標を得る

        line_pixel_area = cv2.countNonZero(blur) #線の面積を得る
        
        self.goalline_angle = 0 #エッジ角度の初期値
        self.goalline_slope = 0 
        self.goalline_intercept = 0 #a、bどちらも0を初期値
        
        #--------------------
        # Apply Canny edge detection
        edges = cv2.Canny(frame_mask, 50, 150, apertureSize=3)
        dilation_filter = np.ones((2,2),np.uint8) #膨張フィルタ
        dilation = cv2.dilate(edges,dilation_filter,iterations = 3) #膨張

        # 長方形処理
        # Find contours
        contours, _ = cv2.findContours(dilation, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw rectangles around detected contours
        count = 0
        goal_line = False
        
        for contour in contours:
            # Get bounding rectangle coordinates for the contour
            x, y, w, h = cv2.boundingRect(contour)
            # print("rect ID:", count, "width:", w, "height", h)
            if w > 200 and w/h >= 2:
                goal_line = True
                break
            cv2.rectangle(self.goal, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw rectangle with green color
            count +=1
            if count > 5:
                break
        #--------------------
        
        if line_pixel_area > parameterfile.GOAL_PIXEL_AREA_THRESHOLD and goal_line == True:#見えるエッジの面積がエッジ 存在判定の閾値を超えた時      
            self.center_of_gravity_x,self.center_of_gravity_y= int(line_center_of_gravity["m10"]/line_center_of_gravity["m00"]), int(line_center_of_gravity["m01"]/line_center_of_gravity["m00"]) #線の重心座標を代入
    
            field_edges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジ検出
            
            dilation_filter = np.ones((2,2),np.uint8) #膨張フィルタ
            dilation = cv2.dilate(field_edges,dilation_filter,iterations = 1) #膨張
            
            lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換
            LINE_LENGTH = 1000 #描画する線の長さ
            rho_total = 0
            theta_total = 0

            if type(lines) == np.ndarray: #エッジが検出されている時
                theta_std_dev = np.nanstd(lines, 0)
                theta_std_dev = theta_std_dev[0][1] #θの標準偏差を得る
                
                #線のパラメータを不連続なρとθで表すためパラメータ平均値の算出に場合分けが必要
                #画角垂直方向に線がある時θは0付近とπ付近になり得るため標準偏差に基づいて場合分けを行う
                line_parameter_list = lines[:,0]
                rho = line_parameter_list[:,0]
                theta = line_parameter_list[:,1]
                
                if theta_std_dev < 1: #θの標準偏差が1未満である時
                    theta_list = theta
                    rho_list = rho
                elif theta_std_dev >= 1: #θの標準偏差が1以上の時                 
                    theta_list = np.where(theta>math.pi/2, theta-math.pi, theta)
                    rho_list = np.where(theta>math.pi/2, -rho, rho)
                
                rho_average = np.average(rho_list)
                theta_average = np.average(theta_list) 
                                       
                a = math.cos(theta_average)
                b = math.sin(theta_average)
                x0 = a * rho_average
                y0 = b * rho_average
                
                self.goalline_x1_average = (x0 - LINE_LENGTH * b)
                self.goalline_x2_average = (x0 + LINE_LENGTH * b)
                self.goalline_y1_average = (y0 + LINE_LENGTH * a)
                self.goalline_y2_average = (y0 - LINE_LENGTH * a)           
                
                self.goalline_slope = (self.goalline_y2_average-self.goalline_y1_average)/(self.goalline_x2_average-self.goalline_x1_average) #傾きを導出
                self.goalline_intercept = self.goalline_y1_average - self.goalline_slope*self.goalline_x1_average #切片を導出
                
                self.goalline_angle = int(math.degrees(math.atan(-(self.goalline_y2_average-self.goalline_y1_average)/(self.goalline_x2_average-self.goalline_x1_average)))) #角度の計算(度)

                #線の角度について機体前後方向と平行が0度になるように計算
                if self.goalline_angle < 0:
                    self.goalline_angle = -(self.goalline_angle + 90) 
                else:
                    self.goalline_angle = -(self.goalline_angle - 90)
        
        if self.goalline_slope == 0 and self.goalline_intercept == 0:
            self.is_found_goal = False
        else:
            self.is_found_goal = True
        
        return self.goalline_angle, self.goalline_slope, self.goalline_intercept #エッジ角度、エッジ切片を返す
    
    def detect_ball_line(self):
        frame = self.BEV_img #キャリブレーション後画像の読み込み        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, parameterfile.BALL_LINE_COLOR_MIN, parameterfile.BALL_LINE_COLOR_MAX)   #エッジ赤線をマスク
        
        self.ball_line = frame_mask
        
        blur = cv2.medianBlur(frame_mask,parameterfile.BLUR_FILTER_SIZE_BALL_LINE) #ぼかしフィルタ
        line_center_of_gravity = cv2.moments(blur, False) #線の重心座標を得る

        line_pixel_area = cv2.countNonZero(blur) #線の面積を得る
        
        self.ballline_angle = 0 #エッジ角度の初期値
        self.ballline_slope = 0 
        self.ballline_intercept = 0 #a、bどちらも0を初期値
        
        if line_pixel_area > parameterfile.BALL_LINE_PIXEL_AREA_THRESHOLD:#見えるエッジの面積がエッジ 存在判定の閾値を超えた時      
            self.center_of_gravity_x,self.center_of_gravity_y= int(line_center_of_gravity["m10"]/line_center_of_gravity["m00"]), int(line_center_of_gravity["m01"]/line_center_of_gravity["m00"]) #線の重心座標を代入
    
            field_edges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジ検出
            
            dilation_filter = np.ones((2,2),np.uint8) #膨張フィルタ
            dilation = cv2.dilate(field_edges,dilation_filter,iterations = 1) #膨張
            
            lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換
            LINE_LENGTH = 1000 #描画する線の長さ
            rho_total = 0
            theta_total = 0

            if type(lines) == np.ndarray: #エッジが検出されている時
                theta_std_dev = np.nanstd(lines, 0)
                theta_std_dev = theta_std_dev[0][1] #θの標準偏差を得る
                
                #線のパラメータを不連続なρとθで表すためパラメータ平均値の算出に場合分けが必要
                #画角垂直方向に線がある時θは0付近とπ付近になり得るため標準偏差に基づいて場合分けを行う
                line_parameter_list = lines[:,0]
                rho = line_parameter_list[:,0]
                theta = line_parameter_list[:,1]
                
                if theta_std_dev < 1: #θの標準偏差が1未満である時
                    theta_list = theta
                    rho_list = rho
                elif theta_std_dev >= 1: #θの標準偏差が1以上の時                 
                    theta_list = np.where(theta>math.pi/2, theta-math.pi, theta)
                    rho_list = np.where(theta>math.pi/2, -rho, rho)
                
                rho_average = np.average(rho_list)
                theta_average = np.average(theta_list) 
                                       
                a = math.cos(theta_average)
                b = math.sin(theta_average)
                x0 = a * rho_average
                y0 = b * rho_average
                
                self.ballline_x1_average = (x0 - LINE_LENGTH * b)
                self.ballline_x2_average = (x0 + LINE_LENGTH * b)
                self.ballline_y1_average = (y0 + LINE_LENGTH * a)
                self.ballline_y2_average = (y0 - LINE_LENGTH * a)           
                
                self.ballline_slope = (self.ballline_y2_average-self.ballline_y1_average)/(self.ballline_x2_average-self.ballline_x1_average) #傾きを導出
                self.ballline_intercept = self.ballline_y1_average - self.ballline_slope*self.ballline_x1_average #切片を導出
                
                self.ballline_angle = int(math.degrees(math.atan(-(self.ballline_y2_average-self.ballline_y1_average)/(self.ballline_x2_average-self.ballline_x1_average)))) #角度の計算(度)

                #線の角度について機体前後方向と平行が0度になるように計算
                if self.ballline_angle < 0:
                    self.ballline_angle = -(self.ballline_angle + 90) 
                else:
                    self.ballline_angle = -(self.ballline_angle - 90)
        
        return self.ballline_angle, self.ballline_slope, self.ballline_intercept #エッジ角度、エッジ切片を返す
    
    def display_resultimg(self):#結果画像の表示用関数
        """return result image
        Returns:
        --------
        result image data
        """
        ret, result = self.cap.read()

        if self.corner_type != "NONE": #コーナーが存在する時 corner_type == 0 の時は存在しない
            cv2.rectangle(result, 
                          (int(self.corner_pixel_coordinate_x), int(self.corner_pixel_coordinate_y)), 
                          (int(self.corner_pixel_coordinate_x)+30, int(self.corner_pixel_coordinate_y)+30), 
                          (255, 255, 0), 2) 
            if self.corner_type == "LEFT_WIDE":
                cv2.putText(result,
                            text="Left",
                            org=(int(self.corner_pixel_coordinate_x+10), int(self.corner_pixel_coordinate_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            elif self.corner_type == "RIGHT_WIDE":
                cv2.putText(result,
                            text="Right",
                            org=(int(self.corner_pixel_coordinate_x+10), int(self.corner_pixel_coordinate_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
    
        result = self.calibrate_img() #キャリブレーション後画像の読み込みと結果表示画像の作成
        
        if self.is_found_edge == True and self.corner_type == "NONE": #エッジが存在するとき　かつ　コーナーが見えていないとき(コーナーが存在するとエッジの直線近似の前提が崩れる)
            #見えている線の合成の描画
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

        if self.is_found_ball == True:     
            #ボール重心の描画
            cv2.circle(result, (self.ball_pixel_coordinate_x,self.ball_pixel_coordinate_y), 4, 100, 2, 4) #ボール重心座標にマーク
            cv2.putText(result,
                        text=str(self.ball_pixel_coordinate_x),
                        org=(self.ball_pixel_coordinate_x+10, self.ball_pixel_coordinate_y+10),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)
            cv2.putText(result,
                        text=str(self.ball_pixel_coordinate_y),
                        org=(self.ball_pixel_coordinate_x+10, self.ball_pixel_coordinate_y+30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)            
            
        if self.corner_type != "NONE": #コーナーが存在する時 corner_type == 0 の時は存在しない
            cv2.rectangle(result, 
                          (int(self.corner_pixel_coordinate_x), int(self.corner_pixel_coordinate_y)), 
                          (int(self.corner_pixel_coordinate_x)+parameterfile.CORNER_TEMPLATE_WIDTH, int(self.corner_pixel_coordinate_y)+parameterfile.CORNER_TEMPLATE_HEIGHT), 
                          (255, 255, 0), 2) 
            if self.corner_type == "LEFT":
                cv2.putText(result,
                            text="Left",
                            org=(int(self.corner_pixel_coordinate_x+10), int(self.corner_pixel_coordinate_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            elif self.corner_type == "RIGHT":
                cv2.putText(result,
                            text="Right",
                            org=(int(self.corner_pixel_coordinate_x+10), int(self.corner_pixel_coordinate_y+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
                
        return result