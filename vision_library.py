import math
import time
import cv2
import numpy as np
import parameters as p

class VisionLibrary:
       
    def __init__(self): 
        
        def decode_fourcc(v): 
            v = int(v)
            return "".join([chr((v >> 8 * i) & 0xFF) for i in range(4)])
            
        print("[初期化中]")
            
        self.cap = cv2.VideoCapture(0) #デバイス番号を0で指定しインスタンス生成
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1) #カメラ画像取得の際のバッファを設定
        ret, im = self.cap.read() #カメラ画像を読み込む
        print(type(self.cap)) #カメラ設定が問題ないか表示
        print("カメラ設定正常: ",self.cap.isOpened()) #カメラ画像が読み込まれているか表示(Trueが正常)

        # フォーマット・解像度・FPSの設定
        #cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V')) #フォーマット指定
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, p.camWIDTH) #幅指定
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, p.camHEIGHT) #高さ指定
        self.cap.set(cv2.CAP_PROP_FPS, p.camFPS) #FPS指定

        # フォーマット・解像度・FPSの取得
        fourcc = decode_fourcc(self.cap.get(cv2.CAP_PROP_FOURCC)) #フォーマットをデコードしたものを代入
        width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH) #幅を代入
        height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT) #高さを代入
        fps = self.cap.get(cv2.CAP_PROP_FPS) #FPSを代入
        print("fourcc:{} fps:{} width:{} height:{}".format(fourcc, fps, width, height)) #フォーマット、解像度、FPSを表示
        
        #歪み補正パラメータが配置されたディレクトリパス指定
        TMP_FOLDER_PATH = "./tmp/" 
        self.MTX_PATH = TMP_FOLDER_PATH + "mtx.csv" 
        self.DIST_PATH = TMP_FOLDER_PATH + "dist.csv"
        
        print("[初期化完了]")


    def calib_img(self):
        
        #キャリブレーションパラメータの読み込み
        def loadCalibrationFile(mtx_path, dist_path):
            try:
                mtx = np.loadtxt(mtx_path, delimiter=',')
                dist = np.loadtxt(dist_path, delimiter=',')
            except Exception as e:
                raise e
            return mtx, dist #パラメータ配列を返す
        
        ret, frame = self.cap.read() #カメラ画像の読み込み　画像の配列は2つめの戻り値frameに格納 retは画像が読み込めたかのbool値が入る
        mtx, dist = loadCalibrationFile(self.MTX_PATH, self.DIST_PATH) #キャリブレーションパラメータ配列を得る
        frame_undistort = cv2.undistort(frame, mtx, dist, None) # パラメータを元に画像補正
        lane_shape = np.float32([p.BEVtopleft, p.BEVtopright, p.BEVbottomleft, p.BEVbottomright]) #鳥瞰図変換のパラメータ　画角内に4つの角が収まる長方形を置いたときの角の場所
        img_shape = np.float32([(0,0),(p.BEVwidth,0),(0,p.BEVheight),(p.BEVwidth, p.BEVheight)]) #上記長方形の大きさ
        M = cv2.getPerspectiveTransform(lane_shape, img_shape) #鳥瞰図変換パラメータMを得る
        self.BEV = cv2.warpPerspective(frame_undistort, M, (p.BEVwidth, p.BEVheight)) #実際の寸法(mm)に合わせて鳥瞰図変換pixel=mm
        
        return self.BEV #歪み補正、鳥瞰図変換後の画像を返す
        
    def detect_edge(self): #エッジ検出の関数
        frame = self.calib_img() #キャリブレーション後画像の読み込み
        #self.resultimg = frame #結果表示用画像の作成
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, p.fieldlower, p.fieldupper)   #エッジ赤線をマスク

        Blur = cv2.medianBlur(frame_mask,p.ksize) #ぼかしフィルタ
        Linecog = cv2.moments(Blur, False) #線の重心座標を得る

        Linearea = cv2.countNonZero(Blur) #線の面積を得る
        
        angle = 0 #エッジ角度の初期値
        self.edgea = 0 
        self.edgeb = 0 #a、bどちらも0を初期値
        
        #見えるエッジの面積がエッジ 存在判定の閾値を超えた時       
        if Linearea > p.LineareaTH:

            self.xcog,self.ycog= int(Linecog["m10"]/Linecog["m00"]) , int(Linecog["m01"]/Linecog["m00"]) #線の重心座標を代入
            #cv2.circle(self.resultimg, (xcog,ycog), 4, 100, 2, 4) #重心を円で示す

            fieldedges = cv2.Canny(frame_mask, 50, 150, apertureSize = 3) #エッジ検出
            
            kernel = np.ones((2,2),np.uint8) #膨張フィルタ
            dilation = cv2.dilate(fieldedges,kernel,iterations = 1) #膨張

            Lines = cv2.HoughLines(dilation, 1, (np.pi/180), 80) #ハフ変換
            line_length = 1000 #描画する線の長さ

            x1total, x2total, y1total, y2total = 0, 0, 0, 0 #線のパラメータの平均を得るための合計値の初期値
            
            try:
                for line in Lines: #Hough変換で得た配列Linesについて繰り返す
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
                        b = y1 - a*x1
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
                
                
                self.angle = int(math.degrees(math.atan(-(y2total-y1total)/(x2total-x1total)))) #角度の計算(度)

                
                if self.angle < 0:
                    self.angle = -(self.angle + 90)
                else:
                    self.angle = -(self.angle - 90)
                    
                self.edgea = (y2total-y1total)/(x2total-x1total) #傾きを導出
                self.edgeb = y1total/(len(Lines)) - a*x1total/(len(Lines)) #切片を導出
                
                
                #-------------------------------------------------------------------
                # #線の角度(度)の画像への書き込み
                # cv2.putText(self.resultimg,
                #     text=str(angle),
                #     org=(xcog+10, ycog+30),
                #     fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                #     fontScale=0.6,
                #     color=(0, 255, 0),
                #     thickness=2,
                #     lineType=cv2.LINE_4)
                
                # #見えている線の合成の描画
                # cv2.line(
                #     self.resultimg, 
                #     (int(x1total/len(Lines)), int(y1total/len(Lines))), 
                #     (int(x2total/len(Lines)), int(y2total/len(Lines))), 
                #     (0, 0, 255), thickness=2, lineType=cv2.LINE_4 ) 
                #-------------------------------------------------------------------
                
                self.x1avg = int(x1total/len(Lines))
                self.y1avg = int(y1total/len(Lines))
                self.x2avg = int(x2total/len(Lines))
                self.y2avg = int(y2total/len(Lines))
                
            except: #エッジが検出されなかったとき
                self.edgea = 0 
                self.edgeb = 0 #a、bどちらも0を代入    
                        
            #return self.resultimg, self.edgea, self.edgeb #エッジ角度、エッジ切片を返す
    
        else: #エッジの色が存在しない時
            self.edgea = 0 
            self.edgeb = 0 #a、bどちらも0を代入 
        
            #return self.resultimg, self.edgea, self.edgeb #エッジ角度、エッジ切片を返す
            
        return self.edgea, self.edgeb #エッジ角度、エッジ切片を返す
    
    def detect_ball(self): #ボール検出の関数
        frame = self.calib_img() #キャリブレーション後画像の読み込み
        #self.resultimg = frame #結果表示用画像の作成
            
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame_mask = cv2.inRange(hsv, p.balllower, p.ballupper) #ボール色をマスク
        area = cv2.countNonZero(frame_mask) #ボールのマスクの画素数を取得
        ballcog = cv2.moments(frame_mask, False) #ボールの重心座標を取得
        
        #もしボール画素数が存在判定の閾値を超えていたら
        if area > p.BallSizeTH:
            ball = True
            self.ballx,self.bally= int(ballcog["m10"]/ballcog["m00"]) , int(ballcog["m01"]/ballcog["m00"]) #ボールの重心座標を得る
            
            #----------------------------------------------------------------------
            # cv2.circle(self.resultimg, (x,y), 4, 100, 2, 4) #ボール重心座標にマーク
            
            #重心座標の画像への書き込み
            # cv2.putText(self.resultimg,
            #             text=str(x),
            #             org=(x+10, y+10),
            #             fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            #             fontScale=0.6,
            #             color=(0, 255, 0),
            #             thickness=2,
            #             lineType=cv2.LINE_4)
            # cv2.putText(self.resultimg,
            #             text=str(y),
            #             org=(x+10, y+30),
            #             fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            #             fontScale=0.6,
            #             color=(0, 255, 0),
            #             thickness=2,
            #             lineType=cv2.LINE_4)
            #----------------------------------------------------------------------
            
            #return self.resultimg, self.ballx, self.bally

        else:
            ball = False
            self.ballx = 0
            self.bally = 0
            #return self.resultimg, self.ballx, self.bally
            
        return self.ballx, self.bally

    def detect_corner(self): #コーナー検出の関数
        frame = self.calib_img() #キャリブレーション後画像の読み込み
        #self.resultimg = frame #結果表示用画像の作成
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) #BEV図をhsv色空間へ変換
        frame = cv2.inRange(hsv, p.fieldlower, p.fieldupper)   #エッジ赤線をマスク
        
        # 処理対象画像に対して、テンプレート画像との類似度を算出する
        matchleft = cv2.matchTemplate(frame, p.fieldcornerleft, cv2.TM_CCOEFF_NORMED)
        matchright = cv2.matchTemplate(frame, p.fieldcornerright, cv2.TM_CCOEFF_NORMED)
        matchinnerleft = cv2.matchTemplate(frame, p.fieldinnercornerleft, cv2.TM_CCOEFF_NORMED)

        # 閾値に基づき類似度の高い部分を検出する
        matchfindleft = np.where(matchleft >= p.patternmatchTH)
        matchfindright = np.where(matchright >= p.patternmatchTH)
        matchfindinnerleft = np.where(matchinnerleft >= p.patternmatchTH)
    
        if len(matchfindleft[0]) != 0: #左コーナーが検出されているとき
            self.cornery = sum(matchfindleft[0])/len(matchfindleft[0])
            self.cornerx = sum(matchfindleft[1])/len(matchfindleft[1])            
            self.cornertype = 1
        
        elif len(matchfindright[0]) != 0: #右コーナーが検出されているとき
            self.cornery = sum(matchfindright[0])/len(matchfindright[0])
            self.cornerx = sum(matchfindright[1])/len(matchfindright[1])
            self.cornertype = 2
            
        
        elif len(matchfindinnerleft[0]) != 0: #左コーナー(内)が検出されているとき
            self.cornery = sum(matchfindinnerleft[0])/len(matchfindinnerleft[0])
            self.cornerx = sum(matchfindinnerleft[1])/len(matchfindinnerleft[1])
            self.cornertype = 3

        else:
            self.cornery = 0
            self.cornerx = 0
            self.cornertype = 0
            
        return self.cornertype, self.cornerx, self.cornery
        
        
    def disp_resultimg(self):#結果画像の表示用関数
        result = self.calib_img() #キャリブレーション後画像の読み込みと結果表示画像の作成
        
        try: #エッジが存在しない時は実行されない
            #見えている線の合成の描画
            if self.cornertype == 0:
                if self.xcog != 0 or self.ycog !=0:
                    cv2.line(result, 
                            (self.x1avg, self.y1avg), 
                            (self.x2avg, self.y2avg), 
                            (0, 255, 255), thickness=2, lineType=cv2.LINE_4 )
                    
                    # 線の角度(度)の画像への書き込み
                    cv2.putText(result,
                        text=str(self.angle),
                        org=(self.xcog+10, self.ycog+30),
                        fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale=0.6,
                        color=(0, 255, 0),
                        thickness=2,
                        lineType=cv2.LINE_4)
        
        except:
            print("No Line")
            
            
        try: #ボールが存在しない時は実行されない
            #ボール重心の描画
            if self.ballx != 0 or self.bally != 0:
                cv2.circle(result, (self.ballx,self.bally), 4, 100, 2, 4) #ボール重心座標にマーク
                cv2.putText(result,
                            text=str(self.ballx),
                            org=(self.ballx+10, self.bally+10),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
                cv2.putText(result,
                            text=str(self.bally),
                            org=(self.ballx+10, self.bally+30),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
        
        except:
            print("No Ball")
            
        try: #コーナーが存在しないときは実行されない
            #コーナーの描画
            if self.cornertype != 0:
                cv2.rectangle(result, (int(self.cornerx), int(self.cornery)), (int(self.cornerx)+p.w, int(self.cornery)+p.h), (255, 255, 0), 2)
                
            if self.cornertype == 1:
                cv2.putText(result,
                            text="Left",
                            org=(int(self.cornerx+10), int(self.cornery+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            
            elif self.cornertype == 2:
                cv2.putText(result,
                            text="Right",
                            org=(int(self.cornerx+10), int(self.cornery+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)

            elif self.cornertype == 3:
                cv2.putText(result,
                            text="Left Inner",
                            org=(int(self.cornerx+10), int(self.cornery+30)),
                            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale=0.6,
                            color=(0, 255, 0),
                            thickness=2,
                            lineType=cv2.LINE_4)
            
        except:
            print("No Corner")
            
        return result
        