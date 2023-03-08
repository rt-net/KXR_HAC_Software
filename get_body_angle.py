import adafruit_bno055
import board
import time
i2c = board.I2C()

sensor = adafruit_bno055.BNO055_I2C(i2c) #センサー値を読み込む
time.sleep(1.0)
yawinit = sensor.euler[0] #yaw初期角度設定(毎回電源投入時にキャリブレーションされるので今は使ってない)
pitchinit = sensor.euler[1] #pitch初期角度設定(毎回電源投入時にキャリブレーションされるので今は使ってない)
rollinit = sensor.euler[2] #roll初期角度設定(毎回電源投入時にキャリブレーションされるので今は使ってない)

def main():
    try:
        yaw = sensor.euler[0]# - yawinit
        if yaw > 180:
            yaw = (yaw-180)-180
        pitch = sensor.euler[1] #-pitchinit
        roll = sensor.euler[2] #-rollinit
        #print(yaw, pitch, roll)
        BodyAngle = (yaw, pitch, roll)
        return BodyAngle
    except:
        print("sensor error")
        BodyAngle = (0, 0, 0)
        return BodyAngle
    #print(sensor.gravity)