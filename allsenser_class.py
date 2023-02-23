from smbus2 import SMBus
import time
import datetime
import csv
import pigpio
import serial
import codecs
from smbus import SMBus
import math
from math import sin, cos, tan, atan2,acos,pi
import picamera,picamera.array
import cv2
from geopy.distance import geodesic
from tkinter import messagebox
import matplotlib.pyplot as plt
import sys
import numpy as np
import sys
import vl53l5cx_ctypes as vl53l5cx
from vl53l5cx_ctypes import STATUS_RANGE_VALID, STATUS_RANGE_VALID_LARGE_PULSE
from enum import Enum
import mpl_toolkits.mplot3d.art3d as art3d
import random
import math as ma
#from gps3 import gps3

#データ送受信系のclass
class recordings:
    def __init__(self,filename):
        self.filename=filename

    def WriteCSV(self,data):
        with codecs.open(self.filename,'a',encoding='shift-jis') as self.f:
            writer=csv.writer(self.f)
            writer.writerow(data)

    def XBEE(self,Xdata):#配列のみに対応
        self.ser = serial.Serial("/dev/ttyUSB0", 9600,timeout=.5)
        n=len(Xdata)
        for i in range(n):
            if i==(n-1):
                data=f'{Xdata[i]}\n\r'.encode('shift-jis')
                self.ser.write(data)
                break

            data=f'{Xdata[i]},'.encode('shift-jis')
            self.ser.write(data)

#気圧センサのclass
class BME220:

    dt_now=datetime.datetime.now()
    endtime=dt_now+datetime.timedelta(seconds=10)
    filename = 'test_' + dt_now.strftime('%Y%m%d_%H%M%S') + '.csv'

    """
    with open(filename,'a') as f:
            writer = csv.writer(f)
            writer.writerow(['time''temperature','pressure','humid'])
    """

    bus_number  = 1
    i2c_address = 0x76

    bus = SMBus(bus_number)

    digT = []
    digP = []
    digH = []

    t_fine = 0.0

    #気圧センサのclass

    def writeReg(self,reg_address, data):
        self.bus.write_byte_data(self.i2c_address,reg_address,data)

    def get_calib_param(self):
        calib = []
        
        for i in range (0x88,0x88+24):
            calib.append(self.bus.read_byte_data(self.i2c_address,i))
        calib.append(self.bus.read_byte_data(self.i2c_address,0xA1))
        for i in range (0xE1,0xE1+7):
            calib.append(self.bus.read_byte_data(self.i2c_address,i))

        self.digT.append((calib[1] << 8) | calib[0])
        self.digT.append((calib[3] << 8) | calib[2])
        self.digT.append((calib[5] << 8) | calib[4])
        self.digP.append((calib[7] << 8) | calib[6])
        self.digP.append((calib[9] << 8) | calib[8])
        self.digP.append((calib[11]<< 8) | calib[10])
        self.digP.append((calib[13]<< 8) | calib[12])
        self.digP.append((calib[15]<< 8) | calib[14])
        self.digP.append((calib[17]<< 8) | calib[16])
        self.digP.append((calib[19]<< 8) | calib[18])
        self.digP.append((calib[21]<< 8) | calib[20])
        self.digP.append((calib[23]<< 8) | calib[22])
        self.digH.append( calib[24] )
        self.digH.append((calib[26]<< 8) | calib[25])
        self.digH.append( calib[27] )
        self.digH.append((calib[28]<< 4) | (0x0F & calib[29]))
        self.digH.append((calib[30]<< 4) | ((calib[29] >> 4) & 0x0F))
        self.digH.append( calib[31] )
        
        for i in range(1,2):
            if self.digT[i] & 0x8000:
                self.digT[i] = (-self.digT[i] ^ 0xFFFF) + 1

        for i in range(1,8):
            if self.digP[i] & 0x8000:
                self.digP[i] = (-self.digP[i] ^ 0xFFFF) + 1

        for i in range(0,6):
            if self.digH[i] & 0x8000:
                self.digH[i] = (-self.digH[i] ^ 0xFFFF) + 1  

    def readData(self):
        data = []
        for i in range (0xF7, 0xF7+8):
            data.append(self.bus.read_byte_data(self.i2c_address,i))
        pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
        temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
        hum_raw  = (data[6] << 8)  |  data[7]
        
    # 	compensate_T(temp_raw)
    # 	compensate_P(pres_raw)
    # 	compensate_H(hum_raw)
        
        t = self.compensate_T(temp_raw)
        p = self.compensate_P(pres_raw)
        h = self.compensate_H(hum_raw)
        return [t,p,h]

    def compensate_P(self,adc_P):
        global  t_fine
        pressure = 0.0
        
        v1 = (t_fine / 2.0) - 64000.0
        v2 = (((v1 / 4.0) * (v1 / 4.0)) / 2048) * self.digP[5]
        v2 = v2 + ((v1 * self.digP[4]) * 2.0)
        v2 = (v2 / 4.0) + (self.digP[3] * 65536.0)
        v1 = (((self.digP[2] * (((v1 / 4.0) * (v1 / 4.0)) / 8192)) / 8)  + ((self.digP[1] * v1) / 2.0)) / 262144
        v1 = ((32768 + v1) * self.digP[0]) / 32768
        
        if v1 == 0:
            return 0
        pressure = ((1048576 - adc_P) - (v2 / 4096)) * 3125
        if pressure < 0x80000000:
            pressure = (pressure * 2.0) / v1
        else:
            pressure = (pressure / v1) * 2
        v1 = (self.digP[8] * (((pressure / 8.0) * (pressure / 8.0)) / 8192.0)) / 4096
        v2 = ((pressure / 4.0) * self.digP[7]) / 8192.0
        pressure = pressure + ((v1 + v2 + self.digP[6]) / 16.0)
        return pressure/100

    def compensate_T(self,adc_T):
        global t_fine
        v1 = (adc_T / 16384.0 - self.digT[0] / 1024.0) * self.digT[1]
        v2 = (adc_T / 131072.0 - self.digT[0] / 8192.0) * (adc_T / 131072.0 - self.digT[0] / 8192.0) * self.digT[2]
        t_fine = v1 + v2
        temperature = t_fine / 5120.0
        return temperature

    def compensate_H(self,adc_H):
        global t_fine
        var_h = t_fine - 76800.0
        if var_h != 0:
            var_h = (adc_H - (self.digH[3] * 64.0 + self.digH[4]/16384.0 * var_h)) * (self.digH[1] / 65536.0 * (1.0 + self.digH[5] / 67108864.0 * var_h * (1.0 + self.digH[2] / 67108864.0 * var_h)))
        else:
            return 0
        var_h = var_h * (1.0 - self.digH[0] * var_h / 524288.0)
        if var_h > 100.0:
            var_h = 100.0
        elif var_h < 0.0:
            var_h = 0.0
        return var_h


    def setup(self):
        osrs_t = 1			#Temperature oversampling x 1
        osrs_p = 1			#Pressure oversampling x 1
        osrs_h = 1			#Humidity oversampling x 1
        mode   = 3			#Normal mode
        t_sb   = 5			#Tstandby 1000ms
        filter = 0			#Filter off
        spi3w_en = 0			#3-wire SPI Disable

        ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode
        config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en
        ctrl_hum_reg  = osrs_h

        self.writeReg(0xF2,ctrl_hum_reg)
        self.writeReg(0xF4,ctrl_meas_reg)
        self.writeReg(0xF5,config_reg)

    def run(self):
        self.setup()
        self.get_calib_param()
        cnt=0

        [t,p,h] = self.readData()
        pp=p

        while True:

                    [t,p,h] = self.readData()
                    pn=p
                    print('pressure',pn,'[pa]')
                    if abs(pn-pp)<1:
                        cnt+=1
                        print('cnt=',cnt)
                        if cnt==10:
                            break
                        
                        pp=pn
                        time.sleep(0.5)
                        
        print('landomg')

#走行に使うサーボモータのclass
class servomoter:
    servo_1 = 16
    servo_2 = 17

    pi = pigpio.pi()

    # ピンを出力に設定
    pi.set_mode(servo_1, pigpio.OUTPUT)
    pi.set_mode(servo_2, pigpio.OUTPUT)

    def front(self,keeptime):
        self.pi.set_servo_pulsewidth(self.servo_1, 1000)
        self.pi.set_servo_pulsewidth(self.servo_2, 1000)
        return

    def back(self,keeptime):
        self.pi.set_servo_pulsewidth(self.servo_1, 2000)
        self.pi.set_servo_pulsewidth(self.servo_2, 2000)
        return

    def right(self,keeptime):
        self.pi.set_servo_pulsewidth(self.servo_1, 2000)
        self.pi.set_servo_pulsewidth(self.servo_2, 2000)
        
    def left(self,keeptime):
        self.pi.set_servo_pulsewidth(self.servo_1, 2000)
        self.pi.set_servo_pulsewidth(self.servo_2, 2000)

    def end(self):
        self.pi.set_servo_pulsewidth(self.servo_1, 0)
        self.pi.set_servo_pulsewidth(self.servo_2, 0)
        return

    def moveCansat(self,direction,keeptime):
        if direction=="front":
            servomoter.front(self,keeptime)
        
        if direction=="right":
            servomoter.back(self,3)
            servomoter.right(self,2)
            servomoter.front(self,keeptime)
        
        if direction=="left":
            servomoter.back(self,3)
            servomoter.left(self,2)
            servomoter.front(self,keeptime)
"""        
        if direction=="kaihi":
            servomoter.back(self,4)
            servomoter.right(self,3)
            servomoter.front(self,6)
"""

#LiDARのclass
class LIDAR:
    runservo = servomoter()
    vl53 = vl53l5cx.VL53L5CX()

    def VL53L5CX(self):
        self.vl53.set_resolution(8 * 8)

        self.vl53.enable_motion_indicator(8 * 8)
        
        self.vl53.set_integration_time_ms(50)
        
        self.vl53.enable_motion_indicator(8 * 8)
        
        self.vl53.set_motion_distance(400, 4000)
        
        self.vl53.start_ranging()
    
        data = self.vl53.get_data()

        dist = np.flipud(np.array(data.distance_mm).reshape((8, 8)))

        try:
            dist
            return dist
        except:
            print("error!")
            sys.exit()
    
    def avetate(self,dist):
        avetate = np.mean(dist,axis=0)
        return avetate

    def aveyoko(self,dist):
        aveyoko = np.mean(dist,axis=1)
        return aveyoko
    
    def check_goal(self):
        while True:
            avetate1 = self.avetate()
            avetate2 = avetate1[:4]
            avetate3 = avetate1[4:]
            aveave1 = np.array([300,300,300,300,300,300,300,300])
            aveave2 = np.array([300,300,300,300])
            avegoal = np.array([150,150,150,150,150,150,150,150])
            ave1 = avetate1>aveave1
            ave2 = avetate1<aveave1
            ave3 = avetate2<aveave2
            ave4 = avetate3<aveave2
            goal = avetate1<avegoal
            print(avetate1)
            print(ave1)

            if goal:
                print("GOAL")
                exist = True
            else:
                if ave1:
                    runservo.moveCansat("front",3)
                else:
                    if ave3:
                        runservo.moveCansat("left",2)
                    else:
                        if ave4:
                            runservo.moveCansat("right",2)
                        else:

        return True
        
#画像認識とLiDARを積むサーボモータのclass
class kubifuri:
    gp_out = 18
    servo = pigpio.pi()
    servo.set_mode(gp_out, pigpio.OUTPUT)

    def kubifuright(self):
        #右90
        self.servo.set_servo_pulsewidth(self.gp_out, 2350)
        time.sleep(0.5)
    
    def kubifuleft(self):
        #left_cm = read_distance()
        #print("left_cm= " + str(left_cm))
        self.servo.set_servo_pulsewidth(self.gp_out, 540)
        time.sleep(0.5)
        #right_cm = read_distance()
        #print("rignt_cm= " + str(right_cm))
    #ここで0
    def kubifuzero(self):
        self.servo.set_servo_pulsewidth(self.gp_out, 1400)
        time.sleep(0.5)
        return

class BMX055:
    # I2C
    def __init__(self):
        self.ACCL_ADDR = 0x19
        self.ACCL_R_ADDR = 0x02
        self.GYRO_ADDR = 0x69
        self.GYRO_R_ADDR = 0x02
        self.MAG_ADDR = 0x13
        self.MAG_R_ADDR = 0x42
        self.i2c = SMBus(1)

    def bmx_setup(self):
        # acc_data_setup : 加速度の値をセットアップ
        self.i2c.write_byte_data(self.ACCL_ADDR, 0x0F, 0x03)
        self.i2c.write_byte_data(self.ACCL_ADDR, 0x10, 0x08)
        self.i2c.write_byte_data(self.ACCL_ADDR, 0x11, 0x00)
        time.sleep(0.5)
        # gyr_data_setup : ジャイロ値をセットアップ
        self.i2c.write_byte_data(self.GYRO_ADDR, 0x0F, 0x04)
        self.i2c.write_byte_data(self.GYRO_ADDR, 0x10, 0x07)
        self.i2c.write_byte_data(self.GYRO_ADDR, 0x11, 0x00)
        time.sleep(0.5)
        # mag_data_setup : 地磁気値をセットアップ
        data = self.i2c.read_byte_data(self.MAG_ADDR, 0x4B)
        if(data == 0):
            self.i2c.write_byte_data(self.MAG_ADDR, 0x4B, 0x83)
            time.sleep(0.5)
        self.i2c.write_byte_data(self.MAG_ADDR, 0x4B, 0x01)
        self.i2c.write_byte_data(self.MAG_ADDR, 0x4C, 0x00)
        self.i2c.write_byte_data(self.MAG_ADDR, 0x4E, 0x84)
        self.i2c.write_byte_data(self.MAG_ADDR, 0x51, 0x04)
        self.i2c.write_byte_data(self.MAG_ADDR, 0x52, 0x16)
        time.sleep(0.5)

    def acc_value(self):
        data = [0, 0, 0, 0, 0, 0]
        acc_data = [0.0, 0.0, 0.0]
        try:
            for i in range(6):
                data[i] = self.i2c.read_byte_data(self.ACCL_ADDR, self.ACCL_R_ADDR + i)
            for i in range(3):
                acc_data[i] = ((data[2*i + 1] * 256) + int(data[2*i] & 0xF0)) / 16
                if acc_data[i] > 2047:
                    acc_data[i] -= 4096
                acc_data[i] *= 0.0098
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return acc_data

    def gyro_value(self):
        data = [0, 0, 0, 0, 0, 0]
        gyro_data = [0.0, 0.0, 0.0]
        try:
            for i in range(6):
                data[i] = self.i2c.read_byte_data(self.GYRO_ADDR, self.GYRO_R_ADDR + i)
            for i in range(3):
                gyro_data[i] = (data[2*i + 1] * 256) + data[2*i]
                if gyro_data[i] > 32767:
                    gyro_data[i] -= 65536
                gyro_data[i] *= 0.0038
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return gyro_data

    def mag_value(self):
        data = [0, 0, 0, 0, 0, 0, 0, 0]
        mag_data = [0.0, 0.0, 0.0]
        try:
            for i in range(8):
                data[i] = self.i2c.read_byte_data(self.MAG_ADDR, self.MAG_R_ADDR + i)
            for i in range(3):
                if i != 2:
                    mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xF8)) / 8
                    if mag_data[i] > 4095:
                        mag_data[i] -= 8192
                else:
                    mag_data[i] = ((data[2*i + 1] * 256) + (data[2*i] & 0xFE)) / 2
                    if mag_data[i] > 16383:
                        mag_data[i] -= 32768
        except IOError as e:
            print("I/O error({0}): {1}".format(e.errno, e.strerror))
        return mag_data

    def magcalibration(self,calibtime):
        print('キャリブレーションを開始します。BMX550を回転させる用意をしてください。')
        ret=messagebox.askquestion('確認','ready?')
        if ret==False:
            print('プログラムを終了します。')
            sys.exit()
        print('キャリブレーション中...Z軸まわりに回転させてください...')
        start=time.time()
        mag_x=[]
        mag_y=[]
        
        while True:
            mag=self.mag_value()
            mag_x.append(mag[0])
            mag_y.append(mag[1])
            now=time.time()
            print('\r残り時間:',f'{calibtime-(now-start):3}','秒',end='')
            if now-start>calibtime:
                break
            time.sleep(0.1)
            
        daix=np.amax(mag_x,axis=0)
        shox=np.amin(mag_x,axis=0)
        daiy=np.amax(mag_y,axis=0)
        shoy=np.amin(mag_y,axis=0)
        
        Px=(daix+shox)/2.0
        Py=(daiy+shoy)/2.0
        print('\nPx=',Px,'Py=',Py)
        
        #補正後のグラフ表示
        plt.scatter(mag_x-Px,mag_y-Py)
        plt.xlabel('x_mag')
        plt.ylabel('y_mag')
        plt.grid()
        plt.show()
        
        return Px,Py

class GPS:
    def __init__(self,Filename):
        #クラスrecordingsをインスタンス化
        self.GPS_recordings=recordings(Filename)
        #ラベル書き込み
        #data=['試行回数','現在時刻','緯度','経度']
        data=['測定回数','時刻','緯度','経度']
        self.GPS_recordings.WriteCSV(data)
        
        #self.gps_socket = gps3.GPSDSocket()
        #self.data_stream = gps3.DataStream()
    
    #この関数は使い物になりません
    def GpsDataReceive(self,Number):
        self.s = serial.Serial('/dev/serial0', 9600, timeout=10)
        
        while True:        
            data = self.s.readline().decode('shift-jis')
            
            if data.startswith("$GPGGA"):
                sData = data.split(',')
                
                #日本時間に変換
                Day = sData[1]
                Hour = int(Day[:2])
                Minute = int(Day[2:4])
                Second = int(Day[4:6])

                #Change to JP Time
                Hour += 9
                if Hour>23:
                    Hour -= 24
                    
                now_time=str(Day)+str(Hour)+str(Minute)+str(Second)
                
                #latitude=緯度
                latitude = sData[2]
                if len(latitude) == 0:
                    print('time:%d:%d:%d len(la)=%s=empty,falsed to receive data.\r\n'%(Hour,Minute,Second,len(latitude)))
                    print('time:%d:%d:%d len(la)=%s=empty,falsed to receive data.\r\n'%(Hour,Minute,Second,len(latitude)))
                    continue
                
                else:
                    latitude = float(latitude)
                    la_decimal, la_integer = math.modf(latitude/100.0)
                    gps_latitude = la_integer + la_decimal / 60.0 * 100.0
                    
                    #lod
                    lo = sData[4]
                    
                    #lo(keido) = logger_longitude
                    lo = float(lo)
                    lo_decimal, lo_integer = math.modf(lo/100.0)
                    gps_longitude = lo_integer + lo_decimal / 60.0 * 100.0
                    
                    y1 = gps_latitude#ido(tate)
                    x1 = gps_longitude#keido(yoko)
                    #print('time:%d:%d:%d, latitude:%f, longitude:%f\n' %(Hour,Minute,Second,y1,x1))
                    
                    data=[Number,y1,x1]
                    self.GPS_recordings.WriteCSV(data)
                    #print(data)
                    return y1,x1
            else:
                continue
    
    def GpsDataReceive1PPS(self,n):#1ppsの線を繋いだ場合の1発どりプログラム
        self.gps_socket.connect()
        self.gps_socket.watch()
        
        for new_data in self.gps_socket:
            if new_data:
                self.data_stream.unpack(new_data)
                if self.data_stream.TPV['lat']!='n/a':
                    n=n+1
                    lat=self.data_stream.TPV['lat']
                    lon=self.data_stream.TPV['lon']
                    data=[n,self.data_stream.TPV['time'],self.data_stream.TPV['lat'],self.data_stream.TPV['lon']]
                    self.GPS_recordings.WriteCSV(data)
                    break
            else:
                continue
                
        return n,lat,lon
    
    #指定秒数の平均値を返す関数
    def GpsDataReceive1PPS_1(self,n,N,runtime):#1ppsの線を繋いだ場合のプログラム(引数に0をとると時間無制限)
        #n:値総取得回数
        #N:平均回数
        self.gps_socket.connect()
        self.gps_socket.watch()
        start_time=time.time()
        
        latbuf=[]
        lonbuf=[]
        
        print('緯度経度取得中')
        
        if runtime==0:
            try:
                for new_data in self.gps_socket:
                    if new_data:
                        self.data_stream.unpack(new_data)
                        if self.data_stream.TPV['lat']!='n/a' and self.data_stream.TPV['time']!='n/a':
                            n=n+1
                            data=[N,n,self.data_stream.TPV['time'],self.data_stream.TPV['lat'],self.data_stream.TPV['lon']]
                            #print(data)
                            self.GPS_recordings.WriteCSV(data)
            except KeyboardInterrupt:
                LATAVE=0
                LONAVE=0
                print('測定を終了します。')
                
        else:
            for new_data in self.gps_socket:
                now_time=time.time()
                if new_data:
                    self.data_stream.unpack(new_data)
                    if self.data_stream.TPV['lat']!='n/a' and self.data_stream.TPV['time']!='n/a':
                        n=n+1
                        self.data_stream.unpack(new_data)
                        latbuf.append(self.data_stream.TPV['lat'])
                        lonbuf.append(self.data_stream.TPV['lon'])
                        data=[N,n,self.data_stream.TPV['time'],self.data_stream.TPV['lat'],self.data_stream.TPV['lon']]
                        #print(data)
                        self.GPS_recordings.WriteCSV(data)
                
                if now_time-start_time>=runtime:
                    LENGTH=len(latbuf)
                    LATSUM=sum(latbuf)
                    LONSUM=sum(lonbuf)
                    LATAVE=LATSUM/LENGTH
                    LONAVE=LONSUM/LENGTH
                    break
                
        return n,LATAVE,LONAVE
    
    #値の流しを行うバージョン(不要)
    """
    def GpsDataReceive1PPS_2(self,n,runtime):#1ppsの線を繋いだ場合のプログラム(引数に0をとると時間無制限)
        #n=通しの取得番号
        self.gps_socket.connect()
        self.gps_socket.watch()
        start_time=time.time()
        
        latbuf=[]
        lonbuf=[]
        
        k=0
        print('緯度経度取得中')
        
        if runtime==0:
            try:
                for new_data in self.gps_socket:
                    if new_data:
                        self.data_stream.unpack(new_data)
                        if self.data_stream.TPV['lat']!='n/a':
                            k=k+1
                            if k<=5:
                                continue
                            else:
                                n=n+1
                                data=[n,self.data_stream.TPV['time'],self.data_stream.TPV['lat'],self.data_stream.TPV['lon']]
                                #print(data)
                                self.GPS_recordings.WriteCSV(data)
                            
            except KeyboardInterrupt:
                LATAVE=0
                LONAVE=0
                print('測定を終了します。')
                
        else:
            for new_data in self.gps_socket:
                now_time=time.time()
                if new_data:
                    self.data_stream.unpack(new_data)
                    if self.data_stream.TPV['lat']!='n/a':
                        k=k+1
                        if k<=5:
                            continue
                        else:
                            n=n+1
                            self.data_stream.unpack(new_data)
                            latbuf.append(self.data_stream.TPV['lat'])
                            lonbuf.append(self.data_stream.TPV['lon'])
                            data=[n,self.data_stream.TPV['time'],self.data_stream.TPV['lat'],self.data_stream.TPV['lon']]
                            #print(data)
                            self.GPS_recordings.WriteCSV(data)
                
                if now_time-start_time>=runtime:
                    LENGTH=len(latbuf)
                    LATSUM=sum(latbuf)
                    LONSUM=sum(lonbuf)
                    LATAVE=LATSUM/LENGTH
                    LONAVE=LONSUM/LENGTH
                    break
                
        return n,LATAVE,LONAVE
    """
    
    def GpsDataDistance(self,shiten,shuten):#中身はまんまgeopyだが関数名統一したかったので導入
        distance = geodesic(shiten,shuten).m
        return distance

    def GpsDataAzimuth(self,shiten, shuten):
        # Radian角に修正#r  = 6378.137e3
        #(緯度y、経度x)
        y1=shiten[0]
        x1=shiten[1]
        y2=shuten[0]
        x2=shuten[1]
        #print("x1=%lf,y1=%lf,x2=%lf,y2=%lf" % (x1,y1,x2,y2))
        _x1, _y1, _x2, _y2 = x1*pi/180, y1*pi/180, x2*pi/180, y2*pi/180
        Δx = _x2 - _x1
        _y = sin(Δx)
        _x = cos(_y1) * tan(_y2) - sin(_y1) * cos(Δx)
        
        psi = atan2(_y, _x) * 180 / pi
        if psi < 0:
            azimuth = 360 + atan2(_y, _x) * 180 / pi

        else:
            azimuth = atan2(_y, _x) * 180 / pi
        
        return azimuth
       
    def GpsDecideDirections(self,Azimuth,R_Azimuth):
        kaiten=R_Azimuth-Azimuth

        if kaiten>20:
            direction='left'
            if kaiten>180:
                direction='right'
                kaiten=360-kaiten
            
        elif -20<=kaiten and kaiten<=20:
            kaiten=0
            direction='fornt'
            
        elif kaiten<20:
            kaiten=-kaiten
            direction='right'
            if kaiten>180:
                direction='left'
                kaiten=360-kaiten
        
        return kaiten,direction
    
#カメラのクラス
class camera:
    def __init__(self,kaizo_x,kaizo_y,path,awb,ex,ksize,approx,framerate,hsv1min,hsv1max,hsv2min,hsv2max):
        #設定パラメータ
        self.path=path
 
        self.kaizo_x=kaizo_x
        self.kaizo_y=kaizo_y

        self.awbmode=awb
        self.exmode=ex
        self.ksize=ksize
        self.framerate=framerate
        
        self.hsv1_min=hsv1min
        self.hsv1_max=hsv1max
        self.hsv2_min=hsv2min
        self.hsv2_max=hsv2max

        self.approx_param=approx
        self.path=path
 
        self.kaizo_x=kaizo_x
        self.kaizo_y=kaizo_y

        self.awbmode=awb
        self.exmode=ex
        self.ksize=ksize
        self.framerate=framerate
        
        self.hsv1_min=hsv1min
        self.hsv1_max=hsv1max
        self.hsv2_min=hsv2min
        self.hsv2_max=hsv2max

        self.approx_param=approx
        
        self.contours_triangle=[]
    
    """
    def filename(self,n):
        now = datetime.datetime.now()
        if n==1:
            pic_filename=self.path+now.strftime('%Y%m%d_%H%M%S')+'_1image'+'.jpg'
            return pic_filename
        elif n==2:
            mask_filename=self.path+now.strftime('%Y%m%d_%H%M%S')+'_2mask'+'.jpg'
            return mask_filename
        elif n==3:
            contours_filename=self.path+now.strftime('%Y%m%d_%H%M%S')+'_3contours'+'.jpg'
            return contours_filename
        elif n==4:
            plotcenter_filename=self.path+now.strftime('%Y%m%d_%H%M%S')+'_4centers'+'.jpg'
            return plotcenter_filename
        elif n==5:
            direction_filename=self.path+now.strftime('%Y%m%d_%H%M%S')+'_5directions'+'.jpg'
            return direction_filename
    """
    
    def take_a_picture(self,videoport):
        with picamera.PiCamera() as camera:
            with picamera.array.PiRGBArray(camera) as cap:
                #カメラ設定
                camera.resolution=(self.kaizo_x,self.kaizo_y)
                camera.framerate=self.framerate
                
                camera.awb_mode=self.awbmode
                camera.exposure_mode=self.exmode

                camera.capture(cap,'bgr',use_video_port=videoport)
                img=cap.array

                #fn=self.filename(self,1)
                #cv2.imwrite(fn,img)
                
                cv2.imshow('pic',img)
                cv2.waitKey(0)
                
                #リセット
                cap.seek(0)
                cap.truncate()
                cv2.destroyAllWindows()
                
                return img
    
    #2班用(OpenCVによる画像撮影プログラム)
    #ホワイトバランス等の調整が必要かも
    def take_a_pictuire_cv2(self):
        cap=cv2.VideoCapture(0)
        ret,img=cap.read()
        cv2.imshow('image',img)
        cv2.waitKey(0)
        cap.release()
        #cv2.destroyAllWindows()
        return img

    #赤検知→マスク画像生成
    def detect_red(self,img):
        #BGR→HSV変換
        imghsv=cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask1 = cv2.inRange(imghsv, self.hsv1_min,self.hsv1_max)
        mask2 = cv2.inRange(imghsv, self.hsv2_min,self.hsv2_max)

        mask = mask1 + mask2
        
        #メジアンフィルタでノイズ除去
        mask = cv2.medianBlur(mask,self.ksize)
        cv2.imshow('画像',mask)
        cv2.waitKey(0)
        #fn=self.filename(self,2)
        #cv2.imwrite(fn,mask)
        """
        now = datetime.datetime.now()
        fn="/home/pi/Nupps2022_programs/CanSat/Logs/"+now.strftime('%Y%m%d_%H%M%S')+'hsv'+'.jpg'
        fn1="/home/pi/Nupps2022_programs/CanSat/Logs/"+now.strftime('%Y%m%d_%H%M%S')+'mask1'+'.jpg'
        fn2="/home/pi/Nupps2022_programs/CanSat/Logs/"+now.strftime('%Y%m%d_%H%M%S')+'mask2'+'.jpg'
        cv2.imwrite(fn,imghsv)
        cv2.imwrite(fn1,mask1)
        cv2.imwrite(fn2,mask2)
        """

        return mask

    #三角形輪郭確認
    def check_triangle_contours(self,img,mask):
        #輪郭抽出
        contours1,hierarchy1 = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        angles=[]
        n=0
        #すべての輪郭を近似
        for i,cnt in enumerate(contours1):
            approx1=cv2.approxPolyDP(cnt,self.approx_param*cv2.arcLength(cnt,True),True)
            if len(approx1)==3:
                area=cv2.contourArea(approx1)
                if area>=3e3:
                    #self.contours_triangle=approx1
                    area_triangle=area
                    print(f"contour {i}: before: {len(cnt)}, after: {len(approx1)},area:{area_triangle}")
                    img = cv2.drawContours(img,[approx1],0,(0,255,0),2)
                    cv2.imshow('img',img)
                    cv2.waitKey(0)

                    a,b,c=approx1
                    vec=[a-b,b-c,c-a]
                    length_vec=[np.linalg.norm(vec[0]), np.linalg.norm(vec[1]),np.linalg.norm(vec[2])]
                    #print(length_vec)
                    inner_product = [np.inner(vec[0], vec[1]),np.inner(vec[1],vec[2]),np.inner(vec[2],vec[0])]
                    length_seki=[length_vec[0]*length_vec[1],length_vec[1]*length_vec[2],length_vec[2]*length_vec[0]]

                    degree=[]
                    for n in range(3):
                        cos = inner_product[n] / length_seki[n]
                        rad = math.acos(cos)
                        degree.append(180-np.rad2deg(rad))
                    
                    #print(degree)
                    degree.sort()
                    print(degree)
                    A=degree[0]/degree[2]
                    B=degree[1]/degree[2]
                    print('A:',A,'B:',B)
                    
                    if B>=0.6 and B<=1:
                        if B-A>0.3:
                            n=n+1
                            self.contours_triangle=approx1
                            angles.append(B)
            
        if self.contours_triangles==[[]]:
            print("E1_trianglecontours couldn't be detected")
            exist=False
            
        elif angles==[]:
            exist=False
            
        else:
            exist=True
            MAX_B=max(angles)
            index=angles.index(MAX_B)
            print(index)
            img = cv2.drawContours(img,[self.contours_triangle],0,(255,255,255),5)
            self.img_contours = cv2.putText(img,"triangle",(200,100),\
                                            cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0))
            #fn=self.filename(self,3)
            #cv2.imwrite(fn,self.img_contours)
            cv2.imshow('img',img)
            cv2.waitKey(0)
            print("【success!】:triangle contour has been detected")
        return exist
    
    def calculate_center(self):
        #三角形重心を求める
        M=cv2.moments(self.contours_triangle)

        if M["m00"]==0:
            print("E2_can't find center")
            exist=False
        else:
            exist=True
            self.cx=M["m10"]/M["m00"]
            self.cy=M["m01"]/M["m00"]
            #画像中心に点を打つ
            img=cv2.drawMarker(self.img_contours,(int(self.kaizo_x/2),int(self.kaizo_y/2)),(225,0,0),\
                markerType=cv2.MARKER_TILTED_CROSS,markerSize=50,thickness=5)
            #三角形重心に点を打つ
            self.img_plot=cv2.drawMarker(img,(int(self.cx),int(self.cy)),(255,0,255),markerType=cv2.MARKER_TILTED_CROSS,markerSize=50,thickness=5)
            #fn=Cam.filename(self,4)
            #cv2.imwrite(fn,img)
            cv2.imshow('img',self.img_plot)
            cv2.waitKey(0)
        return exist

    def decide_directions(self):
        pixel=self.kaizo_x/2-self.cx
        if pixel<-100:
            direction="right"
            movetime=0.2#ゆくゆくはピクセル数に応じた秒数にしたい
        elif pixel>=100:
            direction="left"
            movetime=0.2
        elif -100<pixel and pixel<100:
            direction="front"
            movetime=10
        img=cv2.putText(self.img_plot,direction,(200,200),cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,255))
        #fn=self.filename(self,5)
        #cv2.imwrite(fn,img)
        
        return direction,movetime
    
    def serch(self):
        #img=self.take_a_picture(False)
        img=self.take_a_pictuire_cv2()
        mask=self.detect_red(img)
        exist=self.check_triangle_contours(img,mask)
        
        return exist
    
    def calc_and_decide(self):
        exist=self.calculate_center()
        if exist:
            direction,movetime=self.decide_directions()
        else:
            direction=False
            movetime=False
        
        return direction,movetime
    """        
    #パラシュートがあったらあるよ！って認識してくれるやつ
    def check_triangle_contours_red(self,img,mask):
        #輪郭抽出
        contours1,hierarchy1 = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        angles=[]
        n=0
        if contours1:
            exist = True

        else:
            exist = False
            
        return exist
    """
    #赤色輪郭ならなんでも確認（パラシュート探索用）
    def check_parachute_contours(self,img,mask):
        #輪郭抽出
        contours1,hierarchy1 = cv2.findContours(mask, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        n=0
        #すべての輪郭を近似
        for i,cnt in enumerate(contours1):
            approx1=cv2.approxPolyDP(cnt,self.approx_param*cv2.arcLength(cnt,True),True)
            #print(i,len(approx1))
            area=cv2.contourArea(approx1)
            
            if area>=3e2:
                n=n+1
                print(n)
                img = cv2.drawContours(img,[approx1],0,(0,255,0),2)
                cv2.imshow('img',img)
                cv2.waitKey(0)
        
        if n>=1:
            exist=True

        else:
            exist=False
        
            """
            if len(approx1)==3:
                area=cv2.contourArea(approx1)
                if area>=3e3:
                    #self.contours_triangle=approx1
                    area_triangle=area
                    print(f"contour {i}: before: {len(cnt)}, after: {len(approx1)},area:{area_triangle}")
                    img = cv2.drawContours(img,[approx1],0,(0,255,0),2)
                    cv2.imshow('img',img)
                    cv2.waitKey(0)
                    a,b,c=approx1
                    vec=[a-b,b-c,c-a]
                    length_vec=[np.linalg.norm(vec[0]), np.linalg.norm(vec[1]),np.linalg.norm(vec[2])]
                    #print(length_vec)
                    inner_product = [np.inner(vec[0], vec[1]),np.inner(vec[1],vec[2]),np.inner(vec[2],vec[0])]
                    length_seki=[length_vec[0]*length_vec[1],length_vec[1]*length_vec[2],length_vec[2]*length_vec[0]]
                    degree=[]
                    for n in range(3):
                        cos = inner_product[n] / length_seki[n]
                        rad = math.acos(cos)
                        degree.append(180-np.rad2deg(rad))
                    
                    #print(degree)
                    degree.sort()
                    print(degree)
                    A=degree[0]/degree[2]
                    B=degree[1]/degree[2]
                    print('A:',A,'B:',B)
                    
                    if B>=0.6 and B<=1:
                        if B-A>0.3:
                            n=n+1
                            self.contours_triangle=approx1
                            angles.append(B)
            
        if self.contours_triangles==[[]]:
            print("E1_trianglecontours couldn't be detected")
            exist=False
            
        elif angles==[]:
            exist=False
            
        else:
            exist=True
            MAX_B=max(angles)
            index=angles.index(MAX_B)
            print(index)
            img = cv2.drawContours(img,[self.contours_triangle],0,(255,255,255),5)
            self.img_contours = cv2.putText(img,"triangle",(200,100),\
                                            cv2.FONT_HERSHEY_SIMPLEX,2,(0,255,0))
            #fn=self.filename(self,3)
            #cv2.imwrite(fn,self.img_contours)
            cv2.imshow('img',img)
            cv2.waitKey(0)
            print("【success!】:triangle contour has been detected")
        """
        return exist
    
    def find_a_parachute(self):
        #img=self.take_a_picture(False)
        img=self.take_a_pictuire_cv2()
        mask=self.detect_red(img)
        exist=self.check_parachute_contours(img,mask)
        
        return exist
            
class hikouki:
    kyuziku=BMX055()

    Hz=50
    T=1/Hz

    kyuziku.bmx_setup()

    theta_x=theta_y=theta_z=0
    x=np.array([[theta_x],[theta_y],[theta_z]])
    print(x)

    sigma2=np.array([[1,0,0],[0,1,0],[0,0,1]])

    Q=np.array([[1,0,0],[0,1,0],[0,0,1]])/100

    R=np.array([[1,0,0],[0,1,0],[0,0,1]])/10

    time.sleep(T)

    while True:
        class EulerOrder(Enum):
            XYZ=0
            XZY=1
            YXZ=2
            YZX=3
            ZXY=4
            ZYX=5

        def plane(self,offset):
            x = [1,-1,-1, 1,  -1,-1, 1]
            y = [0, 1,-1, 0,   0, 0, 0]
            z = [0, 0, 0, 0,-0.5, 0, 0]

            mx = list(map(lambda a: a + offset[0], x))
            my = list(map(lambda b: b + offset[1], y))
            mz = list(map(lambda c: c + offset[2], z))

            return mx, my, mz

        def axis(self,offset):
            x = [1, 0, 0]
            y = [0, 1, 0]
            z = [0, 0, 1]

            mx = list(map(lambda a: a + offset[0], x))
            my = list(map(lambda b: b + offset[1], y))
            mz = list(map(lambda c: c + offset[2], z))

            return mx, my, mz

        def EulerAngles(self,p, th, order):
            if order == self.EulerOrder.XYZ:
                #XYZ
                x = ((ma.cos(th[1])*ma.cos(th[2]))*p[0]) + ((-ma.cos(th[1])*ma.sin(th[2]))*p[1]) + (ma.sin(th[1])*p[2])
                y = ((ma.sin(th[0])*ma.sin(th[1])*ma.cos(th[2])+ma.cos(th[0])*ma.sin(th[2]))*p[0]) + ((-ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((-ma.sin(th[0])*ma.cos(th[1]))*p[2])
                z = ((-ma.cos(th[0])*ma.sin(th[1])*ma.cos(th[2])+ma.sin(th[0])*ma.sin(th[2]))*p[0]) + ((ma.cos(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.sin(th[0])*ma.cos(th[2]))*p[1]) + ((ma.cos(th[0])*ma.cos(th[1]))*p[2])
                return x,y,z
            elif order == self.EulerOrder.XZY:
                #XZY
                x = ((ma.cos(th[1])*ma.cos(th[2]))*p[0]) + (-ma.sin(th[2])*p[1]) + ((ma.sin(th[1])*ma.cos(th[2]))*p[2])
                y = ((ma.cos(th[0])*ma.cos(th[1])*ma.sin(th[2])+ma.sin(th[0])*ma.sin(th[1]))*p[0]) + ((ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((ma.cos(th[0])*ma.sin(th[1])*ma.sin(th[2])-ma.sin(th[0])*ma.cos(th[1]))*p[2])
                z = ((ma.sin(th[0])*ma.cos(th[1])*ma.sin(th[2])-ma.cos(th[0])*ma.sin(th[1]))*p[0]) + ((ma.sin(th[0])*ma.cos(th[2]))*p[1]) + ((ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[0])*ma.cos(th[1]))*p[2])
                return x,y,z
            elif order == self.EulerOrder.YXZ:
                #YXZ
                x = ((ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[1])*ma.cos(th[2]))*p[0]) + ((ma.sin(th[0])*ma.sin(th[1])*ma.cos(th[2])-ma.cos(th[1])*ma.sin(th[2]))*p[1]) + ((ma.cos(th[0])*ma.sin(th[1]))*p[2])
                y = ((ma.cos(th[0])*ma.sin(th[2]))*p[0]) + ((ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((-ma.sin(th[0]))*p[2])
                z = ((ma.sin(th[0])*ma.cos(th[1])*ma.sin(th[2])-ma.sin(th[1])*ma.cos(th[2]))*p[0]) + ((ma.sin(th[0])*ma.cos(th[1])*ma.cos(th[2])+ma.sin(th[1])*ma.sin(th[2]))*p[1]) + ((ma.cos(th[0])*ma.cos(th[1]))*p[2])
                return x,y,z
            elif order == self.EulerOrder.YZX:
                #YZX
                x = ((ma.cos(th[1])*ma.cos(th[2]))*p[0]) + ((-ma.cos(th[0])*ma.cos(th[1])*ma.sin(th[2])+ma.sin(th[0])*ma.sin(th[1]))*p[1]) + ((ma.sin(th[0])*ma.cos(th[1])*ma.sin(th[2])+ma.cos(th[0])*ma.sin(th[1]))*p[2])
                y = ((ma.sin(th[2]))*p[0]) + ((ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((-ma.sin(th[0])*ma.cos(th[2]))*p[2])
                z = ((-ma.sin(th[1])*ma.cos(th[2]))*p[0]) + ((ma.cos(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.sin(th[0])*ma.cos(th[1]))*p[1]) + ((-ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[0])*ma.cos(th[1]))*p[2])
                return x,y,z
            elif order == self.EulerOrder.XYZ.ZXY:
                #ZXY
                x = ((-ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[1])*ma.cos(th[2]))*p[0]) + ((-ma.cos(th[0])*ma.sin(th[2]))*p[1]) + ((ma.sin(th[0])*ma.cos(th[1])*ma.sin(th[2])+ma.sin(th[1])*ma.cos(th[2]))*p[2])
                y = ((ma.sin(th[0])*ma.sin(th[1])*ma.cos(th[2])+ma.cos(th[1])*ma.sin(th[2]))*p[0]) + ((ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((-ma.sin(th[0])*ma.cos(th[1])*ma.cos(th[2])+ma.sin(th[1])*ma.sin(th[2]))*p[2])
                z = ((-ma.cos(th[0])*ma.sin(th[1]))*p[0]) + ((ma.sin(th[0]))*p[1]) + ((ma.cos(th[0])*ma.cos(th[1]))*p[2])
                return x,y,z
            elif order == self.EulerOrder.ZYX:
                #ZYX
                x = ((ma.cos(th[1])*ma.cos(th[2]))*p[0]) + ((ma.sin(th[0])*ma.sin(th[1])*ma.cos(th[2])-ma.cos(th[0])*ma.sin(th[2]))*p[1]) + ((ma.cos(th[0])*ma.sin(th[1])*ma.cos(th[2])+ma.sin(th[0])*ma.sin(th[2]))*p[2])
                y = ((ma.cos(th[1])*ma.sin(th[2]))*p[0]) + ((ma.sin(th[0])*ma.sin(th[1])*ma.sin(th[2])+ma.cos(th[0])*ma.cos(th[2]))*p[1]) + ((ma.cos(th[0])*ma.sin(th[1])*ma.sin(th[2])-ma.sin(th[0])*ma.cos(th[2]))*p[2])
                z = ((-ma.sin(th[1]))*p[0]) + ((ma.sin(th[0])*ma.cos(th[1]))*p[1]) + ((ma.cos(th[0])*ma.cos(th[1]))*p[2])
            
                return x,y,z

        def PaperAirplaneEuler(self,angle, order):
            fig = plt.figure(figsize=(8,6))
            ax = fig.add_subplot(111, projection='3d')
            plt.cla()

            th9 = [0.0]*3

            offset = [0,0,0]

            x,y,z = self.plane(offset)  
            axx,axy,axz = self.axis(offset)

            th9[0] = angle[0] * np.pi / 180.0
            th9[1] = angle[1] * np.pi / 180.0
            th9[2] = angle[2] * np.pi / 180.0
            
            x9,y9,z9 = [0]*7,[0]*7,[0]*7
            for i in range(7):
                x9[i],y9[i],z9[i] = self.EulerAngles(self,[x[i],y[i],z[i]], th9, order)
            
        
            axx2,axy2,axz2 = [0.0]*3, [0.0]*3, [0.0]*3
            speed = 0.0

            angle2 = [0.0, 0.0, 0.0]
            th2 = [0.0, 0.0, 0.0]

            if order == self.EulerOrder.XYZ:   od = [0,1,2]
            elif order == self.EulerOrder.XZY: od = [0,2,1]
            elif order == self.EulerOrder.YXZ: od = [1,0,2]
            elif order == self.EulerOrder.YZX: od = [1,2,0]
            elif order == self.EulerOrder.ZXY: od = [2,0,1]
            elif order == self.EulerOrder.ZYX: od = [2,1,0]
            OrderNo = 0
            ra = od[OrderNo]

            plt.cla()
            
            if angle[ra] >= angle2[ra]:
                    angle2[ra] += speed
                    if angle2[ra] >= angle[ra]:
                        angle2[ra] = angle[ra]

            th2[0] = angle2[0] * ma.pi / 180.0
            th2[1] = angle2[1] * ma.pi / 180.0
            th2[2] = angle2[2] * ma.pi / 180.0

            for i in range(3):
                    axx2[i],axy2[i],axz2[i] = self.EulerAngles(self,[axx[i],axy[i],axz[i]], th2, order)   # 回転軸のベクトル用

            poly1 = list(zip(x9[:4],y9[:4],z9[:4]))
            ax.add_collection3d(art3d.Poly3DCollection([poly1], color='red', linewidths=0.3, alpha=0.02))
            poly2 = list(zip(x9[3:7],y9[3:7],z9[3:7]))
            ax.add_collection3d(art3d.Poly3DCollection([poly2], color='brown', linewidths=0.3, alpha=0.02))

            ax.set_xlabel("x");     ax.set_ylabel("y");     ax.set_zlabel("z")
            ax.set_xlim(-2,2);      ax.set_ylim(-2,2);      ax.set_zlim(-2,2)
            ax.set_box_aspect((1,1,1))
            
            ax.text(-1,-1,-2.3, 'Target Euler Angle: '+format(angle[0],'.1f')+', '+format(angle[1],'.1f')+', '+format(angle[2],'.1f'), fontsize=9)

            plt.show()

        omega = kyuziku.gyro_value()
        a = kyuziku.acc_value()
        m = kyuziku.mag_value()


        theta_hat_x=x[0]+(omega[0]+omega[1]*math.tan(x[1])*math.sin(x[0])+omega[2]*math.tan(x[1])*math.cos(x[0]))*T
        theta_hat_y=x[1]+(omega[1]*math.cos(x[0])-omega[2]*math.sin(x[0]))*T
        theta_hat_z=x[2]+(omega[1]*math.sin(x[0])/math.cos(x[1])+omega[2]*math.cos(x[0])/math.cos(x[1]))*T

        x_hat=np.array([theta_hat_x,theta_hat_y,theta_hat_z])

        tiltheta_x=math.atan(-a[1]/-a[2])
        tiltheta_y=math.atan(a[0]/math.sqrt(a[1]**2+a[2]**2))
        tiltheta_z=math.atan((m[0]*math.cos(tiltheta_y)+m[1]*math.sin(tiltheta_y)*math.sin(tiltheta_x)+\
                    m[2]*math.sin(tiltheta_y)*math.cos(tiltheta_x))/(m[1]*math.cos(tiltheta_x)-m[2]*math.sin(tiltheta_x)))

        x_tilda=np.array([[tiltheta_x],[tiltheta_y],[tiltheta_z]])

        A11=1+(omega[0]+omega[1]*math.tan(x[1])*math.cos(x[0])-omega[2]*math.tan(x[1])*math.sin(x[0]))*T
        A12=(omega[1]*math.sin(x[0])/math.cos(x[1])**2+omega[2]*math.cos(x[0])/math.cos(x[1])**2)*T
        A13=0
        A21=(-omega[1]*math.sin(x[0])-omega[2]*math.cos(x[0]))*T
        A22=1
        A23=0
        A31=(omega[1]*math.cos(x[0])/math.cos(x[1])-omega[2]*math.sin(x[0])/math.cos(x[1]))*T
        A32=(omega[1]*math.sin(x[0])*math.sin(x[1])/math.cos(x[1])**2+omega[2]*math.cos(x[0])*math.sin(x[1])/math.cos(x[1])**2)*T
        A33=1

        A=np.array([[A11,A12,A13],[A21,A22,A23],[A31,A32,A33]])

        C=np.identity(3)

        sigma2Q=np.dot(np.dot(A,sigma2),np.transpose(A))+Q

        sigma2R=np.dot(np.dot(C,sigma2Q),np.transpose(C))+R

        K=np.dot(np.dot(sigma2Q,np.transpose(C)),np.linalg.inv(sigma2R))

        x=x_hat+np.dot(K,(x_tilda-np.dot(C,x_hat)))
        print(x*180/math.pi)
        print('')

        sigma2=np.dot(np.identity(3)-np.dot(K,C),sigma2Q)

        time.sleep(T)
    
        angle = [-tiltheta_x*180/math.pi, tiltheta_y*180/math.pi, -tiltheta_z*180/math.pi]
        order = EulerOrder.ZXY
        PaperAirplaneEuler(angle, order)

