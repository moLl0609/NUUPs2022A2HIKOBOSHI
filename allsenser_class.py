from smbus2 import SMBus
import time
import datetime
import csv
import pigpio
import serial
import codecs

#データ送受信系のclass
class recordings:
    def WriteCSV(self,data):
        with codecs.open(self.filename,'a',encoding='shift-jis') as self.f:
            writer=csv.writer(self.f)
            writer.writerow(data)

    def XBEE(self,Xdata):#配列のみに対応
        n=len(Xdata)
        for i in range(n):
            if i==(n-1):
                data=f'{Xdata[i]}\n\r'.encode('shift-jis')
                self.ser.write(data)
                break

            data=f'{Xdata[i]},'.encode('shift-jis')
            self.ser.write(data)

    def __init__(self,filename):
        self.filename=filename
        self.ser = serial.Serial("/dev/ttyUSB0", 9600,timeout=.5)


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
        
    """
    def write_csv(self,t,p,h):
        dt_now = datetime.datetime.now()
        with open(filename, 'a') as f:
            f.write(dt_now.strftime('%Y/%m/%d %H:%M:%S') + "," + str(t) + "," + str(p) + "," + str(h) + "\n")
    """       

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
            servomoter.back(self,keeptime)
            servomoter.right(self,keeptime)
            servomoter.front(self,keeptime)
        
        if direction=="left":
            servomoter.back(self,keeptime)
            servomoter.left(self,keeptime)
            servomoter.front(self,keeptime)

#LiDARのclass
#class LIDAR:

#画像認識とLiDARを積むサーボモータのclass
class kubihuri:
    gp_out = 18
    servo = pigpio.pi()
    servo.set_mode(gp_out, pigpio.OUTPUT)

    def kubifuri(self):
        #右90
        self.servo.set_servo_pulsewidth(self.gp_out, 2350)
        time.sleep(0.5)
        #left_cm = read_distance()
        #print("left_cm= " + str(left_cm))
        self.servo.set_servo_pulsewidth(self.gp_out, 540)
        time.sleep(0.5)
        #right_cm = read_distance()
        #print("rignt_cm= " + str(right_cm))
        #ここで0
        self.servo.set_servo_pulsewidth(self.gp_out, 1400)
        time.sleep(0.5)
        return

molmolmol


