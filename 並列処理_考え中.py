import settings
import allsenser_class
import time,datetime
import pigpio
import numpy as np
import sys
import concurrent.futures
import threading

#インスタンス化
control_recordings=allsenser_class.recordings(settings.HIKOBOSHILogfn)
GPS=allsenser_class.GPS(settings.Gps)
runservo=allsenser_class.servomoter()
kubiservo=allsenser_class.kubifuri()
lidar=allsenser_class.LIDAR()
BME220=allsenser_class.BME220()
kyu=allsenser_class.BMX055()
camera=allsenser_class.camera(settings.kaizo_x,settings.kaizo_y,settings.path,settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,settings.framerate,settings.hsv1_min,settings.hsv1_max,settings.hsv2_min,settings.hsv2_max)


def gps_navigation():
    # GPSでの誘導を行う処理
    while not goal_detected:
        # ゴールが検知されるまでGPSでの誘導を継続する
        Number=Number+1
        now=(GPS.GpsDataReceive(Number))
        now_time=time.time()
        Azimuth=GPS.GpsDataAzimuth(now,GOAL)
        distance = GPS.GpsDataDistance(now,GOAL)
        if distance<=3:#10m以内だったらループ脱出
            n=n+1
            time.sleep(1)
            for i in range(5):
                Number=Number+1
                now=(GPS.GpsDataReceive(Number))
                distance = GPS.GpsDataDistance(now,GOAL)
                if distance<=10:
                    n=n+1
            
            if n>=3:
                print('10m以内到達')
                data=['10m以内に到達']
                control_recordings.WriteCSV(data)
                break

        R_Azimuth=GPS.GpsDataAzimuth(privious,now)
        kaiten=R_Azimuth-Azimuth

        if kaiten>10:
            direction='left'
            
        elif 0<=kaiten and kaiten<=10:
            direction='fornt'
            
        elif -10<=kaiten and kaiten<0:
            kaiten=-kaiten
            direction='front'
            
        else:
            kaiten=-kaiten
            direction='right'
        """
        print('distance=',distance,'[m]')
        print("azimuth=",Azimuth,'[deg]','RoverAzimuth',R_Azimuth,'[deg]')
        print('kaiten=',kaiten,'[deg]')
        print('direction:',direction,'\n')
        """
        
        movetime=settings.kaitentime/360*kaiten
        #print('movetime',movetime)
        
        data=[Number,kaiten,direction,movetime]
        control_recordings.WriteCSV(data)
        print(data)
        pass

def image_recognition():
    # 画像認識を行う処理
    while not goal_detected:
        # ゴールが検知されるまで画像認識を継続する
        kubiservo.kubifuright()
        exist = camera.serch()
        if exist:
            stop_gps_navigation()
            runservo.moveCansat("right",5)
            while True:
                    direction,movetime=camera.calc_and_decide()
                    if direction!=False:
                        runservo.moveCansat(direction,movetime,10)
                        check=lidar.check_goal()
                        if check:
                            print('ゴール到達')
                            sys.exit()
        else:
            kubiservo.kubifuleft()
            if exist:
                stop_gps_navigation()
                runservo.moveCansat("left",5)
                while True:
                    direction,movetime=camera.calc_and_decide()
                    if direction!=False:
                        runservo.moveCansat(direction,movetime,10)
                        check=lidar.check_goal()
                        if check:
                            print('ゴール到達')
                            sys.exit()

            else:
                kubiservo.kubifuzero()
                if exist:
                    stop_gps_navigation()
                    while True:
                        direction,movetime=camera.calc_and_decide()
                        if direction!=False:
                            runservo.moveCansat(direction,movetime,10)
                            check=lidar.check_goal()
                            if check:
                                print('ゴール到達')
                                sys.exit()
            
        pass
    
    # ゴールが検知されたらGPS誘導を停止し、画像認識での誘導に切り替える
    

def stop_gps_navigation():
    # GPS誘導を停止する処理
    pass

# ゴールが検知されたかどうかを示すフラグ
goal_detected = False

# GPSでの誘導を別スレッドで行う
gps_thread = threading.Thread(target=gps_navigation)
gps_thread.start()

# 画像認識を別スレッドで行う
image_thread = threading.Thread(target=image_recognition)
image_thread.start()

# 両方の処理が終了するまで待機する
gps_thread.join()
image_thread.join()
