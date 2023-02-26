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

"""
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

        print('distance=',distance,'[m]')
        print("azimuth=",Azimuth,'[deg]','RoverAzimuth',R_Azimuth,'[deg]')
        print('kaiten=',kaiten,'[deg]')
        print('direction:',direction,'\n')

        
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
"""
#首振ってカメラで目標見つけます
#その方向へある程度適当に機体の向きを変更します
#首振ったサーボを正面に向けます
#ここでもう一回カメラパシャリしないと次からのカメラ誘導でおかしいことになりそう
#そこからは1班と同じ画像認識での誘導へと切り替えます
kubiservo.itihatizero()
exist = camera.serch()
if exist:
    #direction,movetime=camera.calc_and_decide()
    runservo.moveCansat("right",5,0)
    kubiservo.kyuzero()
else:
    kubiservo.itisango()
    if exist:
        runservo.moveCansat("right",4,0)
        kubiservo.kyuzero()
    else:
        kubiservo.kyuzero()
        if exist:
            runservo.moveCansat("front",0,3)    
        else:
            kubiservo.yongo()
            if exist:
                runservo.moveCansat("left",4,0)
                kubiservo.kyuzero()
            else:
                kubiservo.zero()
                if exist:
                    runservo.moveCansat("leftt",5,0)
                    kubiservo.kyuzero()

    while True:            
        direction,movetime=camera.calc_and_decide()
        if direction!=False:
            runservo.moveCansat(direction,movetime,10)
            check=lidar.check_goal()
            if check:
                print('ゴール到達')
                sys.exit()
        exist=camera.serch()
        if exist:

            continue
        else:
            for i in range(7):
                runservo.right(settings.kaitentime/8)
                exist=camera.serch()
                if exist:
                    break#111行目からのループ脱出
                
            if exist:
                continue#100行目からのループに戻る
            
        #どうしようもなかった場合→GPSモードに戻る
        break#99行目からのループ脱出
    break#94行目からのループ脱出

else:  
    runservo.right(settings.kaitentime/8)#無ければ機体右回転
        
Azimuth=GPS.GpsDataAzimuth(now,GOAL)
RollAve,PitchAve,YawAve,exist=NINEDOF.ObserveEulerAngles_2(3)

if exist:#9軸センサの傾きが小さければヨー角を方位角として流用
    R_Azimuth=YawAve
else:#9軸センサの傾きが大きい場合はGPSセンサによって方位角を求める
    R_Azimuth=GPS.GpsDataAzimuth(privious,now)
    
kaiten,direction=GPS.GpsDecideDirections(Azimuth,R_Azimuth)
movetime=settings.kaitentime/360*kaiten

data=[l,now_time,lat,lon,distance,R_Azimuth,direction,kaiten,movetime]
control_recordings.WriteCSV(data)
print(data)
print('\n')