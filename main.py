import settings
import allsenser_class
import time,datetime
import pigpio
import numpy as np

#インスタンス化
#control_recordings=allsenser_class.recordings(settings.HIKOBOSHILogfn)
GPS=allsenser_class.GPS(settings.GpsLogfn)
runservo=allsenser_class.servomoter()
kubiservo=allsenser_class.kubifuri()
lidar=allsenser_class.LIDAR()
BME220=allsenser_class.BME220()
kyu=allsenser_class.BMX055()
#Camera=functions.camera(settings.kaizo_x,settings.kaizo_y)


#電源on & 着地判定
#もし10回連続で気圧の値の変化が1以下ならば9軸の落下判定へ
BME220.setup()
BME220.get_calib_param()

cnt = 0

[t,p,h] = BME220.readData()
pp = p

while True:
    [t,p,h] = BME220.readData()
    pn = p
    if abs(pn-pp)<1:
        cnt+=1
        if cnt==10:
            break

        pp = pn
        time.sleep(0.5)

#もし10回連続で9軸の値の変化が1以下ならばGPSの落下判定へ
kyu.__init__()
kyu.bmx_setup()

#もし10回連続でGPSの値の変化が1以下ならば着地したこととし分離機構を起動


#走り出し
#撮影したカメラ画像の中にパラシュート写ってた場合をTrueとする的な感じで，，，
a = kubiservo.kubifuright()
#カメラパシャリd
if #d = True:
    runservo.moveCansat("left",5)
else:
    b = kubiservo.kubifuleft()
    #カメラパシャリe
    if #e = True:
    runservo.moveCansat("right",5)

    else:
        c = kubiservo.kubifuzero()
        #カメラパシャリf
        if #f = True:
        runservo.moveCansat("right",7)
    
        else:
        runservo.moveCansat("front",5)
    
#走行
#GPS走行モード
#ゴール地点の確認
GOAL = GPS.goal
print("[Destination]:", "LATITUDE:", GOAL[0], "LONGITUDE:", GOAL[1])

now = (GPS.GpsDataRaceive())
distance = GPS.GpsDataDistance(now,GOAL)
Azimuth = GPS.GpsDataAzimuth(now,GOAL)

#現在地-ゴール直線の方位角とローバー自身のヨー角の差だけ回転→ゴールの方向を向きたい
print('distance=',distance,'[m]')
print('azimuth=',Azimuth,'[deg]')
#print('kaiten=',Azimuth,'[deg]')
#print('direction:',direction,'\n')

privious=now

#kaitentimeはちゃんと確認しないといかんです
keeptime=360/settings.kaitentime*Azimuth
runservo.moveCansat("front", 10)

n=0
#この中に首振りサーボと画像認識を組み込む形になりそうです？
while True:
    now=(GPS.GpsDataReceive())
    Azimuth=GPS.GpsDataAzimuth(now,GOAL)
    distance = GPS.GpsDataDistance(now,GOAL)
    if distance<=15:#10m以内だったらループして画像認識を試みたい
        n=n+1
        time.sleep(1)
        for i in range(5):
            now=(GPS.GpsDataReceive())
            distance = GPS.GpsDataDistance(now,GOAL)
            if distance<=5:
                n=n+1
        
        if n>=3:
            print('5m以内到達')
            break

    R_Azimuth=GPS.GpsDataAzimuth(privious,now)
    
    kaiten=R_Azimuth-Azimuth

    if kaiten>=0:
        direction='left'
    else:
        kaiten=-kaiten
        direction='right'

    print('distance=',distance,'[m]')
    print("azimuth=",Azimuth,'[deg]','RoverAzimuth',R_Azimuth,'[deg]')
    print('kaiten=',kaiten,'[deg]')
    print('direction:',direction,'\n')
    
    privious=now

    keeptime=360/settings.kaitentime*kaiten
    runservo.moveCansat(direction,keeptime)
    runservo.moveCansat("front",10)


#ゴール付近

#lidar
    while True:
        dist = lidar.VL53L5CX()
        avetate1 = lidar.avetate(dist)
        avetate2 = avetate1[:4]
        avetate3 = avetate1[4:]
        aveave1 = np.array([300,300,300,300,300,300,300,300])
        aveave2 = np.array([300,300,300,300])
        avegoal = np.array([150,150,150,150,150,150,150,150,150,])
        ave1 = avetate1>aveave1
        ave2 = avetate1<aveave1
        ave3 = avetate2<aveave2
        ave4 = avetate3<aveave2
        goal = avetate1>avegoal
        print(avetate1)
        print(ave1)

#ゴール判定
        if goal.all():
            print("目的地到着")
"""
        if ave1.all():
            forward()
            print("forward")
            
            
        elif ave2.all():
            stop(1.0)
            back(2.0)
            turn_right(1.3)
            forward()
            print("kaihikoui")
            
            
        elif ave3.all():
            stop(1.0)
            back(2.0)
            turn_left(1.3)
            forward()
            print("right")
            
            
        elif ave4.all():
            stop(1.0)
            back(2.0)
            turn_right(1.3)
            forward()
            print("left")
            
        else:
            forward()
            print("forward")
"""
        
    