import settings
import allsenser_class
import time,datetime
import pigpio

#インスタンス化
#control_recordings=allsenser_class.recordings(settings.HIKOBOSHILogfn)
GPS=allsenser_class.GPS(settings.GpsLogfn)
runservo=allsenser_class.servomoter()
kubiservo=allsenser_class.kubifuri()
#NineAxis=functions.NineAxis()
#Camera=functions.camera(settings.kaizo_x,settings.kaizo_y)
#BME220=allsenser_class.BME220()

#電源on & 着地判定


#走り出し
#撮影したカメラ画像の中にパラシュート写ってた場合をTrueとする的な感じで，，，
a = kubiservo.kubifuright()
#カメラパシャリd
b = kubiservo.kubifuleft()
#カメラパシャリe
c = kubiservo.kubifuzero()
#カメラパシャリf
if #d = True:
    runservo.moveCansat("left",5)
if #e = True:
    runservo.moveCansat("right",5)
if #f = True:
    runservo.moveCansat("right",7)
else:
    runservo.moveCansat("front",5)
    
#走行
#GPS走行モード
#ゴール地点の確認
GOAL = allsenser_class.goal
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

    R_Azimuth=allsenser_class.GpsDataAzimuth(privious,now)
    
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

    movetime=360/settings.kaitentime*kaiten
    functions.moveCanSat(direction,movetime)
    functions.moveCanSat("front",10)


#ゴール付近

#ゴール判定
