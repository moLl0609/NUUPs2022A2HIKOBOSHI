import settings
import allsenser_class
import time,datetime
import pigpio
import numpy as np
import sys
import concurrent.futures


#インスタンス化
control_recordings=allsenser_class.recordings(settings.HIKOBOSHILogfn)
GPS=allsenser_class.GPS(settings.Gps)
runservo=allsenser_class.servomoter()
kubiservo=allsenser_class.kubifuri()
lidar=allsenser_class.LIDAR()
BME220=allsenser_class.BME220()
kyu=allsenser_class.BMX055()
camera=allsenser_class.camera(settings.kaizo_x,settings.kaizo_y,settings.path,settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,settings.framerate,settings.hsv1_min,settings.hsv1_max,settings.hsv2_min,settings.hsv2_max)


#【諸設定・セットアップなど】
#カウンター初期化
Number=0#GPSの総取得回数
l=0#制御試行回数

#ゴール設定
GOAL = settings.goal
data=['【Destination】:','LATITUDE:',GOAL[0],'LONGITUDE:',GOAL[1]]
print(data)
print('\n')
#control_recordings.WriteCSV(data)#これログに書くとgoogle mapへの読み込みが不便になる

#制御ログファイルにラベル記入(着地判定なども込みで制御ログを作った方がいいかも)
data=['試行回数','現在時刻','緯度','経度','距離','方位角','方向','回転角度','回転時間']
control_recordings.WriteCSV(data)

#センサセットアップ
BME220.setup()
BME220.get_calib_param()
kyu.bmx_setup()


#電源on & 着地判定
#もし10回連続で気圧の値の変化が1以下ならば9軸の落下判定へ
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
kyu.bmx_setup()

#もし10回連続でGPSの値の変化が1以下ならば着地したこととし分離機構を起動


#走り出し
#撮影したカメラ画像の中にパラシュート写ってた場合をTrueとする的な感じで，，，
kubiservo.itihatizero()
exist = camera.find_a_parachute()
if exist:
    runservo.moveCansat("left",4,0)
else:
    kubiservo.itisango()
    #カメラパシャリe
    if exist:
        runservo.moveCansat("left",6,0)
    else:
        kubiservo.kyuzero()
        #カメラパシャリf
        if exist:
            runservo.moveCansat("right",7,0)    
        else:
            kubiservo.yongo()
            if exist:
                runservo.moveCansat("right",6,0)
            else:
                kubiservo.zero()
                if exist:
                    runservo.moveCansat("right",4,0)

#走行
#GPS走行モード

#【メインループ】
while True:
    l=l+1
    now_time=datetime.datetime.now().time()
    Number,lat,lon=(GPS.GpsDataReceive1PPS_1(Number,l,5))
    now=(lat,lon)
    distance = GPS.GpsDataDistance(now,GOAL)

    
    #ゴールからの距離が15m以内ならカメラでコーンを探索
    if distance<=15:
        for i in range(7):#22.5度ずつ右回転して1周する
            exist=camera.serch()
            if exist: 
                #画像認識ループ開始
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