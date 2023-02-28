import settings
import allsenser_class
import time,datetime
import pigpio
import numpy as np
import sys
import concurrent.futures 

#プログラム開始時間
start_time=time.time()

#インスタンス化
control_recordings=allsenser_class.recordings(settings.HIKOBOSHI)
GPS=allsenser_class.GPS(settings.path2)
runservo=allsenser_class.servomoter()
kubiservo=allsenser_class.kubifuri()
lidar=allsenser_class.LIDAR()
BME220=allsenser_class.BME220()
kyu=allsenser_class.NineAxis(settings.path3,settings.Px,settings.Py,settings.Pz)
#パラシュート回避の方
camera_p=allsenser_class.camera(settings.kaizo_x,settings.kaizo_y,settings.pcamera,settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,settings.framerate,settings.p_hsv_min,settings.p_hsv_max,settings.p_hsv_min,settings.p_hsv_max)
#ゴールコーン見る方
camera_k=allsenser_class.camera(settings.kaizo_x,settings.kaizo_y,settings.kcamera,settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,settings.framerate,settings.hsv1_min,settings.hsv1_max,settings.hsv2_min,settings.hsv2_max)


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


#分離機構作動
kubiservo.kaihou1()
time.sleep(3)
kubiservo.kaihou2()

#GPSでデータ取っといて〜
GPS.GpsDataReceive1PPS_1(1)
privious=(lat,lon)

#走り出し
#撮影したカメラ画像の中にパラシュート写ってた場合をTrueとする的な感じで，，，
kubiservo.itihatizero()
exist = camera_p.find_a_parachute()
if exist:
    runservo.moveCansat("left",4,5)
else:
    kubiservo.itisango()
    exist = camera_p.find_a_parachute()
    #カメラパシャリe
    if exist:
        runservo.moveCansat("left",6,5)
    else:
        kubiservo.kyuzero()
        exist = camera_p.find_a_parachute()
        #カメラパシャリf
        if exist:
            runservo.moveCansat("right",7,5)    
        else:
            kubiservo.yongo()
            exist = camera_p.find_a_parachute()
            if exist:
                runservo.moveCansat("right",6,5)
            else:
                kubiservo.zero()
                exist = camera_p.find_a_parachute()
                if exist:
                    runservo.moveCansat("right",4,5)

#走行
#GPS走行モード

#【メインループ】
while True:
    l=l+1
    now_time=datetime.datetime.now().time()
    Number,lat,lon=(GPS.GpsDataReceive1PPS_1(Number,5))
    now=(lat,lon)
    control_recordings.XBEE([lat,lon])
    distance = GPS.GpsDataDistance(now,GOAL)

    if distance<=20:

        #首振ってカメラで目標見つけます
        #その方向へある程度適当に機体の向きを変更します
        #首振ったサーボを正面に向けます
        #ここでもう一回カメラパシャリしないと次からのカメラ誘導でおかしいことになりそう
        #そこからは1班と同じ画像認識での誘導へと切り替えます
        kubiservo.itihatizero()
        exist = camera_k.serch()
        if exist:
            #direction,movetime=camera.calc_and_decide()
            runservo.moveCansat("right",5,0)
            kubiservo.kyuzero()
        else:
            kubiservo.itisango()
            exist = camera_k.serch()
            if exist:
                runservo.moveCansat("right",4,0)
                kubiservo.kyuzero()
            else:
                exist = camera_k.serch()
                if exist:
                    runservo.moveCansat("front",0,3)    
                else:
                    kubiservo.yongo()
                    exist = camera_k.serch()

                    if exist:
                        runservo.moveCansat("left",4,0)
                        kubiservo.kyuzero()
                    else:
                        kubiservo.zero()
                        exist = camera_k.serch()
                        if exist:
                            runservo.moveCansat("left",5,0)
                            kubiservo.kyuzero()
    
        exist = camera_k.serch()
        if exist:
            while True:
                direction,movetime=camera_k.calc_and_decide()
                
                if direction!=False:
                    straight_time=5
                    runservo.moveCansat(direction,movetime,straight_time)
                    
                    check=lidar.check_goal()
                    if check:
                        #print('終了します')
                        sys.exit()
                        
                    #messagebox.showinfo('写真撮影','準備ができたらOKを押してください')
                    exist=camera_k.serch()
                
                else:
                    print('ゼロ割りエラー')
                    n=0
                    while True:
                        n=n+1
                        kubiservo.itihatizero()
                        exist = camera_k.serch()
                        if exist:
                            #direction,movetime=camera.calc_and_decide()
                            runservo.moveCansat("right",5,2)
                            kubiservo.kyuzero()
                            break
                        else:
                            kubiservo.itisango()
                            exist = camera_k.serch()
                            if exist:
                                runservo.moveCansat("right",4,2)
                                kubiservo.kyuzero()
                                break
                            else:
                                exist = camera_k.serch()
                                if exist:
                                    runservo.moveCansat("front",0,2)
                                    break    
                                else:
                                    kubiservo.yongo()
                                    exist = camera_k.serch()
                                    if exist:
                                        runservo.moveCansat("left",4,2)
                                        kubiservo.kyuzero()
                                        break
                                    else:
                                        kubiservo.zero()
                                        exist = camera_k.serch()
                                        if exist:
                                            runservo.moveCansat("left",5,0)
                                            kubiservo.kyuzero()
                                            break

                        exist=camera_k.serch()
                        if exist:
                            break
                        
                        if n>=2:
                            escape=True#画像認識モード脱出
                            break
                """
                if escape==True:
                    escape=False
                    break
                """   
                """
                if err>=7 or escape==True:
                    escape=False
                    print('画像認識エラー:GPSモードに戻ります')
                    dis_cnt=dis_cnt+1
                    if dis_cnt>3:#4以上になっちゃったらまた20m絞りに戻す
                        dis_cnt=0
                    err=0
                    Number,lat,lon=(GPS.GpsDataReceive1PPS_1(Number,l,3))
                    now=(lat,lon)
                    distance = GPS.GpsDataDistance(now,GOAL)
                    break
                
                #messagebox.showinfo('写真撮影','準備ができたらOKを押してください')
                exist=camera.serch()
                """
                
    Azimuth=GPS.GpsDataAzimuth(now,GOAL)
    RollAve,PitchAve,YawAve,exist=kyu.ObserveEulerAngles_1(3)

    if exist:#9軸センサの傾きが小さければヨー角を方位角として流用
        R_Azimuth=YawAve
        parts_name='GPS+9軸'
    else:#9軸センサの傾きが大きい場合はGPSセンサによって方位角を求める
        R_Azimuth=GPS.GpsDataAzimuth(privious,now)
        parts_name='GPS'
    
    kaiten,direction=GPS.GpsDecideDirections(Azimuth,R_Azimuth)
    movetime=settings.kaitentime/360*kaiten
    
    data=[l,now_time,lat,lon,distance,R_Azimuth,direction,kaiten,movetime,parts_name]
    control_recordings.WriteCSV(data)
    print(data)
    print('\n')
    
    runservo.moveCansat(direction,movetime,0)
    
    #privious=now