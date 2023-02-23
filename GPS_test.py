import settings
import allsenser_class
import time,datetime
import RPi.GPIO as GPIO
from tkinter import messagebox
import sys

#インスタンス化
control_recordings=allsenser_class.recordings(settings.path1)
GPS=allsenser_class.GPS(settings.path2)
runservo=allsenser_class.servomoter()
#NineAxis=functions.NineAxis()
#Camera=functions.camera(settings.kaizo_x,settings.kaizo_y)

#ゴールの設定
GOAL = settings.goal
data=["【Destination】:","LATITUDE:",GOAL[0],"LONGITUDE:",GOAL[1]]
#control_recordings.WriteCSV(data)
print(data)
print('\n')

now_time=datetime.datetime.now()

#ラベル
data=['試行回数','現在時刻','緯度','経度','距離','方向','回転角度','回転時間']
control_recordings.WriteCSV(data)

#データ取得・距離と角度計算
Number=0
l=1
Number,lat,lon = (GPS.GpsDataReceive1PPS_1(Number,l,5))

now=(lat,lon)
print(now)

distance = GPS.GpsDataDistance(now,GOAL)
Azimuth = GPS.GpsDataAzimuth(now,GOAL)

#GPS単体プログラムではとりあえず最初の1回目は前に10秒程度動かす
"""
print('distance=',distance,'[m]')
print('azimuth=',Azimuth,'[deg]')
#print('kaiten=',Azimuth,'[deg]')
#print('direction:',direction,'\n')
"""

privious=now

direction="front"
if direction=="front":
    kaiten=0
    movetime=30

#データ記録
data=[l,now_time,lat,lon,distance,direction,kaiten,movetime]
print(data)
print('\n')
control_recordings.WriteCSV(data)

hoge=messagebox.askyesno('続行チェック','続行しますか？')
#TB6612.moveCanSat(direction,movetime)#最初は適当に前進10秒
if hoge==False:
    print('終了します')
    sys.exit()#プログラム終了
messagebox.showinfo('移動チェック','移動したらOKを押してください')

n=0#10m以内か？をカウントするための変数

while True:
    l=l+1
    Number,lat,lon=(GPS.GpsDataReceive1PPS_1(Number,l,5))
    now_time=datetime.datetime.now()
    now=(lat,lon)
    print(now)
    Azimuth=GPS.GpsDataAzimuth(now,GOAL)
    distance = GPS.GpsDataDistance(now,GOAL)
    if distance<=15:#10m以内だったらループ脱出→ではなく最終的には画像認識接続→見つからなかったら戻るを繰り返したい
        n=n+1
        time.sleep(1.1)
        for i in range(5):
            Number,lat,lon=(GPS.GpsDataReceive1PPS_1(Number,l,5))
            now=(lat,lon)
            print(now)
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

    if kaiten>20:
        direction='left'
        
    elif 0<=kaiten and kaiten<=20:
        direction='fornt'
        
    elif -20<=kaiten and kaiten<0:
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
    if direction=="front":
        kaiten=0
        movetime=30
    
    data=[l,now_time,lat,lon,distance,direction,kaiten,movetime]
    control_recordings.WriteCSV(data)
    print(data)
    print('\n')

    #TB6612.moveCanSat(direction,movetime)
    hoge=messagebox.askyesno('続行チェック','続行しますか？')
    if hoge==False:
        print('終了します')
        break
    
    messagebox.showinfo('移動チェック','移動したらOKを押してください')
    privious=now