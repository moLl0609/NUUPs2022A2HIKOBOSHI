# -*- coding: utf-8 -*-
"""
Created on Mon Feb 27 17:32:47 2023

@author: taiyo
"""

import settings
import allsenser_class
import time
import sys
import numpy as np

#インスタンス化
#BME280=functions.BME280('hoge')
#US=functions.UltraSonic(settings.US_Trig,settings.US_Echo,22.7,0.1)
#CAMERA=functions.camera(settings.kaizo_x,settings.kaizo_y,'hogehoge',\
        #settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,\
        #settings.framerate,settings.hsv1_min,settings.hsv1_max,settings.hsv2_min,settings.hsv2_max)
NINEDOF=allsenser_class.NineAxis(settings.path3,settings.Px, settings.Py, settings.Pz)


#BME280 着地判定の閾値設定探索用
"""
BME280.get_calib_param()
BME280.setup()
runtime=15
p=[]
HENSA_MAX=[]
HENSA_MIN=[]
start_time=time.time()
for i in range(5):
    while True:
        now_time=time.time()
        if now_time-start_time>=runtime:
            break
        
        nt,np,nh=BME280.readData()
        print(nt,np,nh)
        p.append(np)
        #print(np)
        time.sleep(1)
            
    AVE=sum(p)/len(p)
    MAX=max(p)
    MIN=min(p)
    HENSA_MAX.append(MAX-AVE)
    HENSA_MIN.append(MIN-AVE)
    
    start_time=time.time()
    #continue
HENSA_MAX_AVE=sum(HENSA_MAX)/len(HENSA_MAX)
HENSA_MIN_AVE=sum(HENSA_MIN)/len(HENSA_MIN)
print(HENSA_MAX_AVE)
print(HENSA_MIN_AVE)
"""

#t,p,h=BMW280.readData
#ここでインスタンス化して温度入れようかな

#超音波センサ
"""
exist=US.check_goal()
print(exist)
"""

#カメラ
"""
exist=CAMERA.serch()
if exist:
    direction,movetime=CAMERA.calc_and_decide()
    print(direction)
else:
    print('error!')

exist=CAMERA.find_a_parachute()
#print(exist)
if exist:
    print('パラシュート発見')
else:
    print('なにもありません')

#img=CAMERA.take_a_pictuire_cv2()

#9軸
"""
"""
#【衝撃検知】
NINEDOF.bmx_setup()
ap=NINEDOF.acc_value()
start_time=time.time()
while True:
   an=NINEDOF.acc_value()
   print(an[0]-ap[0],an[1]-ap[1],an[2]-ap[2])
   now_time=time.time()
   keika_time=now_time-start_time
   
   if abs(an[0]-ap[0])>35:#値は適当
       break
   if abs(an[1]-ap[1])>35:#値は適当
       break
   if abs(an[2]-ap[2])>35:#値は適当
       break
   if keika_time>10:
       break
   time.sleep(1/15)
   ap=an
print('衝撃検知')
"""
#Px,Py=NINEDOF.MagCalib_Gensei(10)
Px,Py,Pz=NINEDOF.MagCalib_3D_Gensei(5)
#Px=25.0
#Py=-17.0
#Px=30.3
#Py=0.6
#Px=26
#Py=7
#Pz=-138.5
#Px=34.0
#Py=-20.0
#Pz=-140.0
#家キャリブレーション2月19日
#Px= 9.5
#Py= 19.0
#Pz= -145.5
#Px= -1.0 
#Py= 5.5
#Pz= -144.0
#323室キャリブレーション2月20日
#Px=-0.5
#Py=18.0
#Pz=-161.0
#Px=35.0
#Py=11.0
#Px=-7.0
#Py=35.5
#Pz=-136.0
#Px=-15.0
#Py=18.5
#Pz=-139.0
#Px=-15.0
#Py=18.5
#Pz=-139
#Px=-10.6
#Py=5.7
#Pz=-136.2


#NINEDOF.Not_observ_Euler(Px,Py,Pz)

#print('Px=',Px,'Py=',Py)
#NINEDOF.Not_observ_Euler(Px,Py,Pz)
#print(YAWAVE)
