import time,datetime
import numpy as np


now_time = datetime.datetime.now()

#使うピンの情報や，目的地座標，カメラ設定パラメタ
#ピンは全てGPIO番号で指定すること（BCM)

#【制御ログ設定】
#ログ入れるパスの設定欄だそうです
#【制御ログ設定】
username="pi/"
#team="LOG"

#path="/home/"+moll+team+"LOG/"
HIKOBOSHI='/home/moll/HIKOBOSHI'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
path1='/home/moll/LOG/CONTROL_LOG_'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'
path2='/home/moll/GPS_LOG/GPS_LOG_'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'
path3='/home/moll/kyu_LOG'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'
pcamera='/home/moll/pcamera_LOG'
kcamera='/home/moll/kcamera_LOG'
#GPS=path+'GPS'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'

#NINEDOF=path+'NINEDOF'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'
#BME280=path+'BME280'+now_time.strftime('%Y%m%d_%H%M%S')+'.csv'

#着地判定系設定
timer=300#単位は秒
p_param_inside=0.1#放出とみなす気圧差（2月18日室内測定の結果を反映）
p_param_outside=0.5#適当
bmeHz=1#取得周波数

"""
username="__________________________________________________________"
team="HIKOBOSHI2022/"

path="_______"+username+team+"LOG/"
HIKOBOSHI=path+'HIKOBOSHI'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
Gps=path+'GPS'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
kyutyan=path+'kyutyan'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
kiatukun=path+'kiatukun'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
"""
#path='/home/moll/Nupps/camera/'

#【ゴール座標設定(long=経度，lat=緯度)】
#滑走路薬学部の十字標示
GPS_long=140.054014
GPS_lat=35.727382
goal=(GPS_lat,GPS_long)

'''
#種コン会場
GPS_long=hogehoge
GPS_lat=hogehoge
goal=(GPS_lat,GPS_long)
'''

#【機体の1回転にかかる時間】
#計らないとまずい
kaitentime=5.75

#【カメラ撮影系設定】
kaizo_x= 1280
kaizo_y = 720
framerate=15
ksize=19#中間フィルタ
approx_param=0.1#輪郭近似設定

#Auto White balance
#awbmode='off'
awbmode='auto'
#awbmode='sunlight'
#awbmode='cloudy'
#awbmode='shade'

#exposure(露光設定)
#exmode='off'
exmode='auto'
#exmode='night'
#exmode='backlight'

#【画像処理系設定】赤の値域調整
# 赤色領域のマスク（255：赤色、0：赤色以外）  
#mask1
#hsv1_min=np.array([0, 75, 75])#[0,100,100]
#hsv1_max=np.array([5, 215, 230])#[5,200,200]
#mask2
#PiCamera用(2/24晴れ)
#hsv2_min=np.array([160, 65, 65])
#hsv2_max=np.array([179, 215, 230])#不要？
#hsv2_max=np.array([179, 255, 255])

#OpenCV用(2/24晴れ)
#mask1
hsv1_min=np.array([0, 95, 95])#[0,100,100]
hsv1_max=np.array([5, 150, 200])#[5,200,200]
#mask2
hsv2_min=np.array([160, 70, 80])
hsv2_max=np.array([179, 215, 230])

#去年のやつ
#hsv2_min=np.array([170,64,0])
#hsv2_max=np.array([179,255,255])

#パラシュート
p_hsv_min=np.array([0,200,130])
p_hsv_max=np.array([20,255,255])

#9軸センサ地磁気キャリブレーションパラメタ
Px=-44.9
Py=-13.9
Pz=-141.8

#【ピン設定】
#servomoter(走行用)
SV_1=16
SV_2=17

#servomoter(首振り用)
SV_3=18