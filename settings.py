import time,datetime



now_time = datetime.datetime.now()

#使うピンの情報や，目的地座標，カメラ設定パラメタ
#ピンは全てGPIO番号で指定すること（BCM)

#【制御ログ設定】
#ログ入れるパスの設定欄だそうです
"""
username="__________________________________________________________"
team="HIKOBOSHI2022/"

path="_______"+username+team+"LOG/"
HIKOBOSHI=path+'HIKOBOSHI'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
GpsPath=path+'GPS'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
kyutyan=path+'kyutyan'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
kiatukun=path+'kiatukun'+now_time.strftime('%Y%m%d_%H%M%S')+'csv'
"""

#【ゴール座標設定(long=経度，lat=緯度)】
#滑走路薬学部の十字標示
GPS_long=140.054014
GPS_lat=35.727382
goal=(GPS_lat,GPS_long)
#種コン会場
'''
GPS_long=hogehoge
GPS_lat=hogehoge
'''

#【機体の1回転にかかる時間】
#こちらの機体でも測らないといかんとです
kaitentime=5.75

#【カメラ撮影系設定】
kaizo_x=3280
kaizo_y=2464
framerate=15
ksize=19#中間フィルタ
approx_param=0.1#輪郭近似設定

#Auto White balance
#awbmode='off'
awbmode='auto'
#awbmode='night'
#awbmode='backlight'

#exposure(露光設定)
#exmode='off'
exmode='auto'
#exmode='sunlight'
#exmode='cloudy'
#exmode='shade'

#【画像処理系設定】赤の値域調整


#【ピン設定】
#servomoter(走行用)
SV_1=16
SV_2=17

#servomoter(首振り用)
SV_3=18