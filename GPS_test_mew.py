from gps3 import gps3
import allsenser_class
import settings

GPSREC=allsenser_class.recordings(settings.path2)

gps_socket = gps3.GPSDSocket()
data_stream = gps3.DataStream()
gps_socket.connect()
gps_socket.watch()

data=['試行回数','緯度','経度']
GPSREC.WriteCSV(data)

k=0

for new_data in gps_socket:
  if new_data:
    k=k+1
    data_stream.unpack(new_data)
    data1=[k,data_stream.TPV['lat'],data_stream.TPV['lon']]
    print(data1)
    GPSREC.WriteCSV(data1)
    
    print('time : ', data_stream.TPV['time'])
    print('lat : ', data_stream.TPV['lat'])
    print('lon : ', data_stream.TPV['lon'])
    print('alt : ', data_stream.TPV['alt'])

    
"""
import allsenser_class
import settings
import time

#インスタンス化
GPS=allsenser_class.GPS(settings.path2)

n,lat,lon=GPS.GpsDataReceive1PPS_1(5,10)
print('緯度:',lat,'経度',lon)
print('値の取得回数:',n,'回')

try:
    while True:
        Number=Number+1
        data=GPS.GpsDataReceive(Number)
        print(data)
except KeyboardInterrupt:
    print('Done!')
"""#