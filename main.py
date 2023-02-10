import settings
import allsenser_class
import time,datetime
import pigpio

#インスタンス化
#control_recordings=allsenser_class.recordings(settings.HIKOBOSHILogfn)
#GPS=allsenser_class.GPS(settings.GpsLogfn)
RUNSERVO=allsenser_class.servomoter()
#NineAxis=functions.NineAxis()
#Camera=functions.camera(settings.kaizo_x,settings.kaizo_y)
#BME220=allsenser_class.BME220()

#BME220.setup()

#BME220.run()

RUNSERVO.front(10)
RUNSERVO.end()
