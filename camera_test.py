import settings
import allsenser_class
import time,datetime
import pigpio
import numpy as np
import sys
import concurrent.futures

camera=allsenser_class.camera(settings.kaizo_x,settings.kaizo_y,settings.pcamera,settings.awbmode,settings.exmode,settings.ksize,settings.approx_param,settings.framerate,settings.hsv1_min,settings.hsv1_max,settings.hsv2_min,settings.hsv2_max)


camera.take_a_picture_cv2()