import sys
import os
import logging
import time

_path = os.getcwd() + '/lib'
# print(_path)
sys.path.append(_path)

#from utils import reset_mcu
#reset_mcu()

from picarx_improved import Picarx
from opencv_lane import *

try:
    from servo import Servo
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    import time
    from utils import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar -X system (ezblock is not present). Shadowing hardware calls with substitute functions ")
    from sim_ezblock import*

#from adc import ADC

vid=cv2.VideoCapture(0)

class Sensing(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

class Interpreter(object):
    def __init__(self, sensitivity, polarity):
        self.sensitivity = sensitivity
        self.polarity = polarity

    def get_grayscale_value(self, fl_list):
        current_pos = None
        try:
            if abs(fl_list[0] - fl_list[2]) > self.sensitivity:
                if fl_list[0] < fl_list[2]:
                    if fl_list[0] + abs((fl_list[2]-fl_list[0])/4) > fl_list[1]:
                        current_pos = .5 * self.polarity   
                    else:
                        current_pos = 1 * self.polarity
                else:
                    if fl_list[2]+abs((fl_list[2]-fl_list[0])/4) < fl_list[1]:
                        current_pos = -1 * self.polarity   
                    else:
                        current_pos = -.5 * self.polarity
            else:
                current_pos = 0
        except:
            logging.info("robot pos: {0}".format(current_pos))
        
        return current_pos

class controller():
    def __init__(self,px, scaling_factor=40):
        self.scaling_factor = scaling_factor
        self.px = px
        
    def controller(self,pos):
        try:
            self.px.set_dir_servo_angle(pos*self.scaling_factor)
            self.px.forward(40)
            time.sleep(0.1)
        finally:
            self.px.stop()
                
    def camera_control(self,frame):
       lane_lines=detect_lane(frame)
       frame_shape=frame.shape
       angle,lines = calculate_heading(lane_lines,frame_shape[1],frame_shape[0])
       self.px.set_dir_servo_angle(angle)
       self.px.forward(15,angle)
       time.sleep(0.1)

if __name__ == "__main__":

    sensitivity = 400
    polarity = 1
    scale = 50
    runtime = 10
    speed = 10

    car = Picarx()

    command = input("1) Line following or 2) lane following: \n")

    if command == 1:
        sensor = Sensing()
        interpreter = Interpreter(sensitivity, polarity)
        control = controller(car, scale)

        t = time.time()
        while time.time() - t < runtime:
            readings = sensor.get_grayscale_data()
            direction = interpreter.get_grayscale_value(readings)
            angle = control.controller(direction)
            car.forward(speed=speed)
        car.stop()

    else:
        control = controller(car, scale)
        t = time.time()
        for angle in range(0,35):
            car.set_camera_servo2_angle(angle)
            time.sleep(0.01)
        while time.time() - t < runtime:
            ret, frame = vid.read()
            control.camera_control(frame)
        car.stop()
