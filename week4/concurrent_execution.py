#file where all the 3 functions- sensing, interpretation and control, are intregrated together
import sys
import os
import logging
import threading
import time
import concurrent.futures

_path = os.getcwd() + '/lib'
# print(_path)
sys.path.append(_path)

#from utils import reset_mcu
#reset_mcu()

from picarx_improved import Picarx
from opencv_lane import *
from bus import Bus
from threading import Event

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

_stop_requested = Event()
vid=cv2.VideoCapture(0)

#definition of sensor class
class Sensing(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def sensing_producer(self, bus, delay, kill_thread):
        while not kill_thread.is_set():
            bus.write(self.get_grayscale_data())
            time.sleep(delay)

    def get_grayscale_data(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())
        return adc_value_list

#definition  of Interpretation class
class Interpreter(object):
    def __init__(self, sensitivity, polarity):
        self.sensitivity = sensitivity
        self.polarity = polarity

    def interpret_cp(self, in_bus, out_bus, delay, kill_thread):
        while not kill_thread.is_set():
            sensor_vals = in_bus.read()
            if sensor_vals is not None:
                control_val = self.interpret_position(sensor_vals)
                out_bus.write(control_val)
            time.sleep(delay)

    def interpret_position(self, fl_list):
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

#definition of controller class
class Controller():
    def __init__(self,px, scaling_factor=40):
        self.scaling_factor = scaling_factor
        self.px = px

    def control_consumer(self, bus, delay, kill_thread):
        while not kill_thread.is_set():
            self.controller(bus.read())
            time.sleep(delay)
        
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
       angle_to_mid_deg = int((angle*180)/3.14)
       self.px.set_dir_servo_angle(angle_to_mid_deg*0.7)
       self.px.forward(20)
       time.sleep(0.1)

def sigint_handler(sig, frame):
    global _stop_requested
    _stop_requested.set()

if __name__ == "__main__":

    sensitivity = 400
    polarity = 1
    scale = 150
    runtime = 10
    speed = 10

    car = Picarx()

    command = input("1) Line following or 2) lane following: \n")

    if command == '1':
        #line following
        sensor = Sensing()
        interpreter = Interpreter(sensitivity, polarity)
        control = Controller(car, scale)

        t = time.time()
        while time.time() - t < runtime:
            readings = sensor.get_grayscale_data()
            direction = interpreter.interpret_position(readings)
            angle = control.controller(direction)
            car.forward(speed=speed)

            # setup busses
            sensor_values_bus = Bus()
            interpreter_bus = Bus()

            # delay values (seconds)
            sensor_delay = 0.1
            interpreter_delay = 0.1
            control_delay = 0.1

        with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
            eSensor = executor.submit(sensor.sensing_producer, sensor_values_bus, sensor_delay, _stop_requested)
            eInterpreter = executor.submit(interpreter.interpret_cp, sensor_values_bus, interpreter_bus, interpreter_delay, _stop_requested)
            eController = executor.submit(control.control_consumer, interpreter_bus, control_delay, _stop_requested)
            eSensor.result()
        car.stop()

    else:
        #lane following
        control = Controller(car, scale)
        t = time.time()
        #for angle in range(0,-35, -1):
        #    car.set_camera_servo1_angle(angle)
        #    time.sleep(0.01)
        while time.time() - t < runtime:
            ret, frame = vid.read()
            control.camera_control(frame)
        car.stop()
