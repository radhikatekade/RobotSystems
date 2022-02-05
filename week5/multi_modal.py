import sys
import os
import logging
import threading
import time
import concurrent.futures

_path = os.getcwd() + '/lib'
# print(_path)
sys.path.append(_path)

from picarx_improved import Picarx
from opencv_lane import *
#from bus import Bus
from threading import Event
from rossros import *

try:
    from servo import Servo
    from pwm import PWM
    from pin import Pin
    from adc import ADC
    from filedb import fileDB
    import time
    from ultrasonic import Ultrasonic
    from utils import reset_mcu
    reset_mcu()
    time.sleep(0.01)
except ImportError:
    print("This computer does not appear to be a PiCar -X system (ezblock is not present). Shadowing hardware calls with substitute functions ")
    from sim_ezblock import*

_stop_requested = Event()
vid=cv2.VideoCapture(0)

#definition of sensor class
class Sensing(object):
    def __init__(self,ref = 1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    #def sensing_producer(self, bus, delay, kill_thread):
    #    while not kill_thread.is_set():
    #        bus.write(self.get_grayscale_data())
    #        time.sleep(delay)

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

    #def interpret_cp(self, in_bus, out_bus, delay, kill_thread):
    #    while not kill_thread.is_set():
    #        sensor_vals = in_bus.read()
    #        if sensor_vals is not None:
    #            control_val = self.interpret_position(sensor_vals)
    #            out_bus.write(control_val)
    #        time.sleep(delay)

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

    #def control_consumer(self, bus, delay, kill_thread):
    #    while not kill_thread.is_set():
    #        self.controller(bus.read())
    #        time.sleep(delay)
        
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


class ultrasonic_Sensor:
    def __init__(self):
        self.D0 = Pin("D0")
        self.D1 = Pin("D1")

    def get_ultrasensor_reading(self):
        distance = Ultrasonic(self.D0, self.D1).read()
        return distance

class ultrasonic_Interpreter:
    def __init__(self, thresh= 10):
        self.thresh = thresh
        self.stop = False

    def interpret_obstacle(self, distance):
        if distance < self.thresh:
            self.stop = True
        else:
            self.stop = False

        return self. stop

class ultrasonic_Controller:
    def __init__(self, speed=40):
        self.speed = speed

    def ultra_controller(self, car, stop):
        if stop:
            car.stop()
        else:
            car.forward(self.speed)

if __name__ == "__main__":

    sensitivity = 400
    polarity = 1
    scale = 150
    runtime = 10
    speed = 10
    thresh = 20

    car = Picarx()

    #line following
    sensor = Sensing()
    interpreter = Interpreter(sensitivity, polarity)
    control = Controller(car, scale)

    us_sensor = ultrasonic_Sensor()
    us_interpreter = ultrasonic_Interpreter(thresh=thresh)
    us_control = ultrasonic_Controller(speed = speed)

    # setup busses
    #grayscale
    sensor_values_bus = Bus(initial_message=[0, 0, 0],
                            name="sensor values bus")
    sensor_interpreter_bus = Bus(initial_message=0,
                          name="sensor interpreter bus")

    # ultrasonic sensor busses
    us_values_bus = Bus(initial_message=0,
                           name="ultrasonic sensor bus")
    us_interpreter_bus = Bus(initial_message=False,
                                name="ultrasonic interpreter bus")

    # delay values (seconds)
    sensor_delay = 0.1
    interpreter_delay = 0.1
    control_delay = 0.1

    # grayscale sensor threads
    greyscale_read = Producer(sensor.get_grayscale_data(),
                              output_busses=sensor_values_bus,
                              delay=0.09,
                              name="Greyscale Sensor Reading")

    greyscale_proc = ConsumerProducer(interpreter.interpret_position,
                                      input_busses=sensor_values_bus,
                                      output_busses=sensor_interpreter_bus,
                                      delay=0.1,
                                      name="Greyscale Sensor Processing")
    greyscale_cont = Consumer(control.controller,
                          input_busses=sensor_interpreter_bus,
                          delay=0.1,
                          name="Greyscale Steering Controller")

    us_read = Producer(us_sensor.get_ultrasensor_reading,
                              output_busses=us_values_bus,
                              delay=0.09,
                              name="Ultrasonic Sensor Reading")

    us_proc = ConsumerProducer(us_interpreter.interpret_obstacle,
                                      input_busses=us_values_bus,
                                      output_busses=us_interpreter_bus,
                                      delay=0.1,
                                      name="Ultrasonic  Sensor Processing")
    us_cont = Consumer(us_control.ultra_controller,
                              input_busses=us_interpreter_bus,
                              delay=0.1,
                              name="Ultrasonic Steering Controller")

    thread_list = [greyscale_read, greyscale_proc, greyscale_cont, us_read, us_proc, us_read]
    runConcurrently(thread_list)

    # t = time.time()
    # while time.time() - t < runtime:
    #     readings = sensor.get_grayscale_data()
    #     direction = interpreter.interpret_position(readings)
    #     angle = control.controller(direction)
    #     car.forward(speed=speed)

    #     # setup busses
    #     sensor_values_bus = Bus()
    #     interpreter_bus = Bus()

    #     # delay values (seconds)
    #     sensor_delay = 0.1
    #     interpreter_delay = 0.1
    #     control_delay = 0.1

    # with concurrent.futures.ThreadPoolExecutor(max_workers=3) as executor:
    #     eSensor = executor.submit(sensor.sensing_producer, sensor_values_bus, sensor_delay, _stop_requested)
    #     eInterpreter = executor.submit(interpreter.interpret_cp, sensor_values_bus, interpreter_bus, interpreter_delay, _stop_requested)
    #     eController = executor.submit(control.control_consumer, interpreter_bus, control_delay, _stop_requested)
    #     eSensor.result()
    car.stop()