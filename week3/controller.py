import sys

from week3.interpreter import Interpreter

sys.path.append(r'/home/pi/picar-x/lib')
from utils import reset_mcu
reset_mcu()
from sensing import Sensing
#from week3.sensing import Sensing
from picarx import Picarx


class controller():
    def __init__(self,px, scaling_factor=40):
        self.scaling_factor = scaling_factor
        self.px = px
        
    def controller(self,pos):
        try:
            self.px.set_dir_servo_angle(pos*self.scaling_factor)
            self.px.forward(15,pos*self.scaling_factor)
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