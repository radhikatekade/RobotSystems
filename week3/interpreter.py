import sys
import os
import logging

_path = os.getcwd() + '/lib'
# print(_path)
sys.path.append(_path)

from utils import reset_mcu
reset_mcu()
from sensing import Sensing
from picarx import Picarx

class Interpreter(object):
    def __init__(self, sensitivity, polarity):
        self.sensitivity = sensitivity
        self.polarity = polarity

    # def get_line_status(self,fl_list):
    #     if self.polarity: 
    #         if fl_list[0] > self.ref and fl_list[1] > self.ref and fl_list[2] > self.ref:
    #             return 'stop'

    #         elif fl_list[1] <= self.ref:
    #             return 'forward'

    #         elif fl_list[0] <= self.ref:
    #             return 'right'

    #         elif fl_list[2] <= self.ref:
    #             return 'left'
    #     else:
    #         if fl_list[0] > self.ref and fl_list[1] > self.ref and fl_list[2] > self.ref:
    #             return 'stop'

    #         elif fl_list[1] <= self.ref:
    #             return 'forward'

    #         elif fl_list[0] <= self.ref:
    #             return 'right'

    #         elif fl_list[2] <= self.ref:
    #             return 'left'

        def get_grayscale_value(self, fl_list):
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