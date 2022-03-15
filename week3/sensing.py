import sys
import os


_path = os.getcwd() + '/lib'
# print(_path)
sys.path.append(_path)

from utils import reset_mcu
reset_mcu()

from picarx import Picarx

from adc import ADC

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

# if __name__ == "__main__":
#     import time
#     GM = Sensing(950)
#     while True:
#         print(GM.get_grayscale_data())
#         time.sleep(1)