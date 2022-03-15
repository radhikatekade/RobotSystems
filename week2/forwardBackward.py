import sys
sys.path.append(r'/home/pi/RobotSystems/lib')

import logging
from logdecorator import log_on_start , log_on_end , log_on_error
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

logging.basicConfig(
    format='%(asctime)s %(levelname)-8s %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S')
    
from utils import reset_mcu
reset_mcu()

from picarx import Picarx
import time


@log_on_start(logging.DEBUG, "Start maneuvering...")
@log_on_end(logging.DEBUG, "Motion executed successfully")
def forwardBackward():
    try:
        px = Picarx()
        
        px.forward(30)
        time.sleep(0.5)
        px.backward(30)
        time.sleep(0.5)

        px.forward(0)
        time.sleep(1)
    finally:
        px.forward(0)

if __name__ == "__main__":

    forwardBackward()