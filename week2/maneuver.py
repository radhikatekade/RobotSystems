import sys
sys.path.append(r'/home/pi/RobotSystems/lib')
from utils import reset_mcu
reset_mcu()

from picarx import Picarx
import time

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
    print("import successful")