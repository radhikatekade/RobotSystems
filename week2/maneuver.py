import time
from picarx_improved import Picarx
from utils import reset_mcu
import sys
sys.path.append(r'/home/pi/RobotSystems/lib')
reset_mcu()


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


def parallelParking():
    try:
        px = Picarx()

        px.forward(30)
        time.sleep(0.5)

        px.forward(0)
        time.sleep(1)

        for angle in range(0, 45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.backward(30)
        time.sleep(0.7)

        px.forward(0)
        time.sleep(1)

        for angle in range(45, -45, -1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.backward(20)
        time.sleep(0.7)

        px.forward(0)
        time.sleep(1)

        for angle in range(-45, 45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.forward(30)
        time.sleep(0.3)

        px.forward(0)
        time.sleep(1)

        for angle in range(45, 0, -1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.forward(10)
        time.sleep(0.2)

        px.forward(0)
        time.sleep(1)
    finally:
        px.forward(0)

def kTurning():
    try:
        px = Picarx()

        px.forward(30)
        time.sleep(0.5)

        px.forward(0)
        time.sleep(1)

        for angle in range(0, -45, -1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.forward(30)
        time.sleep(0.7)

        px.forward(0)
        time.sleep(1)

        for angle in range(-45, 45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.backward(30)
        time.sleep(0.7)

        px.forward(0)
        time.sleep(1)

        for angle in range(45, -45, -1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.forward(30)
        time.sleep(1)

        px.forward(0)
        time.sleep(1)

        for angle in range(-45, 0):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)

        px.forward(0)
        time.sleep(1)

        px.forward(30)
        time.sleep(0.7)

        px.forward(0)
        time.sleep(1)

    finally:
        px.forward(0)


if __name__ == "__main__":
#    print("import successful")
    while True:
        command = input("1- Forward and backward in straight lines or with different steering angles\n2- Parallel-parking left and right\n3 Three-point turning (K-turning) with initial turn to the left or right\n4 Exit\nEnter: ")
        
        if command == "1":
            forwardBackward()
        elif command == "2":
            parallelParking()
        elif command == "3":
            kTurning()
        elif command == "4":
            break

