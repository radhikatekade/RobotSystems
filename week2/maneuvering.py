import sys
sys.path.append(r'/home/pi/RobotSystems/lib')
from utils import reset_mcu
reset_mcu()

from picarx import Picarx
import time

def kTurning():
    try:
        px = Picarx()

        px.forward(30)
        time.sleep(0.5)
        
        px.forward(0)
        time.sleep(1)

        for angle in range(0,-45,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
        
        px.forward(0)
        time.sleep(1)

        px.forward(30)
        time.sleep(0.7)
        
        px.forward(0)
        time.sleep(1)

        for angle in range(-45,45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
        
        px.forward(0)
        time.sleep(1)

        px.backward(30)
        time.sleep(0.7)
        
        px.forward(0)
        time.sleep(1)

        for angle in range(45,-45,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
    
        px.forward(0)
        time.sleep(1)
        
        px.forward(30)
        time.sleep(1)   

        px.forward(0)
        time.sleep(1)

        for angle in range(-45,0):
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

        for angle in range(0,45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
        
        px.forward(0)
        time.sleep(1)

        px.backward(30)
        time.sleep(0.7)
        
        px.forward(0)
        time.sleep(1)

        for angle in range(45,-45,-1):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
        
        px.forward(0)
        time.sleep(1)

        px.backward(20)
        time.sleep(0.7)
        
        px.forward(0)
        time.sleep(1)

        for angle in range(-45,45):
            px.set_dir_servo_angle(angle)
            time.sleep(0.01)   
    
        px.forward(0)
        time.sleep(1)
        
        px.forward(30)
        time.sleep(0.3)   

        px.forward(0)
        time.sleep(1)
        
        for angle in range(45,0,-1):
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


if __name__ == "__main__":
    while True:
        command = input("Select from the three maneuvering options below:\n(a) Forward and backward in straight lines or with different steering angles\n(b) Parallel-parking left and right\n(c) Three-point turning (K-turning) with initial turn to the left or right\n(d) Exit\nEnter Here : ")
        
        if command == "a":
            forwardBackward()
        elif command == "b":
            parallelParking()
        elif command == "c":
            kTurning()
        elif command == "d":
            break


