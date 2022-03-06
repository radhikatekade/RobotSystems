
import sys
import cv2
import time
from ArmIK.Transform import getAngle
from ArmIK.ArmMoveIK import ArmIK
import HiwonderSDK.Board as Board
from ArmIK.Transform import *
from ArmIK.ArmMoveIK import *
from week7.perception import Perception

class Motion:

    def __init__(self, task):

        self.AK = ArmIK()
        self.servo1 = 500
        self.gripperAngle_closed = 500
        self.base_z = 1.5
        self.block_height = 2.5
        self.num_blocks_stacked = 0
        self.task = task

    def set_rgb(color):
        if color == "red":
            Board.RGB.setPixelColor(0, Board.PixelColor(255, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(255, 0, 0))
            Board.RGB.show()
        elif color == "green":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 255, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 255, 0))
            Board.RGB.show()

        elif color == "blue":
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 255))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 255))
            Board.RGB.show()
        else:
            Board.RGB.setPixelColor(0, Board.PixelColor(0, 0, 0))
            Board.RGB.setPixelColor(1, Board.PixelColor(0, 0, 0))
            Board.RGB.show()

    # Move to default position
    def initMove(self):
        Board.setBusServoPulse(1, self.servo1 - 250, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(1, self.servo1 - 50, 300)
        time.sleep(0.5)
        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((0, 10, 10), -30, -30, -90, 1500)
        time.sleep(1.5)

    def set_task_parameters(self):
        if self.task == 'sorting':
            self.coordinates = {
                'red': (-15 + 0.5, 12 - 0.5, 1.5),
                'green': (-15 + 0.5, 6 - 0.5, 1.5),
                'blue': (-15 + 0.5, 0 - 0.5, 1.5),
            }
        elif self.task == 'stacking':
            self.coordinates = {
                'red': (-15 + 1, -7 - 0.5, 1.5),
                'green': (-15 + 1, -7 - 0.5, 1.5),
                'blue': (-15 + 1, -7 - 0.5, 1.5),
            }
        else:
            raise IOError("Task not supported")

    def sorting(self, block_x, block_y, block_rotation, block_color):

        if not block_color in ['red', 'green', 'blue']:
            raise Exception("Bad color :(")

        place_x, place_y, place_z = self.coordinates[block_color]

        if not self.check_move_reachable(block_x, block_y):
            return False

        self.pick_block(block_x, block_y, self.base_z, block_rotation)

        self.place(place_x, place_y, place_z)

        self.initMove()

    def palletize(self, block_x, block_y, block_rotation):

        place_x, place_y, place_z = self.coordinates["pallet"]

        place_z += self.block_height * self.num_blocks_stacked

        if not self.check_move_reachable(block_x, block_y):
            return False

        self.pick_block(block_x, block_y, self.base_z, block_rotation)

        self.place(place_x, place_y, place_z)

        self.initMove()
        self.num_blocks_stacked += 1
        self.num_blocks_stacked %= 3

    def rotate_endEffector_by_angle(self, targetAngle):

        Board.setBusServoPulse(2, targetAngle, 500)

    def move_endEffector_to_target(self, target_world_X, target_world_Y):

        status = self.AK.setPitchRangeMoving(
            (target_world_X, target_world_Y, 7), -90, -90, 0)

        if status != False:

            self.last_target_world_X = target_world_X
            self.last_target_world_Y = target_world_Y

        return status

    def closeGripper(self):

        Board.setBusServoPulse(1, self.gripperAngle_closed, 500)

    def openGripper(self):
        # open gripper
        Board.setBusServoPulse(1, self.gripperAngle_closed - 280, 500)


    def pick_block(self, x, y, z, rotation):

        servo2_angle = getAngle(x, y, rotation)
        Board.setBusServoPulse(1, self.servo1 - 280, 500)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        self.AK.setPitchRangeMoving((x, y, z), -90, -90, 0, 1000)
        time.sleep(1.5)

        self.closeGripper()
        time.sleep(0.8)

        Board.setBusServoPulse(2, 500, 500)
        self.AK.setPitchRangeMoving((x, y, 12), -90, -90, 0, 1000)
        time.sleep(1)

    def place(self, x, y, z, rotation=-90):

        result = self.AK.setPitchRangeMoving((x, y, 12), -90, -90, 0)
        time.sleep(result[2]/1000)

        servo2_angle = getAngle(x, y, -90)
        Board.setBusServoPulse(2, servo2_angle, 500)
        time.sleep(0.5)

        self.AK.setPitchRangeMoving((x, y, z + 3), -90, -90, 0, 500)
        time.sleep(0.5)

        self.AK.setPitchRangeMoving((x, y, z), -90, -90, 0, 1000)
        time.sleep(0.8)

        self.openGripper()
        time.sleep(0.8)

        self.AK.setPitchRangeMoving((x, y, 12), -90, -90, 0, 800)
        time.sleep(0.8)

    # initial move, returns false if unreachable, otherwise makes initial motion
    def check_move_reachable(self, x, y, z=7):

        result = self.AK.setPitchRangeMoving((x, y, z), -90, -90, 0)
        if result == False:
            return False
        else:
            # wait for motion to execute
            time.sleep(result[2]/1000)

            return True

    def stop_motion(self):
        self.stop = True
        self.initMove()
        print("Stopping motion")
        sys.exit()

    

if __name__ == "__main__":
    print(
        "Requires 1 argument for action to take: pick from [sort, palletize]\n")
    task = input("Enter 1 or 0:")

    if int(task):
        _task = 'sorting'
    else:
        _task = 'stacking'

    __target_color = ('red')
    # Create perception object
    size = (640, 480)
    perception = Perception(__target_color, size, range_rgb)
    cap = cv2.VideoCapture(-1)
    time.sleep(3)

    # Create motion object
    motion = Motion(task=_task)
    motion.set_task_parameters()
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            # Find color and make bounding boxes on frames, get max contour bbox
            Frame, valid, areaMaxContour = perception.FindColor(frame)
            world_x, world_y, rotation_angle, color = perception.get_block_location(
                contour=areaMaxContour, display_img=Frame)

            cv2.imshow('Frame', Frame)

            key = cv2.waitKey(1)
            if key == 27:
                break
            if world_x is not None:
                if int(task):
                    motion.sorting(world_x, world_y, rotation_angle, color)
                else:
                    motion.palletize(world_x, world_y, rotation_angle)

    cv2.destroyAllWindows()
