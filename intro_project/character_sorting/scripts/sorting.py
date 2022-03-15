#!/usr/bin/python3
# coding=utf8
# Date:2021/04/20
# Author:Aiden
import sys
import cv2
import math
import time
import rospy
import threading
import joblib
import numpy as np
from threading import Timer
from skimage.feature import hog

from std_msgs.msg import *
from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from warehouse.msg import Grasp
from character_sorting.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import apriltag
from armpi_fpv import bus_servo_control

# sorting
# If not declared, the length and distance units used are all m
d_tag_map = 0

tag_z_min = 0.01
tag_z_max = 0.015

d_color_map = 30

color_z_min = 0.01
color_z_max = 0.015
d_color_y = 20
color_y_adjust = 400

center_x = 340

__isRunning = False

lock = threading.RLock()

# Initialize the inverse kinematics
ik = ik_transform.ArmIK()

# Load the classifier model
clf = joblib.load("digits_cls.pkl")


range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}

# initial position


def initMove(delay=True):
    with lock:
        bus_servo_control.set_servos(
            joints_pub, 1500, ((1, 75), (2, 500), (3, 80), (4, 825), (5, 625), (6, 500)))
    if delay:
        rospy.sleep(2)

# close rgb


def turn_off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)


x_dis = 500
Y_DIS = 0
y_dis = Y_DIS
last_x_dis = x_dis
last_x_dis = y_dis
x_pid = PID.PID(P=0.01, I=0.001, D=0)  # initialize pid
y_pid = PID.PID(P=0.00001, I=0, D=0)

tag_x_dis = 500
tag_y_dis = 0
tag_x_pid = PID.PID(P=0.01, I=0.001, D=0)  # initialize pid
tag_y_pid = PID.PID(P=0.02, I=0, D=0)

stop_state = 0
move_state = 1
adjust = False
approach = False
rotation_angle = 0
start_move = False
adjust_error = False
last_X, last_Y = 0, 0
box_rotation_angle = 0
count = 0
count2 = 0
count3 = 0
count_d = 0
count_timeout = 0
count_tag_timeout = 0
count_adjust_timeout = 0
character_range = ['A','B','C','4','5','3','2','8']
# Variable reset


def reset():
    global X, Y
    global adjust
    global approach
    global move_state
    global start_move
    global current_tag
    global detect_color
    global x_dis, y_dis
    global adjust_error
    global last_X, last_Y
    global tag1, tag2, tag3
    global box_rotation_angle
    global tag_x_dis, tag_y_dis
    global last_x_dis, last_y_dis
    global rotation_angle, last_box_rotation_angle
    global count, count2, count3, count_timeout, count_adjust_timeout, count_d, count_tag_timeout

    with lock:
        X = 0
        Y = 0

        x_dis = 500
        y_dis = Y_DIS
        tag_x_dis = 500
        tag_y_dis = 0
        x_pid.clear()
        y_pid.clear()
        tag_x_pid.clear()
        tag_y_pid.clear()
        last_x_dis = x_dis
        last_y_dis = y_dis

        adjust = False
        approach = False
        start_move = False
        adjust_error = False

        move_state = 1
        turn_off_rgb()
        rotation_angle = 0
        box_rotation_angle = 0

        count = 0
        count2 = 0
        count3 = 0
        count_d = 0
        count_timeout = 0
        count_tag_timeout = 0
        count_adjust_timeout = 0

        tag1 = ['tag1', -1, -1, -1, 0]
        tag2 = ['tag2', -1, -1, -1, 0]
        tag3 = ['tag3', -1, -1, -1, 0]
        current_tag = ['tag1', 'tag2', 'tag3']
        detect_color = ('red', 'green', 'blue')

# app initialization call


def init():
    global stop_state
    global __target_data

    rospy.loginfo("object sorting Init")
    stop_state = 0
    __target_data = ((), ())
    initMove()
    reset()


y_d = 0
roll_angle = 0
gripper_rotation = 0
# Half of the diagonal length of the block
square_diagonal = 0.03*math.sin(math.pi/4)
F = 1000/240.0
# gripping


def pick(grasps, have_adjust=False):
    global roll_angle, last_x_dis
    global adjust, x_dis, y_dis, tag_x_dis, tag_y_dis, adjust_error, gripper_rotation

    position = grasps.grasp_pos.position
    rotation = grasps.grasp_pos.rotation
    approach = grasps.grasp_approach
    retreat = grasps.grasp_retreat

    # Calculate whether the target position can be reached, if not, return False
    target1 = ik.setPitchRanges((position.x + approach.x, position.y +
                                approach.y, position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges(
        (position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges(
        (position.x, position.y, position.z + grasps.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges(
        (position.x + retreat.x, position.y + retreat.y, position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False
    if target1 and target2 and target3 and target4:
        if not have_adjust:
            servo_data = target1[1]
            bus_servo_control.set_servos(joints_pub, 1800, ((
                3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
            rospy.sleep(2)
            if not __isRunning:
                return False

            # Step 3: Move to the target point
            servo_data = target2[1]
            bus_servo_control.set_servos(joints_pub, 1500, ((
                3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
            rospy.sleep(2)
            if not __isRunning:
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((
                    1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            roll_angle = target2[2]
            gripper_rotation = box_rotation_angle

            x_dis = tag_x_dis = last_x_dis = target2[1]['servo6']
            y_dis = tag_y_dis = 0

            if state == 'color':
                # Step 4: Fine adjustment of position
                if not adjust:
                    adjust = True
                    return True
            else:
                return True
        else:
            # Step 5: align
            bus_servo_control.set_servos(
                joints_pub, 500, ((2, 500 + int(F*gripper_rotation)), ))
            rospy.sleep(0.8)
            if not __isRunning:
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((
                    1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # Step 6: gripping
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, grasps.grasp_posture - 80), ))
            rospy.sleep(0.6)
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, grasps.grasp_posture), ))
            rospy.sleep(0.8)
            if not __isRunning:
                bus_servo_control.set_servos(
                    joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((
                    1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # Step 7: Lift the object
            if grasps.up != 0:
                servo_data = target3[1]
                bus_servo_control.set_servos(joints_pub, 500, ((
                    3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(0.6)
            if not __isRunning:
                bus_servo_control.set_servos(
                    joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                servo_data = target4[1]
                bus_servo_control.set_servos(joints_pub, 1000, ((
                    1, 200), (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                return False

            # Step 8: Move to the evacuation point
            servo_data = target4[1]
            if servo_data != target3[1]:
                bus_servo_control.set_servos(joints_pub, 1000, ((
                    3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5'])))
                rospy.sleep(1)
                if not __isRunning:
                    bus_servo_control.set_servos(
                        joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                    rospy.sleep(0.5)
                    return False

            # Step 9: Move to a stable point
            servo_data = target1[1]
            bus_servo_control.set_servos(
                joints_pub, 1500, ((2, 500), (3, 80), (4, 825), (5, 625)))
            rospy.sleep(1.5)
            if not __isRunning:
                bus_servo_control.set_servos(
                    joints_pub, 500, ((1, grasps.pre_grasp_posture), ))
                rospy.sleep(0.5)
                return False

            return target2[2]
    else:
        rospy.loginfo('pick failed')
        return False


def place(places):
    position = places.grasp_pos.position
    rotation = places.grasp_pos.rotation
    approach = places.grasp_approach
    retreat = places.grasp_retreat

    # Calculate whether the target position can be reached, if not, return False
    target1 = ik.setPitchRanges((position.x + approach.x, position.y +
                                approach.y, position.z + approach.z), rotation.r, -180, 0)
    target2 = ik.setPitchRanges(
        (position.x, position.y, position.z), rotation.r, -180, 0)
    target3 = ik.setPitchRanges(
        (position.x, position.y, position.z + places.up), rotation.r, -180, 0)
    target4 = ik.setPitchRanges(
        (position.x + retreat.x, position.y + retreat.y, position.z + retreat.z), rotation.r, -180, 0)

    if not __isRunning:
        return False
    if target1 and target2 and target3 and target4:
        # Step 1: The gimbal turns towards the target direction
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((1, places.pre_grasp_posture), (2, int(
            F*rotation.y)), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            return False

        # Step 2: Move to the approach point
        bus_servo_control.set_servos(joints_pub, 1500, ((3, servo_data['servo3']), (
            4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        rospy.sleep(1.6)
        if not __isRunning:
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            return False

        # Step 3: Move to the target point
        servo_data = target2[1]
        bus_servo_control.set_servos(joints_pub, 500, ((3, servo_data['servo3']), (
            4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # Step 4: Lifting
        if places.up != 0:
            servo_data = target3[1]
            bus_servo_control.set_servos(joints_pub, 800, ((3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.8)
        if not __isRunning:
            bus_servo_control.set_servos(
                joints_pub, 500, ((1, places.grasp_posture), ))
            rospy.sleep(0.5)
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # Step 5: Place
        bus_servo_control.set_servos(
            joints_pub, 100, ((1, places.pre_grasp_posture - 20), ))
        rospy.sleep(0.2)
        bus_servo_control.set_servos(
            joints_pub, 500, ((1, places.grasp_posture), ))
        rospy.sleep(1)
        if not __isRunning:
            servo_data = target4[1]
            bus_servo_control.set_servos(joints_pub, 1000, ((1, 200), (3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(1)
            return False

        # Step 6: Move to the evacuation point
        servo_data = target4[1]
        if servo_data != target3[1]:
            bus_servo_control.set_servos(joints_pub, 600, ((3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
            rospy.sleep(0.6)
            if not __isRunning:
                return False

        # Step 7: Move to a stable point
        servo_data = target1[1]
        bus_servo_control.set_servos(joints_pub, 1000, ((
            2, 500), (3, 80), (4, 825), (5, 625), (6, servo_data['servo6'])))
        rospy.sleep(1)
        if not __isRunning:
            return False

        return True
    else:
        rospy.loginfo('place failed')
        return False


# Placement coordinates x, y, z(m)
place_position = {'tag1': [0.18,  0.057,   0.01],
                  'tag2': [0.18,  0,       0.01],
                  'tag3': [0.18,  -0.057,  0.01]}

grasps = Grasp()


def move():
    global y_d
    global grasps
    global approach
    global x_adjust
    global move_state

    while True:
        if __isRunning:
            if approach:
                position = None
                approach = True

                if not adjust and move_state == 1:
                    # gripping position
                    grasps.grasp_pos.position.x = X
                    grasps.grasp_pos.position.y = Y
                    if state == 'color':
                        grasps.grasp_pos.position.z = Misc.map(
                            Y - 0.15, 0, 0.15, color_z_min, color_z_max)
                    else:
                        grasps.grasp_pos.position.z = Misc.map(
                            Y - 0.12, 0, 0.15, tag_z_min, tag_z_max)
                    # Pitch angle when gripping
                    grasps.grasp_pos.rotation.r = -175

                    # Lifting distance after gripping
                    grasps.up = 0

                    # Approaching direction and distance when gripping
                    grasps.grasp_approach.y = -0.01
                    grasps.grasp_approach.z = 0.02

                    # The direction and distance of retreat after gripping
                    grasps.grasp_retreat.z = 0.04

                    # Opening and closing of the front and rear grippers
                    grasps.grasp_posture = 450
                    grasps.pre_grasp_posture = 75
                    buzzer_pub.publish(0.1)
                    result = pick(grasps)
                    if result:
                        move_state = 2
                    else:
                        reset()
                        initMove(delay=False)
                elif not adjust and move_state == 2:
                    result = pick(grasps, have_adjust=True)
                    if not result:
                        reset()
                        initMove(delay=False)
                    move_state = 3
                elif not adjust and move_state == 3:
                    if result:
                        if state == 'color':
                            position = place_position[pick_color]
                        elif state == 'tag':
                            position = place_position[current_tag]
                        if position[0] < 0:
                            yaw = int(
                                120 - (90 + math.degrees(math.atan2(position[0], position[1]))))
                        else:
                            yaw = int(
                                120 + (90 - math.degrees(math.atan2(position[0], position[1]))))

                        places = Grasp()
                        places.grasp_pos.position.x = position[0]
                        places.grasp_pos.position.y = position[1]
                        places.grasp_pos.position.z = position[2]
                        places.grasp_pos.rotation.r = -180
                        places.grasp_pos.rotation.y = yaw

                        places.up = 0.0
                        places.grasp_approach.z = 0.02
                        places.grasp_retreat.z = 0.04

                        places.grasp_posture = 75
                        places.pre_grasp_posture = 450
                        place(places)

                    initMove(delay=False)
                    reset()
                else:
                    rospy.sleep(0.001)
            else:
                rospy.sleep(0.01)
        else:
            rospy.sleep(0.01)


th = threading.Thread(target=move)
th.setDaemon(True)
th.start()

# detect character
def getOutputCharacter(im):
    # Convert to grayscale and apply Gaussian filtering
    im_gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    im_gray = cv2.GaussianBlur(im_gray, (5, 5), 0)

    # Threshold the image
    ret, im_th = cv2.threshold(im_gray, 90, 255, cv2.THRESH_BINARY_INV)

    # Find contours in the image
    ctrs, hier = cv2.findContours(
        im_th.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Get rectangles contains each contour
    rects = [cv2.boundingRect(ctr) for ctr in ctrs]

    # For each rectangular region, calculate HOG features and predict
    # the digit using Linear SVM.
    for rect in rects:
        # Draw the rectangles
        cv2.rectangle(im, (rect[0], rect[1]), (rect[0] +
                      rect[2], rect[1] + rect[3]), (0, 255, 0), 3)

        # Make the rectangular region around the digit
        roi = im_th[rect[1]:rect[1] + rect[3], rect[0]:rect[0] + rect[2]]

        # Resize the image
        roi = cv2.resize(roi, (28, 28), interpolation=cv2.INTER_AREA)
        roi = cv2.dilate(roi, (3, 3))

        # Calculate the HOG features
        roi_hog_fd = hog(roi, orientations=9, pixels_per_cell=(
            14, 14), cells_per_block=(1, 1), visualize=False)

        nbr = clf.predict(np.array([roi_hog_fd], 'float64'))

        cv2.putText(im, str(int(nbr[0])), (rect[0], rect[1]),
                    cv2.FONT_HERSHEY_DUPLEX, 2, (0, 255, 255), 3)

        # Calculate the center 
        center_x = rect[0] + (rect[2]/2)
        center_y = rect[1] + (rect[3]/2)

        return im, nbr, center_x, center_y


size = (320, 240)

last_x = 0
last_y = 0
state = None
x_adjust = 0
pick_color = ''
# Color picking strategy


def character_sort(img, target):
    global X, Y
    global count
    global state
    global adjust
    global approach
    global x_adjust
    global pick_color
    global current_tag
    global adjust_error
    global x_dis, y_dis
    global count_timeout
    global rotation_angle
    global box_rotation_angle
    global last_x_dis, last_y_dis
    global last_x, last_y, count_d, start_move

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)

    output_image, characters, centerX, centerY = getOutputCharacter(frame_resize)

    for i in character_range:
        if i in characters:
            if abs(centerX - last_x) <= 5 and abs(centerY - last_y) <= 5 and not start_move:
                count_d += 1
                if count_d > 5:
                    count_d = 0
                    start_move = True

                    led = Led()
                    led.index = 0
                    led.rgb.r = range_rgb[i][2]
                    led.rgb.g = range_rgb[i][1]
                    led.rgb.b = range_rgb[i][0]
                    rgb_pub.publish(led)
                    led.index = 1
                    rgb_pub.publish(led)
                    rospy.sleep(0.1)

                    # Location mapping
                    if 298 + d_color_map < centerY <= 424 + d_color_map:
                        Y = Misc.map(centerY, 298 + d_color_map,
                                    424 + d_color_map, 0.12, 0.12 - 0.04)
                    elif 198 + d_color_map < centerY <= 298 + d_color_map:
                        Y = Misc.map(centerY, 198 + d_color_map,
                                    298 + d_color_map, 0.12 + 0.04, 0.12)
                    elif 114 + d_color_map < centerY <= 198 + d_color_map:
                        Y = Misc.map(centerY, 114 + d_color_map, 198 +
                                    d_color_map, 0.12 + 0.08, 0.12 + 0.04)
                    elif 50 + d_color_map < centerY <= 114 + d_color_map:
                        Y = Misc.map(centerY, 50 + d_color_map, 114 +
                                    d_color_map, 0.12 + 0.12, 0.12 + 0.08)
                    elif 0 + d_color_map < centerY <= 50 + d_color_map:
                        Y = Misc.map(centerY, 0 + d_color_map, 50 +
                                    d_color_map, 0.12 + 0.16, 0.12 + 0.12)
                    else:
                        Y = 1
            else:
                count_d = 0

            last_x = centerX
            last_y = centerY
            if (not approach or adjust) and start_move:  # adjust pid
                x_pid.SetPoint = center_x  # set
                x_pid.update(centerX)  # current
                dx = x_pid.output
                x_dis += dx  # output

                x_dis = 0 if x_dis < 0 else x_dis
                x_dis = 1000 if x_dis > 1000 else x_dis

                if adjust:
                    y_pid.SetPoint = color_y_adjust
                    start_move = True
                    centerY += abs(Misc.map(70*math.sin(math.pi/4)/2, 0, size[0], 0, img_w)*math.sin(
                        math.radians(abs(gripper_rotation) + 45))) + 65*math.sin(math.radians(abs(roll_angle)))
                    if Y < 0.12 + 0.04:
                        centerY += d_color_y
                    if 0 < centerY - color_y_adjust <= 5:
                        centerY = color_y_adjust
                    y_pid.update(centerY)

                    dy = y_pid.output
                    y_dis += dy
                    y_dis = 0.1 if y_dis > 0.1 else y_dis
                    y_dis = -0.1 if y_dis < -0.1 else y_dis
                else:
                    dy = 0
                
                if adjust and (abs(last_x_dis - x_dis) >= 2 or abs(last_y_dis - y_dis) > 0.002):
                    position = grasps.grasp_pos.position
                    rotation = grasps.grasp_pos.rotation
                    target = ik.setPitchRanges(
                        (position.x, position.y + y_dis, position.z), rotation.r, -180, 0)
                    if target:
                        servo_data = target[1]
                        bus_servo_control.set_servos(joints_pub, 100, ((3, servo_data['servo3']), (
                            4, servo_data['servo4']), (5, servo_data['servo5']), (6, int(x_dis))))
                        rospy.sleep(0.1)
                        last_x_dis = x_dis
                        last_y_dis = y_dis
                    else:
                        bus_servo_control.set_servos(
                            joints_pub, 20, ((6, int(x_dis)), ))
                else:
                    bus_servo_control.set_servos(
                        joints_pub, 20, ((6, int(x_dis)), ))

        else:
            count_timeout += 1
            if count_timeout > 20:
                adjust_error = True
                count_timeout = 0
    return output_image


d_map = 0.015
tag_map = [425, 384, 346, 310, 272, 239, 208, 177, 153, 129, 106, 86, 68, 51]
# apriltag picking strategy




def run(img):
    global adjust_error
    global count_tag_timeout
    global count_adjust_timeout
    
    if len(__target_data[0]) != 0:
        img = character_sort(img, __target_data[0])

    img_h, img_w = img.shape[:2]
    cv2.line(img, (int(img_w/2 - 10), int(img_h/2)),
             (int(img_w/2 + 10), int(img_h/2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w/2), int(img_h/2 - 10)),
             (int(img_w/2), int(img_h/2 + 10)), (0, 255, 255), 2)

    return img


def image_callback(ros_image):
    global lock
    global stop_state

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # Convert custom image messages into images
    # Convert to opencv format
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    frame = cv2_img.copy()
    frame_result = frame

    with lock:
        if __isRunning:
            frame_result = run(frame)
        else:
            if stop_state:
                stop_state = 0
                initMove(delay=False)
    # Convert to ros format
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image
    image_pub.publish(ros_image)


org_image_sub_ed = False


def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter character sorting")
    with lock:
        init()
        if not org_image_sub_ed:
            org_image_sub_ed = True
            image_sub = rospy.Subscriber(
                '/usb_cam/image_raw', Image, image_callback)

    return [True, 'enter']


heartbeat_timer = None


def exit_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("exit character sorting")
    with lock:
        __isRunning = False
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                if heartbeat_timer is not None:
                    heartbeat_timer.cancel()
                image_sub.unregister()
        except:
            pass

    return [True, 'exit']


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running character sorting")
    with lock:
        __isRunning = True


def stop_running():
    global lock
    global stop_state
    global __isRunning

    rospy.loginfo("stop running character sorting")
    with lock:
        __isRunning = False
        if (not approach and start_move) or adjust:
            stop_state = 1
        reset()


def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']


__target_data = (())


def set_target(msg):
    global lock
    global __target_data

    rospy.loginfo('%s', msg)
    with lock:
        __target_data = (msg.characters)

    return [True, 'set_target']

# heartbeat


def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy(
            '/character_sorting/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('character_sorting', log_level=rospy.DEBUG)

    # servo release
    joints_pub = rospy.Publisher(
        '/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    

    # image release
    # register result image publisher
    image_pub = rospy.Publisher(
        '/character_sorting/image_result', Image, queue_size=1)

    # app communication service
    enter_srv = rospy.Service('/character_sorting/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/character_sorting/exit', Trigger, exit_func)
    running_srv = rospy.Service(
        '/character_sorting/set_running', SetBool, set_running)
    set_target_srv = rospy.Service(
        '/character_sorting/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service(
        '/character_sorting/heartbeat', SetBool, heartbeat_srv_cb)

    # buzzer
    buzzer_pub = rospy.Publisher('/sensor/buzzer', Float32, queue_size=1)
    # rgb light
    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    config = rospy.get_param('config', {})
    if config != {}:
        d_tag_map = config['d_tag_map']

        tag_z_min = config['tag_z_min']
        tag_z_max = config['tag_z_max']

        d_color_map = config['d_color_map']

        color_z_min = config['color_z_min']
        color_z_max = config['color_z_max']
        d_color_y = config['d_color_y']
        color_y_adjust = config['color_y_adjust']

        center_x = config['center_x']

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)

        msg = SetTarget()
        msg.characters = ['A5']
        set_target(msg)

        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
