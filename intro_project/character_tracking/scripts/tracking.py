#!/usr/bin/python3
# coding=utf8
# Date:2021/02/27
# Author:Aiden
import sys
import cv2
import math

from matplotlib.pyplot import get
import rospy
import numpy as np
from threading import RLock, Timer

from std_srvs.srv import *
from sensor_msgs.msg import Image

from sensor.msg import Led
from character_tracking.srv import *
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from kinematics import ik_transform

from armpi_fpv import PID
from armpi_fpv import Misc
from armpi_fpv import bus_servo_control

from digit_recognition import getOutputDigit

# color tracking

ik = ik_transform.ArmIK()
lock = RLock()

size = (320, 240)
start_move = True
__target_character = ''
__isRunning = False
org_image_sub_ed = False

x_dis = 500
y_dis = 0.167
Z_DIS = 0.2
z_dis = Z_DIS
x_pid = PID.PID(P=0.06, I=0.005, D=0)  # initialize pid
y_pid = PID.PID(P=0.00001, I=0, D=0)
z_pid = PID.PID(P=0.00003, I=0, D=0)

range_rgb = {
    'A': (0, 0, 255),
    'B': (255, 0, 0),
    '2': (0, 255, 0),
    '3': (0, 0, 0),
    '5': (255, 255, 255),
}

# Find the contour with the largest area
# The parameter is a list of contours to be compared


def getAreaMaxContour(contours):
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  # Traverse all contours
        # Calculate the contour area
        contour_area_temp = math.fabs(cv2.contourArea(c))
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 10:  # Only when the area is greater than 300, the contour of the largest area is effective to filter interference
                area_max_contour = c

    return area_max_contour, contour_area_max  # Return the largest contour

# Initial position


def initMove(delay=True):
    with lock:
        target = ik.setPitchRanges((0, y_dis, Z_DIS), -90, -92, -88)
        if target:
            servo_data = target[1]
            bus_servo_control.set_servos(joints_pub, 1500, ((1, 200), (2, 500), (3, servo_data['servo3']), (
                4, servo_data['servo4']), (5, servo_data['servo5']), (6, servo_data['servo6'])))
    if delay:
        rospy.sleep(2)


def turn_off_rgb():
    led = Led()
    led.index = 0
    led.rgb.r = 0
    led.rgb.g = 0
    led.rgb.b = 0
    rgb_pub.publish(led)
    led.index = 1
    rgb_pub.publish(led)

# Variable reset


def reset():
    global x_dis, y_dis, z_dis
    global __target_character

    with lock:
        x_dis = 500
        y_dis = 0.167
        z_dis = Z_DIS
        x_pid.clear()
        y_pid.clear()
        z_pid.clear()
        turn_off_rgb()
        __target_character = ''

# app initialization call


def init():

    print("object tracking Init")
    initMove()
    reset()


def run(img):
    global start_move
    global x_dis, y_dis, z_dis

    img_copy = img.copy()
    img_h, img_w = img.shape[:2]

    cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)),
             (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
    cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)),
             (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    output_image, character, center_x, center_y = getOutputDigit(frame_resize)
    # frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)  # Convert image to LAB space

    for input_character in __target_character:
        if input_character == int(character[0]):

            center_x = int(Misc.map(center_x, 0, size[0], 0, img_w))
            center_y = int(Misc.map(center_y, 0, size[1], 0, img_h))
            radius = int(Misc.map(radius, 0, size[0], 0, img_w))
            # if radius > 100:
            #     return img
            # cv2.circle(img, (int(center_x), int(center_y)), int(radius), range_rgb[__target_character], 2)
            if start_move:
                x_pid.SetPoint = img_w / 2.0  # set
                x_pid.update(center_x)  # current
                dx = x_pid.output
                x_dis += int(dx)  # output

                x_dis = 200 if x_dis < 200 else x_dis
                x_dis = 800 if x_dis > 800 else x_dis

                y_pid.SetPoint = 900  # set
                if abs(area_max - 900) < 50:
                    area_max = 900
                y_pid.update(area_max)  # current
                dy = y_pid.output
                y_dis += dy  # output
                y_dis = 0.12 if y_dis < 0.12 else y_dis
                y_dis = 0.25 if y_dis > 0.25 else y_dis

                z_pid.SetPoint = img_h / 2.0
                z_pid.update(center_y)
                dy = z_pid.output
                z_dis += dy

                z_dis = 0.22 if z_dis > 0.22 else z_dis
                z_dis = 0.17 if z_dis < 0.17 else z_dis

                target = ik.setPitchRanges(
                    (0, round(y_dis, 4), round(z_dis, 4)), -90, -85, -95)
                if target:
                    servo_data = target[1]
                    bus_servo_control.set_servos(joints_pub, 20, (
                        (3, servo_data['servo3']), (4, servo_data['servo4']), (5, servo_data['servo5']), (6, x_dis)))
    return output_image


def image_callback(ros_image):
    global lock

    image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                       buffer=ros_image.data)  # Convert custom image messages into images
    cv2_img = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

    frame = cv2_img.copy()
    frame_result = frame
    with lock:
        if __isRunning:
            frame_result = run(frame)
    rgb_image = cv2.cvtColor(frame_result, cv2.COLOR_BGR2RGB).tostring()
    ros_image.data = rgb_image

    image_pub.publish(ros_image)


def enter_func(msg):
    global lock
    global image_sub
    global __isRunning
    global org_image_sub_ed

    rospy.loginfo("enter object tracking")
    init()
    with lock:
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

    rospy.loginfo("exit object tracking")
    with lock:
        __isRunning = False
        reset()
        try:
            if org_image_sub_ed:
                org_image_sub_ed = False
                heartbeat_timer.cancel()
                image_sub.unregister()
        except BaseException as e:
            rospy.loginfo('%s', e)

    return [True, 'exit']


def start_running():
    global lock
    global __isRunning

    rospy.loginfo("start running object tracking")
    with lock:
        __isRunning = True


def stop_running():
    global lock
    global __isRunning

    rospy.loginfo("stop running object tracking")
    with lock:
        __isRunning = False
        reset()
        initMove(delay=False)


def set_running(msg):
    if msg.data:
        start_running()
    else:
        stop_running()

    return [True, 'set_running']


def set_target(msg):
    global lock
    global __target_character

    rospy.loginfo("%s", msg)
    with lock:
        __target_character = msg.data
        led = Led()
        led.index = 0
        led.rgb.r = range_rgb[__target_character][2]
        led.rgb.g = range_rgb[__target_character][1]
        led.rgb.b = range_rgb[__target_character][0]
        rgb_pub.publish(led)
        led.index = 1
        rgb_pub.publish(led)
        rospy.sleep(0.1)

    return [True, 'set_target']


def heartbeat_srv_cb(msg):
    global heartbeat_timer

    if isinstance(heartbeat_timer, Timer):
        heartbeat_timer.cancel()
    if msg.data:
        heartbeat_timer = Timer(5, rospy.ServiceProxy(
            '/character_tracking/exit', Trigger))
        heartbeat_timer.start()
    rsp = SetBoolResponse()
    rsp.success = msg.data

    return rsp


if __name__ == '__main__':
    rospy.init_node('character_tracking', log_level=rospy.DEBUG)

    joints_pub = rospy.Publisher(
        '/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)

    # get lab range from ros param server
    color_range = rospy.get_param('/lab_config_manager/color_range_list', {})

    # register result image publisher
    image_pub = rospy.Publisher(
        '/character_tracking/image_result', Image, queue_size=1)

    rgb_pub = rospy.Publisher('/sensor/rgb_led', Led, queue_size=1)

    enter_srv = rospy.Service('/character_tracking/enter', Trigger, enter_func)
    exit_srv = rospy.Service('/character_tracking/exit', Trigger, exit_func)
    running_srv = rospy.Service(
        '/character_tracking/set_running', SetBool, set_running)
    set_target_srv = rospy.Service(
        '/character_tracking/set_target', SetTarget, set_target)
    heartbeat_srv = rospy.Service(
        '/character_tracking/heartbeat', SetBool, heartbeat_srv_cb)

    debug = False
    if debug:
        rospy.sleep(0.2)
        enter_func(1)

        msg = SetTarget()
        msg.data = 'blue'

        set_target(msg)
        start_running()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    finally:
        cv2.destroyAllWindows()
