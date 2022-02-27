#!/usr/bin/python3
# coding=utf8

import sys
import time
import cv2
import math

import HiwonderSDK.yaml_handle as yaml_handle
import HiwonderSDK.Misc as Misc

# color tracking

size = (320, 240)

range_rgb = {
    'red': (0, 0, 255),
    'blue': (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255),
}


def load_config():
    lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)
    return lab_data


class Perception:

    def __init__(self):
        self.__target_color = ''

    def setTargetColor(self, color):
        self.__target_color = color

    def get_range_rgb(self):
        return range_rgb

    # Find the contour with the largest area
    # The parameter is a list of contours to be compared

    def getAreaMaxContour(self, contours):
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

    def morphological_operations(self, frame_lab, color_range):

        if self.__target_color in color_range:
            target_color_range = color_range[self.__target_color]
            frame_mask = cv2.inRange(frame_lab, tuple(target_color_range['min']), tuple(
                target_color_range['max']))  # Perform bit operations on the original image and mask
            eroded = cv2.erode(frame_mask, cv2.getStructuringElement(
                cv2.MORPH_RECT, (3, 3)))  # corrosion
            dilated = cv2.dilate(eroded, cv2.getStructuringElement(
                cv2.MORPH_RECT, (3, 3)))  # dilation
            contours = cv2.findContours(
                dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # find the contour

        return contours

    def isContourValid(self, area_max, areaMaxContour, size=(640, 480)):
        """Check if contour is of valid size"""
        if area_max > 1000:  # have found the largest area
            (center_x, center_y), radius = cv2.minEnclosingCircle(
                areaMaxContour)  # Get the smallest circumcircle

            center_x = int(
                Misc.map(center_x, 0, size[0], 0, self.__orig_imsize[1]))
            center_y = int(
                Misc.map(center_y, 0, size[1], 0, self.__orig_imsize[0]))
            radius = int(
                Misc.map(radius, 0, size[0], 0, self.__orig_imsize[1]))
            if radius > 100:
                return False, None
            return True, (center_x, center_y, radius)
        else:
            return False, None

    def find_object(self, img, color_range):
        global x_dis, y_dis, z_dis

        img_copy = img.copy()
        img_h, img_w = img.shape[:2]

        cv2.line(img, (int(img_w / 2 - 10), int(img_h / 2)),
                 (int(img_w / 2 + 10), int(img_h / 2)), (0, 255, 255), 2)
        cv2.line(img, (int(img_w / 2), int(img_h / 2 - 10)),
                 (int(img_w / 2), int(img_h / 2 + 10)), (0, 255, 255), 2)

        frame_resize = cv2.resize(
            img_copy, size, interpolation=cv2.INTER_NEAREST)
        # Convert image to LAB space
        frame_lab = cv2.cvtColor(frame_resize, cv2.COLOR_BGR2LAB)

        area_max = 0
        area_max_contour = 0

        # Performing morphological operations
        contours = self.morphological_operations(frame_lab, color_range)

        area_max_contour, area_max = self.getAreaMaxContour(
            contours)  # find the maximum contour
        valid, (center_x, center_y, radius) = self.isContourValid(
            area_max, area_max_contour,)
        if not valid:
            return img

        cv2.circle(img, (int(center_x), int(center_y)),
                   int(radius), range_rgb[self.__target_color], 2)

        return img



if __name__ == '__main__':

    lab_data = load_config()

    # Create perception object
    obj_detection = Perception()
    obj_detection.setTargetColor('red')
    cap = cv2.VideoCapture(-1)
    time.sleep(3)

    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            # Find color and make bounding boxes on frames
            Frame = obj_detection.find_object(frame, range_rgb)
            frame_resize = cv2.resize(Frame, (320, 240))
            cv2.imshow('frame', frame_resize)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()
