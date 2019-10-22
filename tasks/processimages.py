#!/bin/python3.7

import math

def avg(arr):
    return sum(arr) / len(arr)

camera_offset_front = .242
field_of_view = 1.0472 # radians
pixels_wide = 640 # pixels

def localize_image(robot_start, robot_end, bounding_box_start, bounding_box_end):
    # x is done
    robot_delta_x = robot_end["x"] - robot_start["x"]
    bounding_box_start_width = bounding_box_start["x2"] - bounding_box_start["x1"]
    bounding_box_end_width = bounding_box_end["x2"] - bounding_box_end["x1"]
    x_distance_to_obj = robot_delta_x * bounding_box_end_width / (bounding_box_start_width + bounding_box_end_width)

    robot_delta_y = robot_end["y"] - robot_start["y"]
    bounding_box_start_height = bounding_box_start["y2"] - bounding_box_start["y1"],
    bounding_box_end_height = bounding_box_end["y2"] - bounding_box_end["y1"]

    bounding_box_start_area = bounding_box_start_height * bounding_box_start_width
    bounding_box_end_area = bounding_box_end_height * bounding_box_end_width

    # need to use the current angle the robot is at
    start_angle_y = (bounding_box_start_width / 2 + bounding_box_start["x1"]) \
        / pixels_wide * field_of_view # in radian output
    end_angle_y = (bounding_box_end_width / 2 + bounding_box_end["x1"]) \
        / pixels_wide * field_of_view # in radian output
    y_distance_to_obj = bounding_box_end_area * math.cos(end_angle_y) * robot_delta_y / \
        (bounding_box_end_area * math.cos(end_angle_y) - bounding_box_start_area * math.cos(start_angle_y)) + camera_offset_front
    return x_distance_to_obj, y_distance_to_obj
