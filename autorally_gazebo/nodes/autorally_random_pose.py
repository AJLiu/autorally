#!/usr/bin/env python

import rospy
import logging
import os
import sys
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler
import time
import random
import math
import numpy as np
import ros_numpy
from sensor_msgs.msg import Image


def setup_log(log_name):
    """Initializes handlers for logging"""
    logger = logging.getLogger(log_name)
    logger.setLevel(logging.DEBUG)
    # fh = logging.FileHandler(path + log_name)
    # fh.setLevel(logging.INFO)
    formatter = logging.Formatter('[%(name)s] %(message)s')
    # fh.setFormatter(formatter)
    # logger.addHandler(fh)
    ch = logging.StreamHandler(sys.stdout)
    ch.setLevel(logging.INFO)
    ch.setFormatter(formatter)
    logger.addHandler(ch)
    return logger

def set_pose(pose, x, y, yaw):
    """Converts yaw into quaternion coordinates and assigns the values to a pose object"""
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.3
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

def is_free(x, y, size):
    """Return True if the square of length 'size' centered at (x,y) does not 
    contain an obstacle"""

    if x is None or y is None:
        return False

    _x = int((x + 15) * ppm)
    _y = int((y + 15) * ppm)
    _size = int(size * ppm / 2.0)

    free = np.all(cost_map[_y-_size : _y+_size, _x-_size : _x+_size] < 1)
    return free

def image_left_callback(msg):
    global image_left
    """Process images from the left camera."""
    image_left = ros_numpy.numpify(msg)

def image_right_callback(msg):
    global image_right
    """Process images from the right camera."""
    image_right = ros_numpy.numpify(msg)


def main():
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

    data = []

    # Intialize msg objects
    pose = Pose()
    modelstate = ModelState()
    modelstate.model_name = "Alpha"

    # Wait for images before starting
    time.sleep(1)

    for i in range(num_runs):
        x,y = None, None
        yaw = random.uniform(0, 2*math.pi)

        # Spawn in a random free position in the track
        while not is_free(x,y,1):           
            x = random.uniform(-15, 15)
            y = random.uniform(-15, 15)

        logger.info('{}: global({: >6.2f},{: >6.2f}) : {}'.format( \
                    i, x, y, is_free(x, y, 1)))


        set_pose(pose, x, y, yaw)
        modelstate.pose = pose
        set_state(modelstate)

        # We spawn the car slightly off the ground, so this lets it fall into place
        time.sleep(.1)

        # Save current image
        data.append([image_left, image_right])
        time.sleep(.1)


if __name__ == '__main__':
    name = 'random_pose'

    rospy.init_node(name)
    
    num_runs = rospy.get_param('~num_runs', 10)
    map_path = rospy.get_param('~map_path')
    x_origin = rospy.get_param('~x_origin')
    y_origin = rospy.get_param('~y_origin')
    image_left_topic = rospy.get_param('~image_left_topic', '/left_camera/image_raw')
    image_right_topic = rospy.get_param('~image_right_topic', '/right_camera/image_raw')
    _cost_map = np.load(map_path)

    # Configure cost map
    x_min, x_max = _cost_map['xBounds']
    y_min, y_max = _cost_map['yBounds']
    ppm = _cost_map['pixelsPerMeter']
    vals = _cost_map['channel0']

    cost_map = vals.reshape(int((y_max-y_min) * ppm), int((x_max-x_min) * ppm))

    # Configure images
    image_left = None
    image_right = None
    image_left_sub = rospy.Subscriber(image_left_topic, Image, image_left_callback)
    image_right_sub = rospy.Subscriber(image_right_topic, Image, image_right_callback)

    # Configure logger
    logger = setup_log(name)
    logger = logging.getLogger(name)

    # Run script
    main()
    # rospy.spin()