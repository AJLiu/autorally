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


def setup_log(log_name):

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
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.3
    qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw

def main():
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state",SetModelState)

    pose = Pose()


    modelstate = ModelState()
    modelstate.model_name = "Alpha"

    for i in range(num_runs):
        logger.info(i)

        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)
        yaw = random.uniform(0, 2*math.pi)

        set_pose(pose, x, y, yaw)
        modelstate.pose = pose
        set_state(modelstate)

        logger.info(get_state('Alpha', 'world'))
        time.sleep(5)
if __name__ == '__main__':
    name = 'random_pose'

    rospy.init_node(name)
    
    num_runs = rospy.get_param('~num_runs')

    logger = setup_log(name)
    logger = logging.getLogger(name)
    main()
    # rospy.spin()