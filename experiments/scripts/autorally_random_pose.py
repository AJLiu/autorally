#!/usr/bin/env python

import rospy
import os
from gazebo_msgs.srv import GetModelState, SetModelState, SpawnModel, DeleteModel
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
from tf.transformations import quaternion_from_euler
import time
import random
import math
import numpy as np
import ros_numpy
import logging
from sensor_msgs.msg import Image

from util import setup_log, is_costmap_free, CAR_LENGTH, CAR_WIDTH, OBS_WIDTH, OBS_LENGTH

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

def image_left_callback(msg):
    global image_left
    """Process images from the left camera."""
    image_left = ros_numpy.numpify(msg)

def image_right_callback(msg):
    global image_right
    """Process images from the right camera."""
    image_right = ros_numpy.numpify(msg)

def reset_obstacles(delete_model):
    print("Resetting Obstacles")
    obstacle_map = _cost_map['channel1']
    obstacle_map = obstacle_map.reshape(int((y_max-y_min) * ppm), int((x_max-x_min) * ppm))
    for i in range(num_obstacles):
        obs_name = "obstacle_{}_0".format(i)
        delete_model(str(obs_name))

def spawn_obstacles(spawn_model):
    print("Spawning Obstacles")
    pose = Pose()

    # Generate obstacles
    for i in range(num_obstacles):
        x,y = -50, -50
        yaw = random.uniform(0, 2*math.pi)
        obstacle_type = random.choice(obstacle_types)

        obs_free = False
        # Spawn in a random free position in the track
        while not is_costmap_free(cost_map, (x, y, yaw), (OBS_WIDTH, OBS_LENGTH), x_min = x_min, y_min = y_min, ppm = ppm) and \
              not obs_free:
            x = random.uniform(-15, 15)
            y = random.uniform(-15, 15)
            obs_free, idx_x, idx_y = is_costmap_free(obstacle_map, (x, y, yaw), (OBS_WIDTH, OBS_LENGTH), x_min = x_min, \
                                                        y_min = y_min, ppm = ppm, return_indices = True)
        set_pose(pose, x, y, yaw)
        obs_name = "obstacle_{}_0".format(i)
        print("Obstacle {} at ({},{}) yaw {}".format(obs_name, x, y, yaw))
        spawn_model(obs_name, obstacle_models[obstacle_type], "", pose, "world")
        obstacle_map[idx_y, idx_x] = 100

def main():
    get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    set_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    spawn_model = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
    delete_model = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

    data = []

    # Intialize msg objects
    pose = Pose()
    modelstate = ModelState()
    modelstate.model_name = "Alpha"

    # Generate Obstacles
    # reset_obstacles(delete_model)
    spawn_obstacles(spawn_model)

    print(obstacle_map.shape, obstacle_map.max(), obstacle_map.min())
    # Wait for images before starting
    time.sleep(1)

    for i in range(num_runs):
        x,y = -50, -50
        yaw = random.uniform(0, 2*math.pi)

        # Spawn in a random free position in the track
        while not is_costmap_free(cost_map, (x, y, yaw), (CAR_WIDTH, CAR_LENGTH), x_min = x_min, y_min = y_min, ppm = ppm) and \
              not is_costmap_free(obstacle_map, (x, y, yaw), (CAR_WIDTH, CAR_LENGTH), x_min = x_min, y_min = y_min, ppm = ppm):
            x = random.uniform(-15, 15)
            y = random.uniform(-15, 15)

                   
        logger.info('{}: ({: >6.2f},{: >6.2f})'.format( \
                    i, x, y))

        set_pose(pose, x, y, yaw)
        modelstate.pose = pose
        set_state(modelstate)

        # We spawn the car slightly off the ground, so this lets it fall into place
        time.sleep(.1)

        # Save current image
        data.append([image_left, image_right])
        time.sleep(1)


if __name__ == '__main__':
    name = 'random_pose'

    rospy.init_node(name)
    
    num_runs = rospy.get_param('~num_runs', 10)
    num_obstacles = rospy.get_param('~num_obstacles', 30)
    obstacle_types = rospy.get_param('~obstacle_types')
    map_path = rospy.get_param('~map_path')
    models_path = rospy.get_param('~models_path')
    x_origin = rospy.get_param('~x_origin')
    y_origin = rospy.get_param('~y_origin')
    image_left_topic = rospy.get_param('~image_left_topic', '/left_camera/image_raw')
    image_right_topic = rospy.get_param('~image_right_topic', '/right_camera/image_raw')
    _cost_map = np.load(map_path)

    # Configure cost map
    x_min, x_max = _cost_map['xBounds']
    y_min, y_max = _cost_map['yBounds']
    ppm = _cost_map['pixelsPerMeter']
    cost_map = _cost_map['channel0']
    obstacle_map = _cost_map['channel1']

    cost_map = cost_map.reshape(int((y_max-y_min) * ppm), int((x_max-x_min) * ppm))
    obstacle_map = obstacle_map.reshape(int((y_max-y_min) * ppm), int((x_max-x_min) * ppm))

    # Configure images
    image_left = None
    image_right = None
    image_left_sub = rospy.Subscriber(image_left_topic, Image, image_left_callback)
    image_right_sub = rospy.Subscriber(image_right_topic, Image, image_right_callback)

    # Load obstacle files
    obstacle_models = {}
    for obs in obstacle_types:
        with open("{}/{}/model.sdf".format(models_path, obs), "r") as f:
            obstacle_models[obs] = f.read()

    # Configure logger
    logger = setup_log(name)
    logger = logging.getLogger(name)

    # Run script
    main()
    # rospy.spin()