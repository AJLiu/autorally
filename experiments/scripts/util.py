import numpy as np
import sys
import logging
import itertools

CAR_WIDTH = 1
CAR_LENGTH = 1
OBS_WIDTH = 1
OBS_LENGTH = 1

def get_rotated_idxs(width, length, x, y, rad):
    """Generates a list of indices of all points of an object at some position and rotation
    
    Arguments:
        width {int} -- Width of the object in pixels
        length {int} -- Length of the object in pixels
        x {int} -- X coordinate of the object in pixels
        y {int} -- Y coordinate of the object in pixels
        rad {float} -- Rotation of the object in radians
    
    Returns:
        tuple -- Tuple of two ndarrays representing the x and y coordinates
    """

    T = np.array([
        [np.cos(rad), -np.sin(rad), x],
        [np.sin(rad), np.cos(rad), y],
        [0, 0, 1]
    ])

    x_idx = np.arange(width) - width // 2
    y_idx = np.arange(length) - length // 2


    idx = np.array(list(itertools.product(x_idx,y_idx,[1])))
    idxs = idx.dot(T.T)[:,:2].T
    idxs = np.hstack((idxs.astype(int), np.ceil(idxs).astype(int)))
    return idxs[0], idxs[1]

def is_costmap_free(cost_map, pos, shape, 
                    offset=(-8.5, 0), x_min=-25.0, 
                    y_min=-15.0, ppm=20, return_indices=False):
    """Util function to see if an object is free in the cost map
    
    Arguments:
        cost_map {ndarray} -- cost map, values between 0 and 1 indicate in track
        pos {tuple} -- position of the object in meters and radians (x, y, direction)
        shape {tuple} -- shape of the object in meters (width, length)
    
    Keyword Arguments:
        offset {tuple} -- The offset of the starting position relative to the global origin (default: {(-8.5, 0)})
        x_min {float} -- The minimum x coord from the costmap (default: {-25.0})
        y_min {float} -- The minimum y coord from the costmap (default: {-15.0})
        ppm {int} -- The number of pixels per meter (default: {20})
        return_indices {boolean} -- If set to true, the value returned will also hold the indices of the rotated object in the costmap
    
    Returns:
        boolean or (boolean, x, y)
    """


    x, y, rad = pos
    max_y, max_x = cost_map.shape

    if x is None or y is None:
        if return_indices:
            return False, [], []
        else:
            return False

    x_off, y_off = offset
    width = int(shape[0] * ppm / 2.0)
    length = int(shape[1] * ppm / 2.0)

    # Transform x and y coordinates to fit cost_map
    x = int((x - x_min + x_off) * ppm)
    y = int(max_y - (y-y_min + y_off) * ppm)

    # Get the indexes 
    x, y = get_rotated_idxs(width, length, x, y, rad)

    # Return False if x or y is out of boundaries of the cost_map
    if np.any((x < 0) | (x >= max_x)) or np.any((y < 0) | (y >= max_y)):
        if return_indices:
            return False, [], []
        else:
            return False

    # cost of <1 represents pixels that are in the track
    free = np.all(cost_map[y, x] < 1)

    if return_indices:
        return free, x, y
    else:
        return free


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
