import numpy as np
import sys
import logging

def is_costmap_free(cost_map, pos, size, 
                    offset=(-8.5, 0), x_min=-25.0, 
                    y_min=-15.0, ppm=20):
    """Return True if the square of length 'size' centered at pos=(x,y) does not 
    contain an obstacle"""

    x, y = pos
    max_y, max_x = cost_map.shape

    if x is None or y is None:
        return False

    x_off, y_off = offset
    width = int(size * ppm / 2.0)

    # Transform x and y coordinates to fit cost_map
    x = int((x - x_min + x_off) * ppm)
    y = int(max_y - (y-y_min + y_off) * ppm)

    # Return False if x or y is out of boundaries of the cost_map
    if x-width < 0 or y-width < 0 or x+width >= max_x or y+width >= max_y:
        return False

    # cost of <1 represents pixels that are in the track
    free = np.all(cost_map[y - width : y + width, x - width : x + width] < 1)

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
