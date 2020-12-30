import math
import numpy as np
from nav_msgs.msg import OccupancyGrid
import random


def generate_freespace_indices( map_: OccupancyGrid) -> tuple:
    """generate the indices(represented in a tuple) of the freesapce based on the map

    Returns:
        indices_y_x(tuple): indices of the non-occupied cells, the first element is the y-axis indices,
        the second element is the x-axis indices.
    """
    width_in_cell, height_in_cell = map_.info.width, map_.info.height
    map_2d = np.reshape(map_.data, (height_in_cell, width_in_cell))
    indices_y_x = np.where(map_2d == 0)
    return indices_y_x


def get_random_pos_on_map(free_space_indices, map_: OccupancyGrid):
    """
    Args:
        indices_y_x(tuple): a 2 elementary tuple stores the indices of the non-occupied cells, the first element is the y-axis indices,
            the second element is the x-axis indices.
        map (OccupancyGrid): map proviced by the ros map service

    Returns:
       x_in_meters,y_in_meters,theta
    """

    # def is_pos_valid(x, y, map_):
    #     # in pixel
    #     cell_radius = math.ceil((self.ROBOT_RADIUS + 0.1) / map_.info.resolution)
    #     x_index = int((x - map_.info.origin.position.x) / map_.info.resolution)
    #     y_index = int((y - map_.info.origin.position.y) / map_.info.resolution)

    #     # check occupancy around (x_index,y_index) with cell_radius
    #     for i in range(x_index - cell_radius, x_index + cell_radius, 1):
    #         for j in range(y_index - cell_radius, y_index + cell_radius, 1):
    #             index = j * map_.info.width + i
    #             if index >= len(map_.data):
    #                 return False
    #             try:
    #                 value = map_.data[index]
    #             except IndexError:
    #                 print("IndexError: index: %d, map_length: %d" % (index, len(map_.data)))

    #                 return False
    #             if value != 0:

    #                 return False
    #     return True

    assert len(free_space_indices) == 2 and len(free_space_indices[0]) == len(
        free_space_indices[1]), "free_space_indices is not correctly setup"
    n_freespace_cells = len(free_space_indices[0])
    pos_valid = False
    n_check_failed = 0
    x_in_meters, y_in_meters = None, None
    while not pos_valid:
        idx = random.randint(0, len(n_freespace_cells))
        # in cells
        y_in_cells, x_in_cells = free_space_indices[0][idx], free_space_indices[1][idx]
        # convert x, y in meters
        y_in_meters = y_in_cells * map_.info.resolution + map_.info.origin.position.y
        x_in_meters = x_in_cells * map_.info.resolution + map_.info.origin.position.x
        # pos_valid = is_pos_valid(x_in_meters, y_in_meters, map_)
        if not pos_valid:
            n_check_failed += 1
            if n_check_failed > 100:
                raise Exception("cann't find any no-occupied space please check the map information")
        # in radius
    theta = random.uniform(-math.pi, math.pi)

    return x_in_meters, y_in_meters, theta
