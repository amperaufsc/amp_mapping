
import numpy as np
from Perception.include.obstacle import Obstacle

from typing import List
from SLAM.include.state import State


class Map:
    def __init__(self, obstacles: List[Obstacle] = None):
        if obstacles is None:
            self.map = list()
        else:
            self.map = obstacles

    def add_new_obstacle(self, obstacle:Obstacle):
        if not bool(self.map):
            new_id = 0
        else:
            new_id = self.map[-1].id + 1
        new_obstacle = Obstacle(obstacle.x,obstacle.y,obstacle.confidence,
                                 obstacle.label, obstacle.deviation, obstacle.count, new_id)
        self.map.append(new_obstacle)

    def get_map_as_array(self):
        return [o.get_obstacle_as_array() for o in self.map]

    def get_map_with_count_as_array(self):
        return [o.get_obstacle_with_extra_parameters_as_array() for o in self.map]

    def get_map_with_count_as_numpy_array(self):
        return np.array([o.get_obstacle_with_extra_parameters_as_array() for o in self.map])

    def get_map_as_numpy_array(self):
        return np.array([o.get_obstacle_as_array() for o in self.map])
