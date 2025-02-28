#!/usr/bin/env python3
from typing import Tuple
import numpy as np
import yaml

config_path = 'config.yaml'
with open(config_path, 'r') as file:
    config = yaml.safe_load(file)

weight_cfg = config['weight_conflict']

class Agent:
    def __init__(self, start: Tuple[int, int], goal: Tuple[int, int], max_steering_angle):
        self.start_index = start[0]
        self.start_angle = start[1]
        self.goal_index = goal[0]
        self.goal_angle = goal[1]
        if max_steering_angle is not None:
            self.max_steering_angle = max_steering_angle
        else:
            self.max_steering_angle = 360

        if self.max_steering_angle == 20:
            self.weight = weight_cfg
        elif self.max_steering_angle == 360:
            self.weight = 1.

    # Uniquely identify an agent with its start position
    def __hash__(self):
        return int(str(self.start_index) + str(self.start_angle))

    def __eq__(self, other: 'Agent'):
        return np.array_equal(self.start_index, other.start_index) and \
               np.array_equal(self.goal_index, other.goal_index)

    def __str__(self):
        return (f"Agent:(start_node={self.start_index}, start_angle={self.start_angle},"
                f" goal_node={self.goal_index}, goal_angle={self.goal_angle})")

    def __repr__(self):
        return self.__str__()
