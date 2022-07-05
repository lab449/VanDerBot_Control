import json
from jsonschema import validate, ValidationError
import numpy as np
import sys, os
from typing import Tuple

from robots.robot import Robot

import roboticstoolbox as rtb

class Painter:
    def __init__(self, painter_config: dict, robot: Robot):
        self.__config = painter_config
        with open('conf/painter_schema.json') as json_file:
            painter_schema = json.load(json_file)
        validate(instance=self.__config, schema=painter_schema)

        self.__robot = robot
        self.__canvas_tf = np.array(self.__config['canvas_tf'])
        self.__axis_speed = np.array(self.__config['axis_speed'])
        # print(self.__canvas_tf)

    @property
    def canvas_size(self) -> np.array:
        return self.__canvas_size

    @property
    def canvas_tf(self) -> np.array:
        return self.__canvas_tf

    @canvas_size.setter
    def canvas_size(self, canvas_size: np.array):
        self.__canvas_size = canvas_size
    
    def draw_json(self, paths: dict) -> bool:
        for path in paths['paths']:
            trj_list = []
            for point in path['points']:
                p = self.__canvas_tf @ np.concatenate( [np.array(point['p']), np.ones(1)])
                trj_list.append(p[:2])
            out_array = np.array(trj_list)
            trj = rtb.mstraj(out_array, 0.02, 0.1, qdmax=0.05)
            trajectory = trj.q
            self.__robot.servoL(np.concatenate([trajectory[0], np.array([0.0])]), True)
            self.__robot.servoJ(np.array([0.0, 0.0, -np.pi]))
            print('Take down')
            for xy in trajectory:
                self.__robot.servoL(np.concatenate([xy, np.array([0.0])]), True)
            self.__robot.servoJ(np.array([0.0, 0.0, np.pi]))
            print('Take up')
        return True