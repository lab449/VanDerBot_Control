import json
from jsonschema import validate, ValidationError
import numpy as np
import sys, os
from typing import Tuple

from robots.robot import RobotConnection

class Painter:
    def __init__(self, painter_config: dict):
        self.__config = painter_config
        with open('conf/painter_schema.json') as json_file:
            painter_schema = json.load(json_file)
        validate(instance=self._connection_config, schema=painter_schema)

        self.__robot = None
        self.__canvas_size = self.__config['canvas_size']
        self.__canvas_tf = self.__config['canva_tf']

    
    def settingUp_robot(self, robot: RobotConnection):
        self.__robot = robot

    @property
    def canvas_size(self) -> Tuple[float, float]:
        return self.__canvas_size
    
    def draw_json()