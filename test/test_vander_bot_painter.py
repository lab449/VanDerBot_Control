import unittest
import sys, os
import json
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robots.vander_bot_v2 import VanderBotV2
from kinematic.paint_kinematics import PaintKinematicsKDL
from painting.painter import Painter

class Testing(unittest.TestCase):
    def setUp(self):

        with open('conf/vander_bot_v2_config.json') as file:
            config = json.load(file)
        self.robot = VanderBotV2(config)

        with open('conf/vander_bot_v2_painter.json') as file:
            config_painter = json.load(file)

        ok = self.robot.connect()
        self.assertTrue(ok)
        self.robot_model = PaintKinematicsKDL('conf/vander_bot_v2.urdf')
        self.robot.setRobotModel(self.robot_model)

        self.painter = Painter(config_painter, self.robot)
    
    # def test_servoJ(self):
    #     ok = self.robot.servoJ(np.array([0.0, np.pi/2]), False)
    #     self.assertTrue(ok)
    #     ok = self.robot.wait()
    #     self.assertTrue(ok)

        # ok = self.robot.servoJ(np.array([0.0, 0.0]), True)

    def test_draw_json(self):
        with open('test/dicaprio.json') as file:
            path = json.load(file)
        ok = self.painter.draw_json(path)


if __name__ == '__main__':
    unittest.main()