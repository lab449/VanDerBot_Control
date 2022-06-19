import unittest
import sys, os
import json
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robots.vander_bot_v2 import VanderBotV2
from kinematic.paint_kinematics import PaintKinematicsKDL

class Testing(unittest.TestCase):
    def setUp(self):
        with open('conf/vander_bot_v2_config.json') as file:
            config = json.load(file)
        self.robot = VanderBotV2(config)
        ok = self.robot.connect()
        self.assertTrue(ok)

        self.robot_model = PaintKinematicsKDL('conf/vander_bot_v2.urdf')
        self.robot.setRobotModel(self.robot_model)
    
    # def test_servoJ(self):
    #     ok = self.robot.servoJ(np.array([0.0, np.pi/2]), False)
    #     self.assertTrue(ok)
    #     ok = self.robot.wait()
    #     self.assertTrue(ok)

        # ok = self.robot.servoJ(np.array([0.0, 0.0]), True)

    def test_servoL(self):
        ok = self.robot.servoL(np.array([0.0, 0.2]), False)
        self.assertTrue(ok)
        ok = self.robot.wait()
        self.assertTrue(ok)

        ok = self.robot.servoL(np.array([0.2, 0.2]), True)



if __name__ == '__main__':
    unittest.main()