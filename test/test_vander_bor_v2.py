import unittest
import sys, os
import json
import numpy as np

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robots.vander_bot_v2 import VanderBotV2

class Testing(unittest.TestCase):
    def setUp(self):
        with open('conf/vander_bot_v2_config.json') as file:
            config = json.load(file)
        self.robot = VanderBotV2(config)
        ok = self.robot.connect()
        self.assertTrue(ok)
    
    def test_moveJ(self):
        print('test')
        ok = self.robot.servoJ(np.array([0.1, 0]), False)
        self.assertTrue(ok)
        ok = self.robot.wait()
        self.assertTrue(ok)


if __name__ == '__main__':
    unittest.main()