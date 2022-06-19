from .robot import RobotSerial
import numpy as np
import serial
import sys, os
import time
import logging
import struct
from threading import Thread, Lock

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from kinematic.paint_kinematics import PaintKinematicsKDL

class VanderBotV2(RobotSerial):
    def __init__(self, connection_config: dict):
        super().__init__(connection_config)
        self.__serial_connection = serial.Serial(
            port=connection_config['port'],
            baudrate=connection_config['baudrate'],
            timeout=0.5
        )
        self.__thread = Thread(target=self.__busy_checking, daemon=True)
        self.__lock = Lock()
        self.__busy = True
        self.__last_state: np.array = np.array([-np.pi/2, np.pi/2])

    def connect(self) -> bool:
        while True:
            if self.__serial_connection.in_waiting != 0:
                start = self.__serial_connection.read(self.__serial_connection.inWaiting()).decode()
                if start == 'START':
                    logging.info('Connection established')
                    self.__busy = False
                    break
                else:
                    return False
            time.sleep(1e-3)
        self.__thread.start()
        return True

    def servoL(self, point: np.array, blocked: bool = True) -> bool:
        if self._robot_model is None:
            raise Exception('Robot model is not defined')
        jpose = self._robot_model.ik_pose(point, self.__last_state)
        print(jpose)
        if jpose is None:
            return False
        
        self.servoJ(jpose, blocked)
        return True

    def servoJ(self, point_rad: np.array, blocked: bool = True) -> bool:
        point = np.rad2deg(point_rad)
        if point.size != 2:
            return False
        
        if self.__busy:
            return False
        if not blocked:
            data_in = point.tolist() + [0.0]
            buf = struct.pack('%sf' % len(data_in), *data_in)
            self.__serial_connection.write(buf)
            self.__busy = True 
        else:
            data_in = point.tolist() + [0.0]
            buf = struct.pack('%sf' % len(data_in), *data_in)
            self.__serial_connection.write(buf)
            self.__busy = True     
            self.wait()      
        return True
    
    def wait(self) -> bool:
        while self.__busy:
            time.sleep(1e-3)
        return True

    @property
    def busy(self) -> bool:
        return self.__busy

    @property
    def state(self) -> np.array:
        return self.__last_state

    def __busy_checking(self)-> bool:
         while True:
            try:
                if self.__serial_connection.in_waiting != 0:
                    ok = self.__serial_connection.read(self.__serial_connection.inWaiting()).decode()
                    if ok == 'OK':
                        logging.info('Moving complete')
                        print('OK')
                        self.__busy = False
            except KeyboardInterrupt:
                self.__lock.release()
                break
