from .robot import RobotSerialConnection
import numpy as np
import serial
import time
import logging
import struct
from threading import Thread, Lock

class VanderBotV2(RobotSerialConnection):
    def __init__(self, connection_config: dict):
        super().__init__(connection_config)
        self.__serial_connection = serial.Serial(
            port=connection_config['port'],
            baudrate=connection_config['baudrate'],
            timeout=0.5
        )
        self.__thread = Thread(target=self.__busy_checking)
        self.__lock = Lock()
        self.__busy = True
        self.__last_state: np.array = None

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


    def servoL(self, point: np.array, model: PaintKinematicsKDL) -> bool:
        pass

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

    def __busy_checking(self)-> bool:
         while True:
            try:
                if self.__serial_connection.in_waiting != 0:
                    ok = self.__serial_connection.read(self.__serial_connection.inWaiting()).decode()
                    print("busy_checking ", ok)
                    if ok == 'OK':
                        logging.info('Moving complete')
                        with self.__lock:
                            self.__busy = False
                time.sleep(1e-3)
            except KeyboardInterrupt:
                self.__lock.release()
                break
