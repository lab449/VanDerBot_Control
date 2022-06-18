from abc import ABC, abstractmethod
import json
from jsonschema import validate, ValidationError
import numpy as np
import sys, os

# sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class RobotConnection(ABC):
    def __init__(self, connection_config: dict, timeout: float):
        self._connection_config = connection_config
        self._timeout = timeout

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def moveJ(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def moveL(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def busy(self) -> bool:
        pass
    
    @abstractmethod
    def wait(self) -> bool:
        pass

class RobotSerialConnection(RobotConnection):
    def __init__(self, connection_config: dict, timeout: float = 1.0):
        super().__init__(connection_config, timeout)
        with open('conf/serial_robot_schema.json') as json_file:
            connection_schema = json.load(json_file)
        validate(instance=self._connection_config, schema=connection_schema)

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def moveJ(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def moveL(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def busy(self) -> bool:
        pass
    
    @abstractmethod
    def wait(self) -> bool:
        pass
        

class RobotUDPConnection(RobotConnection):
    def __init__(self, connection_config: dict, timeout: float = 0.1):
        super().__init__(connection_config, timeout)
        with open('conf/ethernet_robot_scheme.json') as json_file:
            connection_schema = json.load(json_file)
        validate(instance=self._connection_config, schema=connection_schema)

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def moveJ(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def moveL(self, point: np.array, blocked: bool = True) -> bool:
        pass

    @abstractmethod
    def busy(self) -> bool:
        pass

    @abstractmethod
    def wait(self) -> bool:
        pass