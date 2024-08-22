from zaber_motion import Units
from zaber_motion.ascii import Connection

from abc import ABC, abstractmethod

class TurntableHandler(ABC):
    @abstractmethod
    def __init__(self, connection_port: str = None):
        return None

    @abstractmethod
    def rotate_to_pos(self, num_degrees: float, use_rads=False):
        return None


class ZaberDeviceController(TurntableHandler):

    def __init__(self, connection_port: str):
        self.connection_port = connection_port
    
    def rotate_to_pos(self, num_degrees: float, use_rads=False):
        with Connection.open_serial_port(self.connection_port) as connection:
            connection.enable_alerts()

            device_list = connection.detect_devices()
            device = device_list[0]

            self.axis = device.get_axis(1)
            if not self.axis.is_homed():
                self.axis.home()

            self.axis.move_absolute(num_degrees, Units.ANGLE_RADIANS if use_rads else Units.ANGLE_DEGREES)


def get_turntable_handler(turntable_handler, connection_port):
    if turntable_handler == "zaber":
        return ZaberDeviceController(connection_port)
        
        
        """ ADD YOUR METHOD HERE """
        
    else:
        raise Exception(f"Unknown turntable handler type: {turntable_handler}")
