from six import with_metaclass

from ant_robot.core.utils import Singleton


class SerialDeviceMgr(with_metaclass(Singleton), object):
    def __init__(self):
        self.__serial_device_dict = dict()

    def get_serial_device(self, device_name, device_class):
        try:
            return self.__serial_device_dict[device_name]
        except KeyError:
            print("Required device is not found! A new default serial device is registered.")
            return self.register_serial_device(device_name, device_class)

    def register_serial_device(self, device_name, device_class, *args, **kwargs):
        if device_name in self.__serial_device_dict.keys():
            return self.__serial_device_dict[device_name]
        else:
            serial_device_instance = device_class(device_name, *args, **kwargs)
            self.__serial_device_dict[device_name] = serial_device_instance
            return self.__serial_device_dict[device_name]
