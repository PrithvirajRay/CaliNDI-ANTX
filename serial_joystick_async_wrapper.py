from functools import partial

from ant_robot.core.serial.i_serial_device import ISerialDevice
from ant_robot.core.serial.serial_robot import SerialBase
from ant_robot.core.utils import AsyncWrapperWithFeedback


class SerialJoystickAsyncWrapper(ISerialDevice):

    @property
    def device_name(self):
        return self.__device_name

    @property
    def is_open(self):
        return self.__serial_device.is_open

    def __init__(self, device_name, *args, **kwargs):
        self.__device_name = device_name
        self.__serial_device = SerialBase(*args, **kwargs)
        self.__serial_device.name = device_name
        self.__serial_device.start()

        self.__async_wrapper = AsyncWrapperWithFeedback()
        self.__async_wrapper.name = str(device_name) + "bg_worker"
        self.__async_wrapper.start()

    def set_synchronization_context(self):
        self.__async_wrapper.set_synchronization_context()

    def config_async(self, port=None, baudrate=None, bytesize=None, parity=None, stopbits=None,
                     read_timeout=None, write_timeout=None, read_line_terminator=None, write_line_terminator=None,
                     callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.config,
                                      port, baudrate, bytesize, parity, stopbits, read_timeout, write_timeout,
                                      read_line_terminator, write_line_terminator)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def open_async(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.open)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def close_async(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.close)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def query_async(self, cmd, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.query, cmd)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def write_async(self, cmd, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.write, cmd)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def read_async(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__serial_device.read)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def config(self, port=None, baudrate=None, bytesize=None, parity=None, stopbits=None,
               read_timeout=None, write_timeout=None, read_line_terminator=None, write_line_terminator=None):
        return self.__serial_device.config(port, baudrate, bytesize, parity, stopbits,
                                           read_timeout, write_timeout, read_line_terminator, write_line_terminator)

    def open(self):
        return self.__serial_device.open()

    def close(self):
        return self.__serial_device.close()

    def query(self, cmd):
        return self.__serial_device.query(cmd)

    def write(self, cmd):
        return self.__serial_device.write(cmd)

    def read(self):
        return self.__serial_device.read()
