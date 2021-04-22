from abc import ABC, abstractmethod


class ISerialDevice(ABC):
    @property
    @abstractmethod
    def is_open(self):
        pass

    @property
    @abstractmethod
    def device_name(self):
        pass

    @abstractmethod
    def set_synchronization_context(self):
        pass

    @abstractmethod
    def config_async(self, port=None, baudrate=None, databits=None, parity=None, stopbits=None,
                     read_timeout=None, write_timeout=None, read_line_terminator=None, write_line_terminator=None,
                     callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def open_async(self, callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def close_async(self, callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def query_async(self, cmd, callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def write_async(self, cmd, callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def read_async(self, callback=None, msg_to_pub=None, **kwargs):
        pass

    @abstractmethod
    def config(self, port=None, baudrate=None, databits=None, parity=None, stopbits=None,
               read_timeout=None, write_timeout=None, read_line_terminator=None, write_line_terminator=None):
        pass

    @abstractmethod
    def open(self):
        pass

    @abstractmethod
    def close(self):
        pass

    @abstractmethod
    def query(self, cmd):
        pass

    @abstractmethod
    def write(self, cmd):
        pass

    @abstractmethod
    def read(self):
        pass
