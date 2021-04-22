import queue
import re
import sys
import threading

import serial
import serial.tools

from ant_robot.core import constants as const


class SerialBase(threading.Thread):
    def __init__(self, port=None, baudrate=9600, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                 stopbits=serial.STOPBITS_ONE, read_timeout=1, write_timeout=1,
                 read_line_terminator=serial.CR + serial.LF, write_line_terminator=serial.CR):
        super(SerialBase, self).__init__(daemon=True)
        self._function_queue = queue.Queue()
        self._feedback_queue = queue.Queue()
        self._serial = None

        self.__port = port
        self.__baudrate = baudrate
        self.__bytesize = bytesize
        self.__parity = parity
        self.__stopbits = stopbits
        self.__read_timeout = read_timeout
        self.__write_timeout = write_timeout
        self.__read_line_terminator = read_line_terminator
        self.__write_line_terminator = write_line_terminator
        self.__lock = threading.Lock()

    @property
    def is_open(self):
        return self._serial.is_open

    def run(self):
        self._serial = serial.Serial()
        if self.__port:
            self._serial.port = self.__port
        if self.__baudrate:
            self._serial.baudrate = self.__baudrate
        if self.__bytesize:
            self._serial.bytesize = self.__bytesize
        if self.__parity:
            self._serial.parity = self.__parity
        if self.__stopbits:
            self._serial.stopbits = self.__stopbits
        if self.__read_timeout:
            self._serial.timeout = self.__read_timeout
        if self.__write_timeout:
            self._serial.writeTimeout = self.__write_timeout

        while True:
            try:
                function_to_execute, args, kwargs = self._function_queue.get(block=True)
                function_to_execute(*args, **kwargs)
            except serial.SerialException as e:
                self._reset_io_buffer()
                msg = "Serial Error: {}".format(e)
                feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
                self._feedback_queue.put(feedback)
            except Exception as e:
                self._reset_io_buffer()
                msg = "Fatal Error: {}".format(e)
                feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_FATAL_ERROR, msg)
                self._feedback_queue.put(feedback)

    def _execute_on_thread(self, function_to_execute, *args, **kwargs):
        with self.__lock:
            self._function_queue.put((function_to_execute, args, kwargs))
            feedback = self._feedback_queue.get()
            return feedback

    def _config(self, port, baudrate, bytesize, parity, stopbits, read_timeout, write_timeout,
                read_line_terminator, write_line_terminator):
        try:
            if self._serial.is_open:
                self._serial.close()
            if port:
                self._serial.port = self.__port = port
            if baudrate:
                self._serial.baudrate = self.__baudrate = baudrate
            if bytesize:
                self._serial.bytesize = self.__bytesize = bytesize
            if parity:
                self._serial.parity = self.__parity = parity
            if stopbits:
                self._serial.stopbits = self.__stopbits = stopbits
            if read_timeout:
                self._serial.timeout = self.__read_timeout = read_timeout
            if write_timeout:
                self._serial.write_timeout = self.__write_timeout = write_timeout
            if read_line_terminator:
                self.__read_line_terminator = read_line_terminator
            if write_line_terminator:
                self.__write_line_terminator = write_line_terminator
            feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, None)
        except serial.SerialException as e:
            msg = "Serial Error (config): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    def _open(self):
        try:
            self._serial.open()
            self._reset_io_buffer()
            feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, None)
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (open): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    def _close(self):
        try:
            if self._serial.is_open:
                self._reset_io_buffer()
                self._serial.close()
                feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, None)
            else:
                msg = "port {} has not opened yet".format(self.__port)
                feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (close): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    def _query(self, query_data):
        tx_status, tx_out = self._transmit_data(query_data)
        if tx_status:
            rx_status, rx_out = self._receive_data()
            if rx_status:
                feedback = rx_out
            else:
                rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                    "{} <query>".format(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                feedback = rx_out
        else:
            tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                "{} <query>".format(tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
            feedback = tx_out
        self._feedback_queue.put(feedback)

    def _write(self, write_data):
        tx_status, tx_out = self._transmit_data(write_data)
        if tx_status:
            feedback = tx_out
        else:
            tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                "{} <write>".format(tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
            feedback = tx_out
        self._feedback_queue.put(feedback)

    def _read(self):
        rx_status, rx_out = self._receive_data()
        if rx_status:
            feedback = rx_out
        else:
            rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                "{} <read>".format(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
            feedback = rx_out
        self._feedback_queue.put(feedback)

    def _transmit_data(self, data, tx_codec="ascii"):
        try:
            if self._serial.is_open:
                data = "{}{}".format(data, self.__write_line_terminator.decode("ascii")).encode(tx_codec)
                self._serial.write(data)
                feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, None)
                return True, feedback
            else:
                msg = "Serial port {} has not opened yet".format(self.__port)
                raise serial.SerialException(msg)
        except serial.SerialException as e:
            port_list = self.__get_all_available_serial_port()
            if self.__port in port_list:
                self._reset_io_buffer()
                msg = "Serial Error (transmit): {}".format(e)
            else:
                msg = "Serial connection break (transmit): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
            return False, feedback

    def _receive_data(self, rx_codec="ascii"):
        try:
            if self._serial.is_open:
                in_data = self._serial.read_until(self.__read_line_terminator)
                if in_data == b'':
                    raise serial.SerialTimeoutException("timeout")
                else:
                    in_data = in_data.decode(rx_codec).rstrip()
                    feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, in_data)
                    return True, feedback
            else:
                msg = "Serial port {} has not opened yet".format(self.__port)
                raise serial.SerialException(msg)
        except UnicodeDecodeError as e:
            self._reset_io_buffer()
            msg = "Decode Error (open): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DECODE_ERROR, msg)
            return False, feedback
        except serial.SerialException as e:
            port_list = self.__get_all_available_serial_port()
            if self.__port in port_list:
                self._reset_io_buffer()
                msg = "Serial Error (receive): {}".format(e)
            else:
                msg = "Serial connection break (receive): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
            return False, feedback

    def _reset_io_buffer(self):
        if self._serial.is_open:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

    @staticmethod
    def __get_all_available_serial_port():
        port_list = []
        if sys.platform.startswith("win"):
            regex = re.compile("COM*")
        elif sys.platform.startswith("linux") or sys.platform.startswith("cygwin"):
            regex = re.compile("/dev/tty(USB|ACM)[0-9]+")
        else:
            raise EnvironmentError("Unsupported platform")

        for port in serial.tools.list_ports.comports():
            if regex.match(port.device):
                port_list.append(port.device)
        return port_list

    @staticmethod
    def _format_feedback(status, msg_type, msg):
        feedback = {const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS: status,
                    const.Serial.FeedbackKey.SERIAL_MSG_TYPE: msg_type,
                    const.Serial.FeedbackKey.SERIAL_MSG_CONTENT: msg}
        return feedback

    """ Basic command for serial communication
    query is used for write and read at once
    write is used for write only
    read is used for read only
    """

    def config(self, port=None, baudrate=None, bytesize=None, parity=None, stopbits=None,
               read_timeout=None, write_timeout=None, read_line_terminator=None, write_line_terminator=None):
        feedback = self._execute_on_thread(self._config, port, baudrate, bytesize, parity, stopbits,
                                           read_timeout, write_timeout, read_line_terminator, write_line_terminator)
        return feedback

    def open(self):
        feedback = self._execute_on_thread(self._open)
        return feedback

    def close(self):
        feedback = self._execute_on_thread(self._close)
        return feedback

    def query(self, query_data):
        feedback = self._execute_on_thread(self._query, query_data)
        return feedback

    def write(self, write_data):
        feedback = self._execute_on_thread(self._write, write_data)
        return feedback

    def read(self):
        feedback = self._execute_on_thread(self._read)
        return feedback
