import re
import time

import serial

from ant_robot.core import constants as const
from ant_robot.core.robot.motor_cmd import MotorCMD
from ant_robot.core.serial.serial_base import SerialBase


class SerialRobot(SerialBase):
    def __init__(self, *args, **kwargs):
        super(SerialRobot, self).__init__(*args, **kwargs)
        self.__has_acknowledgement = False

    def _open(self):
        try:
            self._serial.open()
            self._reset_io_buffer()

            test_cmd = MotorCMD.QUERY.get_configuration_status(1)
            tx_status, tx_out = self._transmit_data(test_cmd)
            if tx_status:
                rx_status, rx_out = self._receive_data()
                if rx_status:
                    result = int(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                    if MotorCMD.CMD_PARSER_CST.is_cmd_acknowledgement_enabled(result):
                        self.__has_acknowledgement = True
                        feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                                                         "Connect Success! Command acknowledgement is enabled")
                    else:
                        self.__has_acknowledgement = False
                        feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                                                         "Connect Success! Command acknowledgement is disabled")
                else:
                    self._serial.close()
                    rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                        "{} <open>".format(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                    feedback = rx_out
            else:
                self._serial.close()
                tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                    "{} <open>".format(tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                feedback = tx_out
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (open): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    def _query(self, cmd):
        try:
            self.__check_if_cmd_is_valid_for_query(cmd)
            tx_status, tx_out = self._transmit_data(cmd)
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
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (query): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    def _write(self, cmd):
        try:
            tx_status, tx_out = self._transmit_data(cmd)
            if tx_status:
                if self.__check_if_answ_cmd(cmd):
                    refresh_status, refresh_out = self.__refresh_serial_and_motor_controller()
                    if refresh_status:
                        if self.__has_acknowledgement:
                            feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                                                             "Command acknowledgement is enabled")
                        else:
                            feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                                                             "Command acknowledgement is disabled")
                    else:
                        feedback = refresh_out
                else:
                    if self.__has_acknowledgement:
                        rx_status, rx_out = self._receive_data()
                        if rx_status:
                            self.__cmd_acknowledgement_checker(str(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))
                            feedback = rx_out
                        else:
                            rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                                "{} <write>".format(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                            feedback = rx_out
                    else:
                        feedback = tx_out
            else:
                tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                    "{} <write>".format(tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                feedback = tx_out
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (write): {}".format(e)
            feedback = self._format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg)
        self._feedback_queue.put(feedback)

    @staticmethod
    def __cmd_acknowledgement_checker(decoded_output):
        if decoded_output == "OK":
            pass
        elif decoded_output == "Unknown command":
            raise serial.SerialException("Unknown command")
        elif decoded_output == "Invalid parameter":
            raise serial.SerialException("Invalid parameter")
        elif decoded_output == "Command not available":
            raise serial.SerialException("Command not available")
        elif decoded_output == "Overtemperature - drive disabled":
            raise serial.SerialException("Overtemperature - drive disabled")

    __cmd_pattern_query = re.compile("^[1-3]")

    @staticmethod
    def __check_if_cmd_is_valid_for_query(cmd):
        if SerialRobot.__cmd_pattern_query.search(cmd):
            pass
        else:
            raise serial.SerialException("Query command is not valid. Please specify a certain motor index ahead.")

    __cmd_pattern_ANSW = re.compile(r'ANSW[0,2]', re.IGNORECASE)

    @staticmethod
    def __check_if_answ_cmd(cmd):
        if SerialRobot.__cmd_pattern_ANSW.search(cmd):
            return True
        else:
            return False

    __cmd_pattern_config = re.compile(r'ANSW[0,2]|BAUD', re.IGNORECASE)

    @staticmethod
    def __check_if_configuration_cmd(cmd):
        if SerialRobot.__cmd_pattern_config.search(cmd):
            return True
        else:
            return False

    def __refresh_serial_and_motor_controller(self):
        # Solve serial transmission garbage response code when sending command after using ANSW or ANSW2
        feedback = None
        self._reset_io_buffer()
        refresh_status = False
        for motor_index in range(1, 4):
            test_cmd = MotorCMD.QUERY.get_configuration_status(motor_index)
            tx_status, tx_out = self._transmit_data(test_cmd)
            if tx_status:
                rx_status, rx_out = self._receive_data()
                if rx_status:
                    result = int(rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                    if motor_index == 3:
                        if MotorCMD.CMD_PARSER_CST.is_cmd_acknowledgement_enabled(result):
                            self.__has_acknowledgement = True
                        else:
                            self.__has_acknowledgement = False
                        feedback = self._format_feedback(True, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA, None)
                        refresh_status = True
                else:
                    time.sleep(0.1)
                    self._reset_io_buffer()
                    if motor_index == 3:
                        feedback = self._format_feedback(False,
                                                         const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DECODE_ERROR,
                                                         "Refresh command acknowledgement failed! Please try again!")
                        refresh_status = False
            else:
                tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT] = \
                    "{} <refresh_serial>".format(tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                feedback = tx_out
                refresh_status = False
        return refresh_status, feedback

    def query(self, cmd):
        if self.__check_if_configuration_cmd(cmd):
            feedback = self._execute_on_thread(self._write, cmd)
        else:
            feedback = self._execute_on_thread(self._query, cmd)
        return feedback

    def write(self, cmd):
        feedback = self._execute_on_thread(self._write, cmd)
        return feedback
