import re
import sys
import time
import math

# import time
from functools import partial
import logging
import serial
import json
import os
import serial.tools.list_ports

# global port="COM5"
from functools import partial
from pubsub import pub
from PyQt5.QtCore import QObject, pyqtSignal

from six import with_metaclass

from ant_robot.core import constants as const
from ant_robot.core import pub_message as pub_msg
from ant_robot.core.gui.ctrl_mgr import CtrlMgr, AbstractCtrl
from ant_robot.core.gui.models.serial_setting_model import SerialSettingModel
from ant_robot.core.serial.serial_joystick_async_wrapper import (
    SerialJoystickAsyncWrapper,
)
from ant_robot.core.gui.views.serial_setting_view import SerialSettingView

# from ant_robot.core.gui.models.robot_setting_model import RobotSettingModel
from ant_robot.core.robot.motor_cmd import MotorCMD
from ant_robot.core.robot.robot_calculation_process import RobotCalculationProcess
from ant_robot.core.serial.serial_device_mgr import SerialDeviceMgr
from ant_robot.core.serial.serial_robot_async_wrapper import SerialRobotAsyncWrapper
from ant_robot.core.utils import AsyncWrapperWithFeedback, Singleton
from PySide2.QtCore import QObject, Signal

# from ant_robot.core.utils import SingletonQObjectMeta
from ant_robot.core import utils

# from ant_robot.core.utils import SingletonQObjectMeta
from ant_robot.core import pub_message as pub_msg


# from ant_robot.core import constants as const
# from ant_robot.core.robot.motor_cmd import MotorCMD
from ant_robot.core.serial.serial_base import SerialBase
from ant_robot.core.serial.serial_robot import SerialRobot
from ant_robot.core.robot.robot_status_monitor import RobotStatusMonitor
from ant_robot.core.robot.robot_motion_executor import RobotMotionExecutor

# from ant_robot.core.gui.ctrls.robot_setting_ctrl import RobotSettingCtrl
from ant_robot.core.gui.ctrls.serial_setting_ctrl import SerialSettingCtrl
from ant_robot.core.gui.models.robot_setting_model import RobotSettingModel
from ant_robot.core.robot.robot_calculation_process import RobotCalculationProcess



# QObject,SerialBase,AbstractCtrl,metaclass=utils.SingletonPyQt):
class RobotMove:
    sig_disconnected = Signal()
    sig_progress_initialize = pyqtSignal(int, str)
    sig_progress_update = pyqtSignal(int)
    sig_progress_finish = pyqtSignal()
    sig_progress_interrupt = pyqtSignal()
    is_movement_correction_enabled = True

    def __init__(self, *args, **kwargs):
        self.__model = SerialSettingModel()
        self.__serial_device_mgr = SerialDeviceMgr()
        self.__serial_robot = self.__serial_device_mgr.get_serial_device(const.Serial.DeviceName.SERIAL_ROBOT, SerialRobotAsyncWrapper)
        self.__serial_robot.set_synchronization_context()
        self.__robot_calculation_process = RobotCalculationProcess()
        self.__async_wrapper = AsyncWrapperWithFeedback()
        self.__async_wrapper.name = "RobotStatusMonitor"
        self.__async_wrapper.start()
        # super(SerialRobot,self).__init__(*args, **kwargs)
        self.__has_acknowledgement = False
        self.__robot_setting_model = RobotSettingModel()
        self.__robot_calculation_process = RobotCalculationProcess()



    def robot_serial_config(self):
        device_name = const.Serial.DeviceName.SERIAL_ROBOT
        serial_device = self.__serial_robot
        device_model = self.__model.get_device_by_name(device_name)
        port = "COM3"
        self.__config_serial_port(
            device_name,
            port,
            device_model.baudrate,
            device_model.bytesize,
            device_model.parity,
            device_model.stopbit,
            device_model.read_timeout,
            device_model.write_timeout,
            device_model.read_line_terminator,
            device_model.write_line_terminator,
        )

        feedback = serial_device.open()
        print(f"robot_serial_config: {feedback}")
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            device_model.port = port

    def robot_connect_status(self):
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
                        feedback = self._format_feedback(
                            True,
                            const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                            "Connect Success! Command acknowledgement is enabled",
                        )
                    else:
                        self.__has_acknowledgement = False
                        feedback = self._format_feedback(
                            True,
                            const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                            "Connect Success! Command acknowledgement is disabled",
                        )
                else:
                    self._serial.close()
                    rx_out[
                        const.Serial.FeedbackKey.SERIAL_MSG_CONTENT
                    ] = "{} <open>".format(
                        rx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]
                    )
                    feedback = rx_out
            else:
                self._serial.close()
                tx_out[
                    const.Serial.FeedbackKey.SERIAL_MSG_CONTENT
                ] = "{} <open>".format(
                    tx_out[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]
                )
                feedback = tx_out
        except serial.SerialException as e:
            self._reset_io_buffer()
            msg = "Serial Error (open): {}".format(e)
            feedback = self._format_feedback(
                False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR, msg
            )
        self._feedback_queue.put(feedback)

        abnormal_behavior_count = 0

        motor1_current_reading = self.__try_get_motor_current_reading(1)
        motor2_current_reading = self.__try_get_motor_current_reading(2)
        motor3_current_reading = self.__try_get_motor_current_reading(3)

        motor1_position_reading = self.__try_get_motor_actual_position_reading(1)
        motor2_position_reading = self.__try_get_motor_actual_position_reading(2)
        motor3_position_reading = self.__try_get_motor_actual_position_reading(3)

        if motor1_current_reading < 5 or motor1_current_reading > 1500:
            abnormal_behavior_count += 1
        if motor2_current_reading < 5 or motor2_current_reading > 1500:
            abnormal_behavior_count += 1
        if motor3_current_reading < 5 or motor3_current_reading > 1500:
            abnormal_behavior_count += 1
        if motor1_position_reading < -10000 or motor1_position_reading > 150000:
            abnormal_behavior_count += 1
        if motor2_position_reading < -10000 or motor2_position_reading > 150000:
            abnormal_behavior_count += 1
        if motor3_position_reading < -10000 or motor3_position_reading > 150000:
            abnormal_behavior_count += 1

        if abnormal_behavior_count >= 3:
            result_feedback = self.__format_feedback(
                False,
                const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_LOST_POWER_ERROR,
                None,
            )

        else:
            result_feedback = self.__format_feedback(
                True,
                const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DATA,
                "Robot Operation is fine",
            )

        return result_feedback

    def __config_serial_port(self, device_name, port, baudrate, bytesize, parity, stopbits,
                             read_timeout, write_timeout, read_line_terminator, write_line_terminator):
        feedback = None
        if device_name == const.Serial.DeviceName.SERIAL_ROBOT:
            feedback = self.__serial_robot.config(port, baudrate, bytesize, parity, stopbits,
                                                  read_timeout, write_timeout,
                                                  read_line_terminator, write_line_terminator)
        if device_name == const.Serial.DeviceName.SERIAL_JOYSTICK:
            feedback = self.__serial_joystick.config(port, baudrate, bytesize, parity, stopbits,
                                                     read_timeout, write_timeout,
                                                     read_line_terminator, write_line_terminator)

        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            print(f"__config_serial_port: Configure serial port success!")
        else:
            print(f"__config_serial_port: Configure serial port failed!")

    def _reset_io_buffer(self):
        if self._serial.is_open:
            self._serial.reset_input_buffer()
            self._serial.reset_output_buffer()

    

    def robot_position(self):
        try:
            current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
            motor1_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor1_datum_axis_pos
            motor2_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor2_datum_axis_pos
            motor3_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor3_datum_axis_pos

            cmd = MotorCMD.QUERY.get_actual_position_in_qc(1)
            feedback = self.__serial_robot.query(cmd)
            print(feedback)
            if bool(feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]):
                motor1_current_theta_in_qc = \
                    int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor1_datum_axis_pos_in_qc
            else:
                return feedback

            cmd = MotorCMD.QUERY.get_actual_position_in_qc(2)
            feedback = self.__serial_robot.query(cmd)
            print(feedback)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                motor2_current_theta_in_qc = \
                    int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor2_datum_axis_pos_in_qc
            else:
                return feedback

            cmd = MotorCMD.QUERY.get_actual_position_in_qc(3)
            feedback = self.__serial_robot.query(cmd)
            print(feedback)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                motor3_current_theta_in_qc = \
                    int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor3_datum_axis_pos_in_qc
            else:
                return feedback

            x0, y0, z0 = self.__robot_calculation_process.calculate_coord_from_motor_theta_in_qc(motor1_current_theta_in_qc,motor2_current_theta_in_qc,motor3_current_theta_in_qc)
            if x0 and y0 and z0:
                result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA,(x0, y0, z0))
                print(result_feedback)
                return result_feedback
            else:
                result_feedback = self.__format_feedback(False,const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,None)
                print(result_feedback)
                return result_feedback
        except (serial.SerialException, ValueError) as e:
            result_feedback = self.__format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_SERIAL_ERROR,str(e))

    def __format_feedback(status, msg_type, msg):
        feedback = {const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS: status,const.Serial.FeedbackKey.SERIAL_MSG_TYPE: msg_type,const.Serial.FeedbackKey.SERIAL_MSG_CONTENT: msg}
        return feedback


# serialconnect=SerialRobot()
# status=RobotStatusMonitor()
# executor=RobotMotionExecutor()
# setting= RobotSettingCtrl()
# serialsetting=SerialSettingCtrl()


# device_name = const.Serial.DeviceName.SERIAL_ROBOT
# serial_device = serialsetting.__serial_robot
# device_model = serialsetting.__model.get_device_by_name(device_name)
# port="COM5"
# serialsetting.__config_serial_port(device_name,
# port,
# device_model.baudrate,
# device_model.bytesize,
# device_model.parity,
# device_model.stopbit,
# device_model.read_timeout,
# device_model.write_timeout,
# device_model.read_line_terminator,
# device_model.write_line_terminator)
# feedback = serial_device.open()
# serialsetting.__view.write_text(feedback)
# if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
# device_model.port = port


# serialsetting.__get_all_available_serial_port()
# serialsetting.__open_serial_port(self)
# serialsetting.__config_serial_port(self,"Robot",port,9600)


# Setting up robot, connecting to it and loading profile
# setting.__load_profile_from_model(self, 2.1)

# serialconnect._open()
# status.__get_robot_actual_coordinate()
# status.__get_robot_operation_status()
# while executor.__calibration():
# status.__get_robot_actual_coordinate()
# status.__get_robot_operation_status()
# executor.__move_by_position(self,10,10,-49.15,1.0,False)
# status.__get_robot_actual_coordinate()
# status.__get_robot_operation_status()

# Close connection to robot
# serial_device = serialsetting.__serial_robot
# if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
# serialsetting.__view.write_text("Current port is closed!")
# else:
# serialsetting.__view.write_text(feedback)

# if __name__ == '__main__':
# if sys.platform == "win32":
# from multiprocessing import freeze_support

# freeze_support()

# sys.path.insert(0, os.path.abspath(os.path.join(__file__, os.pardir, os.pardir)))
# sys.path.insert(0, os.path.abspath(os.path.join(__file__, os.pardir, os.pardir, "ant_robot_python")))
# robot_serial_config()
# robot_connect_status()


if __name__ == "__main__":
    robotmove = RobotMove()
    robotmove.robot_serial_config()
    robotmove.robot_position()
    # robotmove.robot_connect_status()