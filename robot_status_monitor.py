from functools import partial

from six import with_metaclass

from ant_robot.core import constants as const
from ant_robot.core.gui.models.robot_setting_model import RobotSettingModel
from ant_robot.core.robot.motor_cmd import MotorCMD
from ant_robot.core.robot.robot_calculation_process import RobotCalculationProcess
from ant_robot.core.serial.serial_device_mgr import SerialDeviceMgr
from ant_robot.core.serial.serial_robot_async_wrapper import SerialRobotAsyncWrapper
from ant_robot.core.utils import AsyncWrapperWithFeedback, Singleton


class RobotStatusMonitor(with_metaclass(Singleton), object):
    def __init__(self):
        self.__serial_device_mgr = SerialDeviceMgr()
        self.__serial_robot = self.__serial_device_mgr.get_serial_device(const.Serial.DeviceName.SERIAL_ROBOT,
                                                                         SerialRobotAsyncWrapper)
        self.__robot_calculation_process = RobotCalculationProcess()
        self.__robot_setting_model = RobotSettingModel()

        self.__async_wrapper = AsyncWrapperWithFeedback()
        self.__async_wrapper.name = "RobotStatusMonitor"
        self.__async_wrapper.start()

    def set_synchronization_context(self):
        self.__async_wrapper.set_synchronization_context()

    def __get_robot_actual_coordinate(self):
        current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
        motor1_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor1_datum_axis_pos
        motor2_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor2_datum_axis_pos
        motor3_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor3_datum_axis_pos

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(1)
        feedback = self.__serial_robot.query(cmd)
        if bool(feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]):
            motor1_current_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor1_datum_axis_pos_in_qc
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(2)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            motor2_current_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor2_datum_axis_pos_in_qc
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(3)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            motor3_current_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor3_datum_axis_pos_in_qc
        else:
            return feedback

        x0, y0, z0 = self.__robot_calculation_process.calculate_coord_from_motor_theta_in_qc(motor1_current_theta_in_qc,motor2_current_theta_in_qc,motor3_current_theta_in_qc)
        if x0 and y0 and z0:
            result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA,(x0, y0, z0))
            return result_feedback
        else:
            result_feedback = self.__format_feedback(False,const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,None)
            return result_feedback

    def __get_robot_actual_velocity(self):
        current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
        motor1_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor1_datum_axis_pos
        motor2_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor2_datum_axis_pos
        motor3_datum_axis_pos_in_qc = current_loaded_robot_setting.cs_motor3_datum_axis_pos

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(1)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            current_motor1_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor1_datum_axis_pos_in_qc
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(2)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            current_motor2_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor2_datum_axis_pos_in_qc
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_position_in_qc(3)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            current_motor3_theta_in_qc = \
                int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - motor3_datum_axis_pos_in_qc
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_velocity_in_rpm(1)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            motor1_omega_in_rpm = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_velocity_in_rpm(2)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            motor2_omega_in_rpm = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
        else:
            return feedback

        cmd = MotorCMD.QUERY.get_actual_velocity_in_rpm(3)
        feedback = self.__serial_robot.query(cmd)
        if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
            motor3_omega_in_rpm = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
        else:
            return feedback

        vx, vy, vz = self.__robot_calculation_process.calculate_velocity_from_angular_velocity(motor1_omega_in_rpm,
                                                                                               motor2_omega_in_rpm,
                                                                                               motor3_omega_in_rpm,
                                                                                               current_motor1_theta_in_qc,
                                                                                               current_motor2_theta_in_qc,
                                                                                               current_motor3_theta_in_qc)
        if vx and vy and vz:
            result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA,
                                                     (vx, vy, vz))
            return result_feedback
        else:
            result_feedback = self.__format_feedback(False,
                                                     const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,
                                                     None)
            return result_feedback

    @staticmethod
    def __format_feedback(status, msg_type, msg):
        feedback = {const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS: status,
                    const.Serial.FeedbackKey.SERIAL_MSG_TYPE: msg_type,
                    const.Serial.FeedbackKey.SERIAL_MSG_CONTENT: msg}
        return feedback

    def get_robot_actual_coordinate(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__get_robot_actual_coordinate)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def get_robot_actual_velocity(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__get_robot_actual_velocity)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)
