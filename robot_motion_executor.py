import math
import time
from functools import partial

import serial
from PyQt5.QtCore import QObject, pyqtSignal

from ant_robot.core import constants as const
from ant_robot.core import utils
from ant_robot.core.gui.models.robot_setting_model import RobotSettingModel, RobotSettingDataModel
from ant_robot.core.robot.motor_cmd import MotorCMD
from ant_robot.core.robot.robot_calculation_process import RobotCalculationProcess
from ant_robot.core.serial.serial_device_mgr import SerialDeviceMgr
from ant_robot.core.serial.serial_robot_async_wrapper import SerialRobotAsyncWrapper


class RobotMotionExecutor(QObject, metaclass=utils.SingletonPyQt):
    sig_progress_initialize = pyqtSignal(int, str)
    sig_progress_update = pyqtSignal(int)
    sig_progress_finish = pyqtSignal()
    sig_progress_interrupt = pyqtSignal()
    is_movement_correction_enabled = True

    def __init__(self):
        super(RobotMotionExecutor, self).__init__()

        self.__serial_device_mgr = SerialDeviceMgr()
        self.__serial_robot = self.__serial_device_mgr.get_serial_device(const.Serial.DeviceName.SERIAL_ROBOT,
                                                                         SerialRobotAsyncWrapper)
        self.__robot_calculation_process = RobotCalculationProcess()
        self.__robot_setting_model = RobotSettingModel()

        self.__async_wrapper = utils.AsyncWrapperWithFeedback()
        self.__async_wrapper.name = "RobotMotionExecutor"
        self.__async_wrapper.start()

    def set_synchronization_context(self):
        self.__async_wrapper.set_synchronization_context()

    def __calibration(self):
        try:
            self.sig_progress_initialize.emit(100, "Motor Calibration")
            current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()

            # Step 1: bottom ring calibration
            # Set motor parameters for initiating calibration
            cmd = MotorCMD.MOTION_CONTROL. \
                set_peak_current_limit_in_mA(0, current_loaded_robot_setting.os_peak_current_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL. \
                set_continuous_current_limit_in_mA(0, current_loaded_robot_setting.os_continuous_current_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(0, current_loaded_robot_setting.os_max_speed)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(0, current_loaded_robot_setting.os_acceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(0, current_loaded_robot_setting.os_deceleration)
            self.__serial_robot.write_async(cmd)

            for motor_index in range(1, 4):
                current_limit = 0
                if motor_index == 1:
                    current_limit = current_loaded_robot_setting.cs_motor1_down_to_base_current_limit
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(1)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(2)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(3)
                    self.__serial_robot.write_async(cmd)
                elif motor_index == 2:
                    current_limit = current_loaded_robot_setting.cs_motor2_down_to_base_current_limit
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(2)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(3)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(1)
                    self.__serial_robot.write_async(cmd)
                elif motor_index == 3:
                    current_limit = current_loaded_robot_setting.cs_motor3_down_to_base_current_limit
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(3)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(1)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(2)
                    self.__serial_robot.write_async(cmd)

                cmd = MotorCMD.MOTION_CONTROL.set_peak_current_limit_in_mA(motor_index, current_limit)
                self.__serial_robot.write_async(cmd)
                cmd = MotorCMD.MOTION_CONTROL.set_continuous_current_limit_in_mA(motor_index, current_limit)
                self.__serial_robot.write_async(cmd)
                cmd = MotorCMD.MOTION_CONTROL.set_target_relative_position_in_qc(motor_index, -300000)
                self.__serial_robot.write_async(cmd)
                # Move
                cmd = MotorCMD.MOTION_CONTROL.initiate_motion(motor_index)
                self.__serial_robot.write_async(cmd)
                time.sleep(0.1)
                is_success = self.__wait_until_reaching_current_limit(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Current Limit Unreached!")
                # wait until motion completely stop
                time.sleep(1.0)

                cmd = MotorCMD.MOTION_CONTROL.set_home_position_in_qc(motor_index, "0")
                self.__serial_robot.write_async(cmd)
                time.sleep(0.2)

                self.sig_progress_update.emit(15)

                if motor_index == 1:
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(1)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(2)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(3)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(2, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(3, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)
                elif motor_index == 2:
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(2)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(3)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(1)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(3, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(1, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)
                elif motor_index == 3:
                    cmd = MotorCMD.MOTION_CONTROL.disable_motor(3)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(1)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(2)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(1, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_relative_position_in_qc(2, -current_loaded_robot_setting.cs_force_correction_move)
                    self.__serial_robot.write_async(cmd)

                # Move
                cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
                self.__serial_robot.write_async(cmd)
                time.sleep(0.1)
                for motor in range(1, 4):
                    is_success = self.__wait_until_position_attained(motor, 6)
                    if not is_success:
                        raise TimeoutError("Timeout(Motor Calibration): Target Position Not Attained!")
                time.sleep(1.0)

                self.sig_progress_update.emit(5)
                if motor_index == 3:
                    cmd = MotorCMD.MOTION_CONTROL.enable_motor(3)
                    self.__serial_robot.write_async(cmd)

            # Step 2: move all three motors to the pull-up height wrt. the bottom base
            cmd = MotorCMD.MOTION_CONTROL.set_peak_current_limit_in_mA(0, 550)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_continuous_current_limit_in_mA(0, 550)
            self.__serial_robot.write_async(cmd)

            # Step 2.1: raise all robot arm to 85% of pull-up height synchronously
            for motor_index in range(1, 4):
                cmd = MotorCMD.QUERY.get_actual_position_in_qc(motor_index)
                feedback = self.__serial_robot.query(cmd)
                if bool(feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]):
                    result = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                    motor_relative_location_in_qc = (result -
                                                     int(current_loaded_robot_setting.cs_pull_up_position * 0.85))
                    motor_max_speed_in_qc_per_second = abs(4.0 * motor_relative_location_in_qc / 3.0)
                    motor_max_speed_in_rpm = int(math.floor(motor_max_speed_in_qc_per_second / 50))
                    motor_acceleration = int(math.ceil(4 * motor_max_speed_in_qc_per_second / 50))
                    if motor_acceleration > 30000:
                        motor_acceleration = 30000
                    motor_deceleration = motor_acceleration
                    cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(motor_index, motor_max_speed_in_rpm)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.set_acceleration(motor_index, motor_acceleration)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL.set_deceleration(motor_index, motor_deceleration)
                    self.__serial_robot.write_async(cmd)
                    cmd = MotorCMD.MOTION_CONTROL. \
                        set_target_absolute_position_in_qc(motor_index,
                                                           int(current_loaded_robot_setting.cs_pull_up_position * 0.85))
                    self.__serial_robot.write_async(cmd)
                else:
                    raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))

            # Move
            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.1)
            for motor_index in range(1, 4):
                is_success = self.__wait_until_position_attained(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Target Position Not Attained!")
            time.sleep(1.0)

            self.sig_progress_update.emit(5)

            # Step 2.2: move all robot arm to pull-up height fast enough to overcome torque and friction
            pull_up_speed = current_loaded_robot_setting.cs_pull_up_speed
            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(0, pull_up_speed)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(0, 20000)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(0, 20000)
            self.__serial_robot.write_async(cmd)

            pull_up_position = current_loaded_robot_setting.cs_pull_up_position
            cmd = MotorCMD.MOTION_CONTROL.set_target_absolute_position_in_qc(0, pull_up_position)
            self.__serial_robot.write_async(cmd)
            # Move
            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.1)
            for motor_index in range(1, 4):
                is_success = self.__wait_until_position_attained(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Target Position Not Attained!")
            time.sleep(1.0)

            self.sig_progress_update.emit(5)

            # Step 3: slowly push down top ring until current limit reaching 300mA
            cmd = MotorCMD.MOTION_CONTROL.set_peak_current_limit_in_mA(0, 300)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_continuous_current_limit_in_mA(0, 300)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(0, 300)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(0, 2000)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(0, 2000)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_target_absolute_position_in_qc(0, 0)
            self.__serial_robot.write_async(cmd)
            # Move
            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.1)
            for motor_index in range(1, 4):
                is_success = self.__wait_until_reaching_current_limit(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Current Limit Unreached!")
            time.sleep(1.0)

            self.sig_progress_update.emit(5)

            # Disable and enable motor is to cancel previous unfinished movement cmd
            cmd = MotorCMD.MOTION_CONTROL.disable_motor(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.5)
            cmd = MotorCMD.MOTION_CONTROL.enable_motor(0)
            self.__serial_robot.write_async(cmd)

            self.sig_progress_update.emit(5)

            # Step 4: slowly push down top ring until preset current is reached to tight the ring
            push_down_current_limit = current_loaded_robot_setting.cs_push_down_current_limit
            cmd = MotorCMD.MOTION_CONTROL.set_peak_current_limit_in_mA(0, push_down_current_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_continuous_current_limit_in_mA(0, push_down_current_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(0, 20)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_target_absolute_position_in_qc(0, 0)
            self.__serial_robot.write_async(cmd)
            # Move
            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.1)
            for motor_index in range(1, 4):
                is_success = self.__wait_until_reaching_current_limit(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Current Limit Unreached!")
            time.sleep(1.0)

            self.sig_progress_update.emit(5)

            cmd = MotorCMD.MOTION_CONTROL.disable_motor(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.05)
            cmd = MotorCMD.MOTION_CONTROL.enable_motor(0)
            self.__serial_robot.write_async(cmd)

            # Step 4.1: pull all motor arm up a little bit to compensate the push-down pressure
            cmd = MotorCMD.MOTION_CONTROL.set_target_relative_position_in_qc(0, 400)
            self.__serial_robot.write_async(cmd)
            # Move
            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)
            time.sleep(0.1)
            for motor_index in range(1, 4):
                is_success = self.__wait_until_position_attained(motor_index, 6)
                if not is_success:
                    raise TimeoutError("Timeout(Motor Calibration): Target Position Not Attained!")
            time.sleep(1.0)

            self.sig_progress_update.emit(5)

            # Step 5: reset all motor parameters to normal operation mode
            max_speed = current_loaded_robot_setting.os_max_speed
            acceleration = current_loaded_robot_setting.os_acceleration
            deceleration = current_loaded_robot_setting.os_deceleration
            target_pos_corridor = current_loaded_robot_setting.os_target_pos_corridor
            positive_pos_limit = current_loaded_robot_setting.os_positive_pos_limit
            negative_pos_limit = -current_loaded_robot_setting.os_negative_pos_limit
            peak_current_limit = current_loaded_robot_setting.os_peak_current_limit
            continuous_current_limit = current_loaded_robot_setting.os_continuous_current_limit

            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(0, max_speed)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(0, acceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(0, deceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_position_corridor_in_qc(0, target_pos_corridor)
            self.__serial_robot.write_async(cmd)

            cmd = MotorCMD.MOTION_CONTROL.set_position_limit_in_qc(0, positive_pos_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_position_limit_in_qc(0, negative_pos_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.enable_position_limit(0)
            self.__serial_robot.write_async(cmd)

            cmd = MotorCMD.MOTION_CONTROL.set_peak_current_limit_in_mA(0, peak_current_limit)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_continuous_current_limit_in_mA(0, continuous_current_limit)
            self.__serial_robot.write_async(cmd)

            # Step 6: calibrate each of the motor's datum axis
            calibrated_x0 = current_loaded_robot_setting.cs_calibrated_x0
            calibrated_y0 = current_loaded_robot_setting.cs_calibrated_y0
            calibrated_z0 = current_loaded_robot_setting.cs_calibrated_z0
            motor_default_theta_in_qc_after_calibration = self.__robot_calculation_process. \
                calculate_motor_theta_in_qc_from_coord(x0=calibrated_x0, y0=calibrated_y0, z0=calibrated_z0)

            if (motor_default_theta_in_qc_after_calibration[0]
                    and motor_default_theta_in_qc_after_calibration[1]
                    and motor_default_theta_in_qc_after_calibration[2]):

                self.sig_progress_update.emit(3)
                motor1_reading = self.__try_get_motor_actual_position_reading(1)
                motor1_datum_axis_pos_in_qc = motor1_reading - motor_default_theta_in_qc_after_calibration[0]

                self.sig_progress_update.emit(3)
                motor2_reading = self.__try_get_motor_actual_position_reading(2)
                motor2_datum_axis_pos_in_qc = motor2_reading - motor_default_theta_in_qc_after_calibration[1]

                self.sig_progress_update.emit(3)
                motor3_reading = self.__try_get_motor_actual_position_reading(3)
                motor3_datum_axis_pos_in_qc = motor3_reading - motor_default_theta_in_qc_after_calibration[2]

                current_loaded_robot_setting.cs_motor1_datum_axis_pos = motor1_datum_axis_pos_in_qc
                current_loaded_robot_setting.cs_motor2_datum_axis_pos = motor2_datum_axis_pos_in_qc
                current_loaded_robot_setting.cs_motor3_datum_axis_pos = motor3_datum_axis_pos_in_qc

                result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA,
                                                         (motor1_datum_axis_pos_in_qc,
                                                          motor2_datum_axis_pos_in_qc,
                                                          motor3_datum_axis_pos_in_qc))
                print("homing complete")
                self.sig_progress_finish.emit()
                return result_feedback
            else:
                result_feedback = self.__format_feedback(False,
                                                         const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,
                                                         None)
                return result_feedback
        except (serial.SerialException, TimeoutError) as e:
            self.sig_progress_interrupt.emit()
            result_feedback = self.__format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DECODE_ERROR,
                                                     str(e))
            return result_feedback

    def __move_by_position(self, new_x0, new_y0, new_z0, time_limit=1.0, is_waiting_motion_complete=False):
        try:
            if RobotMotionExecutor.is_movement_correction_enabled:
                new_x0, new_y0 = self.__movement_correction(new_x0, new_y0)
            self.sig_progress_initialize.emit(100, "Robot Movement")
            current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
            # Calculate new motor theta
            motor_new_theta_in_qc = self.__robot_calculation_process.calculate_motor_theta_in_qc_from_coord(new_x0,
                                                                                                            new_y0,
                                                                                                            new_z0)

            motor1_new_theta_in_qc = motor_new_theta_in_qc[0]
            motor2_new_theta_in_qc = motor_new_theta_in_qc[1]
            motor3_new_theta_in_qc = motor_new_theta_in_qc[2]

            if motor1_new_theta_in_qc and motor2_new_theta_in_qc and motor3_new_theta_in_qc:
                # Query current motor theta
                motor1_reading = self.__try_get_motor_actual_position_reading(1)
                motor1_current_location_in_qc = motor1_reading - current_loaded_robot_setting.cs_motor1_datum_axis_pos
                motor1_relative_location_in_qc = motor1_new_theta_in_qc - motor1_current_location_in_qc

                motor2_reading = self.__try_get_motor_actual_position_reading(2)
                motor2_current_location_in_qc = motor2_reading - current_loaded_robot_setting.cs_motor2_datum_axis_pos
                motor2_relative_location_in_qc = motor2_new_theta_in_qc - motor2_current_location_in_qc

                motor3_reading = self.__try_get_motor_actual_position_reading(3)
                motor3_new_location_in_qc = motor3_reading - current_loaded_robot_setting.cs_motor3_datum_axis_pos
                motor3_relative_location_in_qc = motor3_new_theta_in_qc - motor3_new_location_in_qc
            else:
                self.sig_progress_interrupt.emit()
                result_feedback = self.__format_feedback(False,
                                                         const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,
                                                         None)
                return result_feedback

            self.sig_progress_update.emit(20)

            """
            Assume theta_i is initial position at time 0, theta_f is final position at time t_f, 
            and both initial velocity and final velocity are 0.
            To satisfy these 4 constraints, a polynomial of at least third degree is required.
            which is 
            theta = a0 + a1 * t + a2 * t^2 + a3 * t^3, thus, 
            velocity = a1 + 2 * a2 * t + 3 * a3 * t^2, and,
            acceleration = 2 * a2 + 6 * a3 * t
             
            resolving these can get 
            v = (6 * delta_theta / t_f^2) * t - (6 * delta_theta / t_f^3) * t^2, where delta_theta = theta_f - theta_i ---1
            a = 6 * delta_theta / t_f^2 - (12 * delta_theta / t_f^3) * t -------------------------------------------------2
            1 and 2 => max_acceleration = 4 * max_speed / t_f
            Apply this relationship to real motor speed curve, which is a trapezoid shape profile instead. 
            Calculate new speed with the same t_f and delta_theta, 
            => real_max_speed = 4 * delta_theta / (3 * t_f)
            """
            motor1_max_speed_in_qc_per_second = abs(4.0 * motor1_relative_location_in_qc / (3.0 * time_limit))
            motor2_max_speed_in_qc_per_second = abs(4.0 * motor2_relative_location_in_qc / (3.0 * time_limit))
            motor3_max_speed_in_qc_per_second = abs(4.0 * motor3_relative_location_in_qc / (3.0 * time_limit))

            # The unit for acceleration and deceleration is (revolution/60) / s^2, so one needs to divide it by 50
            motor1_acceleration = int(math.ceil(4.0 * motor1_max_speed_in_qc_per_second / time_limit / 50.0))
            if motor1_acceleration > 30000:
                motor1_acceleration = 30000
            motor1_deceleration = motor1_acceleration

            motor2_acceleration = int(math.ceil(4.0 * motor2_max_speed_in_qc_per_second / time_limit / 50.0))
            if motor2_acceleration > 30000:
                motor2_acceleration = 30000
            motor2_deceleration = motor2_acceleration

            motor3_acceleration = int(math.ceil(4.0 * motor3_max_speed_in_qc_per_second / time_limit / 50.0))
            if motor3_acceleration > 30000:
                motor3_acceleration = 30000
            motor3_deceleration = motor3_acceleration

            # 1 rpm = 3000 qc/min = 50 qc/s
            motor1_max_speed_in_rpm = int(math.floor(motor1_max_speed_in_qc_per_second / 50.0))
            if motor1_max_speed_in_rpm > 30000:
                motor1_max_speed_in_rpm = 30000
            motor2_max_speed_in_rpm = int(math.floor(motor2_max_speed_in_qc_per_second / 50.0))
            if motor2_max_speed_in_rpm > 30000:
                motor2_max_speed_in_rpm = 30000
            motor3_max_speed_in_rpm = int(math.floor(motor3_max_speed_in_qc_per_second / 50.0))
            if motor3_max_speed_in_rpm > 30000:
                motor3_max_speed_in_rpm = 30000

            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(1, motor1_max_speed_in_rpm)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(1, motor1_acceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(1, motor1_deceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_target_relative_position_in_qc(1, motor1_relative_location_in_qc)
            self.__serial_robot.write_async(cmd)

            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(2, motor2_max_speed_in_rpm)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(2, motor2_acceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(2, motor2_deceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_target_relative_position_in_qc(2, motor2_relative_location_in_qc)
            self.__serial_robot.write_async(cmd)

            cmd = MotorCMD.MOTION_CONTROL.set_maximum_speed_in_rpm(3, motor3_max_speed_in_rpm)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_acceleration(3, motor3_acceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_deceleration(3, motor3_deceleration)
            self.__serial_robot.write_async(cmd)
            cmd = MotorCMD.MOTION_CONTROL.set_target_relative_position_in_qc(3, motor3_relative_location_in_qc)
            self.__serial_robot.write_async(cmd)

            self.sig_progress_update.emit(20)

            cmd = MotorCMD.MOTION_CONTROL.initiate_motion(0)
            self.__serial_robot.write_async(cmd)

            if not is_waiting_motion_complete:
                self.sig_progress_update.emit(60)
                self.sig_progress_finish.emit()
                result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA, None)
                return result_feedback
            else:
                time.sleep(0.2)
                for motor_index in range(1, 4):
                    is_success = self.__wait_until_position_attained(motor_index, 6)
                    if not is_success:
                        raise TimeoutError("Timeout(Motor Calibration): Target Position Not Attained!")
                    self.sig_progress_update.emit(20)
                time.sleep(time_limit / 2)

                self.sig_progress_finish.emit()

                result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA, None)
                return result_feedback
        except (serial.SerialException, TimeoutError) as e:
            self.sig_progress_interrupt.emit()
            result_feedback = self.__format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DECODE_ERROR,
                                                     str(e))
            return result_feedback

    def __move_by_velocity(self, new_vx, new_vy, new_vz):
        try:
            current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
            # Query current motor theta
            cmd = MotorCMD.QUERY.get_actual_position_in_qc(1)
            feedback = self.__serial_robot.query(cmd)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                current_motor1_theta_in_qc = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - \
                                             current_loaded_robot_setting.cs_motor1_datum_axis_pos
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))

            cmd = MotorCMD.QUERY.get_actual_position_in_qc(2)
            feedback = self.__serial_robot.query(cmd)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                current_motor2_theta_in_qc = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - \
                                             current_loaded_robot_setting.cs_motor2_datum_axis_pos
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))

            cmd = MotorCMD.QUERY.get_actual_position_in_qc(3)
            feedback = self.__serial_robot.query(cmd)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                current_motor3_theta_in_qc = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]) - \
                                             current_loaded_robot_setting.cs_motor3_datum_axis_pos
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))

            new_motor_omega_in_rpm = self.__robot_calculation_process. \
                calculate_angular_velocity_from_velocity(vx=new_vx, vy=new_vy, vz=new_vz,
                                                         motor1_theta_in_qc=current_motor1_theta_in_qc,
                                                         motor2_theta_in_qc=current_motor2_theta_in_qc,
                                                         motor3_theta_in_qc=current_motor3_theta_in_qc)

            motor1_omega_in_rpm = new_motor_omega_in_rpm[0]
            motor2_omega_in_rpm = new_motor_omega_in_rpm[1]
            motor3_omega_in_rpm = new_motor_omega_in_rpm[2]

            if motor1_omega_in_rpm and motor2_omega_in_rpm and motor3_omega_in_rpm:
                cmd = MotorCMD.MOTION_CONTROL.set_target_velocity_in_rpm(1, motor1_omega_in_rpm)
                self.__serial_robot.write_async(cmd)
                cmd = MotorCMD.MOTION_CONTROL.set_target_velocity_in_rpm(2, motor2_omega_in_rpm)
                self.__serial_robot.write_async(cmd)
                cmd = MotorCMD.MOTION_CONTROL.set_target_velocity_in_rpm(3, motor3_omega_in_rpm)
                self.__serial_robot.write_async(cmd)
                result_feedback = self.__format_feedback(True, const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_DATA, None)
                return result_feedback
            else:
                result_feedback = self.__format_feedback(False,
                                                         const.Serial.FeedbackMsgType.ROBOT_MSG_TYPE_CALCULATION_ERROR,
                                                         None)
                return result_feedback
        except serial.SerialException as e:
            result_feedback = self.__format_feedback(False, const.Serial.FeedbackMsgType.SERIAL_MSG_TYPE_DECODE_ERROR,
                                                     str(e))
            return result_feedback

    def __movement_correction(self, x0, y0):
        current_loaded_robot_setting: RobotSettingDataModel = self.__robot_setting_model.get_current_loaded_robot_setting()
        x_a = current_loaded_robot_setting.x_a
        x_b = current_loaded_robot_setting.x_b
        x_c = current_loaded_robot_setting.x_c
        y_a = current_loaded_robot_setting.y_a
        y_b = current_loaded_robot_setting.y_b
        y_c = current_loaded_robot_setting.y_c

        predict_error_x = x0 * x_a + y0 * x_b + x_c
        predict_error_y = x0 * y_a + y0 * y_b + y_c

        new_x0 = x0 - predict_error_x
        new_y0 = y0 - predict_error_y
        return new_x0, new_y0

    def __try_get_motor_actual_position_reading(self, motor_index, repeat=3):
        cmd = MotorCMD.QUERY.get_actual_position_in_qc(motor_index)
        for t in range(1, repeat+1):
            feedback = self.__serial_robot.query(cmd)
            if feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]:
                try:
                    result = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                    return result
                except ValueError:
                    print("Error: __try_get_motor_actual_position_reading fail")
                    print(f"Try {t} get motor {motor_index} position fails. original message: {feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]}")
                    if t == 3:
                        print(f"Total try: {repeat}, parse feedback result fail.")
                        raise serial.SerialException("parse feedback result fail")
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))

    def __wait_until_reaching_current_limit(self, motor_index, time_span):
        is_success = False
        time_elapsed = 0
        while (not is_success) and (time_elapsed < time_span):
            cmd = MotorCMD.QUERY.get_operation_status(motor_index)
            feedback = self.__serial_robot.query(cmd)
            if bool(feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]):
                result = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                if MotorCMD.CMD_PARSER_OST.is_continuous_current_limit_activated(result):
                    print("active current limit")
                    is_success = True
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))
            time.sleep(0.1)
            time_elapsed += 0.1
        if not is_success:
            print("Timeout: Current Limit Unreached")
        return is_success

    def __wait_until_position_attained(self, motor_index, time_span):
        is_success = False
        time_elapsed = 0
        while (not is_success) and (time_elapsed < time_span):
            cmd = MotorCMD.QUERY.get_operation_status(motor_index)
            feedback = self.__serial_robot.query(cmd)
            if bool(feedback[const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS]):
                result = int(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT])
                if MotorCMD.CMD_PARSER_OST.is_target_position_attained(result):
                    print("position attained")
                    is_success = True
            else:
                raise serial.SerialException(str(feedback[const.Serial.FeedbackKey.SERIAL_MSG_CONTENT]))
            time.sleep(0.1)
            time_elapsed += 0.1
        if not is_success:
            print("Timeout: Target Position Unreached")
        return is_success

    @staticmethod
    def __format_feedback(status, msg_type, msg):
        feedback = {const.Serial.FeedbackKey.SERIAL_COMMUNICATION_STATUS: status,
                    const.Serial.FeedbackKey.SERIAL_MSG_TYPE: msg_type,
                    const.Serial.FeedbackKey.SERIAL_MSG_CONTENT: msg}
        return feedback

    def calibration(self, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__calibration)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def move_by_position(self, new_x0, new_y0, new_z0, time_limit=1.0, is_waiting_movement_complete=False,
                         callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__move_by_position, new_x0, new_y0, new_z0, time_limit,
                                      is_waiting_movement_complete)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)

    def move_by_velocity(self, new_vx, new_vy, new_vz, callback=None, msg_to_pub=None, **kwargs):
        function_to_execute = partial(self.__move_by_velocity, new_vx, new_vy, new_vz)
        self.__async_wrapper.execute_on_single_thread(function_to_execute,
                                                      callback=callback, msg_to_pub=msg_to_pub, **kwargs)
