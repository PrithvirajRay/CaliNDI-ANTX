from functools import partial
from multiprocessing import Process, Lock, Queue

from six import with_metaclass

from ant_robot.core import utils
from ant_robot.core.gui.models.robot_setting_model import RobotSettingModel
from ant_robot.core.robot.robot_calculation import RobotCalculation


class RobotCalculationProcess(with_metaclass(utils.Singleton), object):
    def __init__(self):
        self.__f = None
        self.__e = None
        self.__rf = None
        self.__re = None
        self.__robot_setting_model = RobotSettingModel()

        self.__function_queue = Queue()
        self.__feedback_queue = Queue()
        self.__lock = Lock()

        self.__worker_process = Process(target=self._run, args=(self.__function_queue, self.__feedback_queue))
        self.__worker_process.daemon = True
        self.__worker_process.name = "RobotCalculationProcess"
        self.__worker_process.start()

    def __execute_on_process(self, function_to_execute):
        with self.__lock:
            self.__function_queue.put(function_to_execute)
            result = self.__feedback_queue.get()
            return result

    @staticmethod
    def _run(function_queue, feedback_queue):
        while True:
            try:
                function_to_execute = function_queue.get(block=True)
                result = function_to_execute()
                feedback_queue.put(result)
            except Exception as e:
                msg = "Calculation Process Error: {}".format(e)
                feedback_queue.put((None, msg))

    def __set_robot_specifications(self):
        current_loaded_robot_setting = self.__robot_setting_model.get_current_loaded_robot_setting()
        self.__f = current_loaded_robot_setting.f
        self.__e = current_loaded_robot_setting.e
        self.__rf = current_loaded_robot_setting.rf
        self.__re = current_loaded_robot_setting.re

    @staticmethod
    def _calculate_motor_theta_in_qc_from_coord(x0, y0, z0, f, e, rf, re):
        robot_calculation = RobotCalculation(f=f, e=e, rf=rf, re=re)
        status, theta = robot_calculation.inverse_kinematics(x0, y0, z0)
        if status[0] and status[1] and status[2]:
            motor1_theta_in_qc = utils.UnitConverter.radian2qc(theta[0])
            motor2_theta_in_qc = utils.UnitConverter.radian2qc(theta[1])
            motor3_theta_in_qc = utils.UnitConverter.radian2qc(theta[2])
            return True, motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc
        else:
            return False, "Calculation error: CalculateMotorThetaInQcFromCoord"

    def calculate_motor_theta_in_qc_from_coord(self, x0, y0, z0):
        self.__set_robot_specifications()
        function_to_execute = partial(self._calculate_motor_theta_in_qc_from_coord, x0, y0, z0,
                                      self.__f, self.__e, self.__rf, self.__re)
        result = self.__execute_on_process(function_to_execute)
        if result[0]:
            motor1_theta_in_qc = int(result[1])
            motor2_theta_in_qc = int(result[2])
            motor3_theta_in_qc = int(result[3])
            return motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc
        else:
            print(result[1])
            return None, None, None

    @staticmethod
    def _calculate_coord_from_motor_theta_in_qc(motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                                f, e, rf, re):
        robot_calculation = RobotCalculation(f=f, e=e, rf=rf, re=re)
        theta1_in_radian = utils.UnitConverter.qc2radian(motor1_theta_in_qc)
        theta2_in_radian = utils.UnitConverter.qc2radian(motor2_theta_in_qc)
        theta3_in_radian = utils.UnitConverter.qc2radian(motor3_theta_in_qc)
        status, coord = robot_calculation.forward_kinematics(theta1_in_radian, theta2_in_radian, theta3_in_radian)
        if status:
            x0 = coord[0]
            y0 = coord[1]
            z0 = coord[2]
            return True, x0, y0, z0
        else:
            return False, "Calculation error: CalculateCoordFromMotorThetaInQc"

    def calculate_coord_from_motor_theta_in_qc(self, motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc):
        self.__set_robot_specifications()
        function_to_execute = partial(self._calculate_coord_from_motor_theta_in_qc,
                                      motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                      self.__f, self.__e, self.__rf, self.__re)
        result = self.__execute_on_process(function_to_execute)
        if result[0]:
            x0 = float(result[1])
            y0 = float(result[2])
            z0 = float(result[3])
            return x0, y0, z0
        else:
            print(result[1])
            return None, None, None

    @staticmethod
    def _calculate_angular_velocity_from_velocity(vx, vy, vz,
                                                  motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                                  f, e, rf, re):
        robot_calculation = RobotCalculation(f=f, e=e, rf=rf, re=re)
        theta1_in_radian = utils.UnitConverter.qc2radian(motor1_theta_in_qc)
        theta2_in_radian = utils.UnitConverter.qc2radian(motor2_theta_in_qc)
        theta3_in_radian = utils.UnitConverter.qc2radian(motor3_theta_in_qc)
        status, angular_velocity = robot_calculation.inverse_velocity_kinematics(vx, vy, vz,
                                                                                 theta1_in_radian,
                                                                                 theta2_in_radian,
                                                                                 theta3_in_radian)
        if status:
            omega1_in_qc_per_s = utils.UnitConverter.radian2qc(angular_velocity[0])
            omega2_in_qc_per_s = utils.UnitConverter.radian2qc(angular_velocity[1])
            omega3_in_qc_per_s = utils.UnitConverter.radian2qc(angular_velocity[2])

            omega1_in_rpm = int(omega1_in_qc_per_s / 50)
            omega2_in_rpm = int(omega2_in_qc_per_s / 50)
            omega3_in_rpm = int(omega3_in_qc_per_s / 50)
            return True, omega1_in_rpm, omega2_in_rpm, omega3_in_rpm
        else:
            return False, "Calculation error: CalculateAngularVelocityFromVelocity"

    def calculate_angular_velocity_from_velocity(self, vx, vy, vz,
                                                 motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc):
        self.__set_robot_specifications()
        function_to_execute = partial(self._calculate_angular_velocity_from_velocity,
                                      vx, vy, vz,
                                      motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                      self.__f, self.__e, self.__rf, self.__re)
        result = self.__execute_on_process(function_to_execute)
        if result[0]:
            omega1_in_rpm = result[1]
            omega2_in_rpm = result[2]
            omega3_in_rpm = result[3]
            return omega1_in_rpm, omega2_in_rpm, omega3_in_rpm
        else:
            print(result[1])
            return None, None, None

    @staticmethod
    def _calculate_velocity_from_angular_velocity(omega1_in_rpm, omega2_in_rpm, omega3_in_rpm,
                                                  motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                                  f, e, rf, re):
        robot_calculation = RobotCalculation(f=f, e=e, rf=rf, re=re)
        theta1_in_radian = utils.UnitConverter.qc2radian(motor1_theta_in_qc)
        theta2_in_radian = utils.UnitConverter.qc2radian(motor2_theta_in_qc)
        theta3_in_radian = utils.UnitConverter.qc2radian(motor3_theta_in_qc)

        # 1 rpm = 3000 qc/min = 50 qc/s
        omega1_in_rad_per_s = utils.UnitConverter.qc2radian(omega1_in_rpm * 50)
        omega2_in_rad_per_s = utils.UnitConverter.qc2radian(omega2_in_rpm * 50)
        omega3_in_rad_per_s = utils.UnitConverter.qc2radian(omega3_in_rpm * 50)
        status, velocity = robot_calculation.forward_velocity_kinematics(omega1_in_rad_per_s,
                                                                         omega2_in_rad_per_s,
                                                                         omega3_in_rad_per_s,
                                                                         theta1_in_radian,
                                                                         theta2_in_radian,
                                                                         theta3_in_radian)
        if status:
            vx = velocity[0]
            vy = velocity[1]
            vz = velocity[2]
            return True, vx, vy, vz
        else:
            return False, "Calculation error: CalculateVelocityFromAngularVelocity"

    def calculate_velocity_from_angular_velocity(self, omega1_in_rpm, omega2_in_rpm, omega3_in_rpm,
                                                 motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc):
        self.__set_robot_specifications()
        function_to_execute = partial(self._calculate_velocity_from_angular_velocity,
                                      omega1_in_rpm, omega2_in_rpm, omega3_in_rpm,
                                      motor1_theta_in_qc, motor2_theta_in_qc, motor3_theta_in_qc,
                                      self.__f, self.__e, self.__rf, self.__re)
        result = self.__execute_on_process(function_to_execute)
        if result[0]:
            vx = result[1]
            vy = result[2]
            vz = result[3]
            return vx, vy, vz
        else:
            print(result[1])
            return None, None, None
