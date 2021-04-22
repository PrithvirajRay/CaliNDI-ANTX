import math

import numpy as np


class RobotCalculation:
    __TAN_30 = math.sqrt(3.0) / 3.0
    __SIN_120 = math.sqrt(3.0) / 2.0
    __COS_120 = -0.5
    __SIN_30 = 0.5
    __TAN_60 = math.sqrt(3.0)

    def __init__(self, f: float, e: float, rf: float, re: float):
        """
        Define specifications of a delta robot
        :param f: the edge length of fixed platform
        :param e: the edge length of end effector
        :param rf: the length of actuator near fixed platform
        :param re: the length of parallelograms near end effector
        """
        self.__f = f
        self.__e = e
        self.__rf = rf
        self.__re = re

    def set_robot_specifications(self, f, e, rf, re):
        if f:
            self.__f = f
        if e:
            self.__e = e
        if rf:
            self.__rf = rf
        if re:
            self.__re = re

    def inverse_kinematics(self, x0, y0, z0):
        """
        Given coordinate x0, y0 and z0, to find joint angle theta1, theta2 and theta3
        :param x0: target x position in mm
        :param y0: target y position in mm
        :param z0: target z position in mm
        :return: is theta existing and its value in radian
        """
        is_theta1_existing, theta1_in_radian = self.__calculate_theta_on_YZ_plane(x0, y0, z0)
        is_theta2_existing, theta2_in_radian = self.__calculate_theta_on_YZ_plane(
            x0 * self.__COS_120 + y0 * self.__SIN_120,
            y0 * self.__COS_120 - x0 * self.__SIN_120,
            z0)
        is_theta3_existing, theta3_in_radian = self.__calculate_theta_on_YZ_plane(
            x0 * self.__COS_120 - y0 * self.__SIN_120,
            y0 * self.__COS_120 + x0 * self.__SIN_120,
            z0)
        return (is_theta1_existing, is_theta2_existing, is_theta3_existing), \
               (theta1_in_radian, theta2_in_radian, theta3_in_radian)

    def __calculate_theta_on_YZ_plane(self, x0, y0, z0):
        """
        Point Definition:
        End effector center position: E0(x0, y0, z0)
        End effector joint position: E1(x_e1, y_e1, z_e1)
        Motor position: F1(x_f1, y_f1, z_f1)
        Linkage joint position: J1(x_j1, y_j1, z_j1)
        Projection of E1 on ZY plane: E1p(x_e1p, y_e1p, z_e1p)

        By triangulation calculation,
        x_e1 = x0, y_e1 = y0 - 0.5 * e * TAN_30, z_e1 = z0
        x_f1 = 0, y_f1 = - 0.5 * f * TAN_30, z_f1 = 0
        x_j1 = 0, y_j1 and z_j1 are variables to be calculated
        x_e1p = 0, y_e1p = y_e1, z_e1p = z_e1

        Two equations:
        distance(F1J1) = rf ------1
        distance(E1J1) = re ------2

        1 => (y_j1 - y_f1)^2 + (z_j1)^2 = rf^2
        2 => (x_e1)^2 + (y_j1 - y_e1)^2 + (z_j1 - z_e1)^2 = re^2

        Combine Eq. 1 and 2 and substitute corresponding value with x0, y0, z0, then one can get z_j1 = a + b * y_j1,
        where a = (x0^2 + z0^2 + y_e1^2 - y_f1^2 + rf^2 - re^2) / (2 * z0)
              b = (y_f1 - y_e1) / z0

        Substitute z_j1 = a + b * y_j1 into Eq. 1,
        (1 + b^2) * y_j1^2 + 2 * (a * b - y_f1) * y_j1 + y_f1^2 + a^2 - rf^2 = 0 ------3
        The determinant of this quadratic equation,
        d = -4 * [(a + b * y_f1)^2 + (1 + b^2) * rf^2]

        After calculating the roots Eq. 3, the larger one should be discarded and choose the smaller one,
        which laying outside the linkage circle
        y_j1 = (y_f1 - a * b - sqrt(d/4)) / (1 + b^2)
        z_j1 = a + b * y_j1
        theta = arctan(z_j1 / (y_j1 - y_f1))

        :param x0: target x position
        :param y0: target y position
        :param z0: target z position
        :return: is_theta_existing, theta
        """
        y_f1 = - 0.5 * self.__f * self.__TAN_30
        y_e1 = y0 - 0.5 * self.__e * self.__TAN_30

        a = (x0 ** 2 + z0 ** 2 + y_e1 ** 2 - y_f1 ** 2 + self.__rf ** 2 - self.__re ** 2) / (2 * z0)
        b = (y_f1 - y_e1) / z0
        d = 4 * (- (a + b * y_f1) ** 2 + (1 + b ** 2) * self.__rf ** 2)

        if d < 0:
            # No real root
            return False, 0
        else:
            y_j1 = (y_f1 - a * b - math.sqrt(d / 4)) / (1 + b ** 2)
            z_j1 = a + b * y_j1
            theta_in_radian = math.atan(z_j1 / (y_j1 - y_f1))
            return True, theta_in_radian

    def forward_kinematics(self, theta1_in_radian, theta2_in_radian, theta3_in_radian):
        """
        t is the shifting factor for each linkage joint
        Linkage joint 1: J1(x1, y1, z1)
        Linkage joint 2: J2(x2, y2, z2)
        Linkage joint 3: J3(x3, y3, z3)
        :param theta1_in_radian: theta 1 in radian
        :param theta2_in_radian: theta 2 in radian
        :param theta3_in_radian: theta 3 in radian
        :return:
        """
        t = 0.5 * (self.__f - self.__e) * self.__TAN_30

        x1 = 0
        y1 = -(t + self.__rf * math.cos(theta1_in_radian))
        z1 = -self.__rf * math.sin(theta1_in_radian)

        y2 = (t + self.__rf * math.cos(theta2_in_radian)) * self.__SIN_30
        x2 = y2 * self.__TAN_60
        z2 = -self.__rf * math.sin(theta2_in_radian)

        y3 = (t + self.__rf * math.cos(theta3_in_radian)) * self.__SIN_30
        x3 = -y3 * self.__TAN_60
        z3 = -self.__rf * math.sin(theta3_in_radian)

        common_denominator = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = x1 ** 2 + y1 ** 2 + z1 ** 2
        w2 = x2 ** 2 + y2 ** 2 + z2 ** 2
        w3 = x3 ** 2 + y3 ** 2 + z3 ** 2

        # x = (a1*z + b1) /common_denominator
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) * 0.5

        # y = (a2*z + b2) / common_denominator
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) * 0.5

        # a*z^2 + b*z + c = 0
        a = a1 ** 2 + a2 ** 2 + common_denominator ** 2
        b = 2 * (a1 * b1 + a2 * (b2 - y1 * common_denominator) - z1 * common_denominator ** 2)
        c = (b2 - y1 * common_denominator) ** 2 + b1 ** 2 + common_denominator ** 2 * (z1 ** 2 - self.__re ** 2)

        # discriminant
        d = b * b - 4.0 * a * c

        if d < 0:
            return False, (0, 0, 0)
        else:
            z0 = -0.5 * (b + math.sqrt(d)) / a
            x0 = (a1 * z0 + b1) / common_denominator
            y0 = (a2 * z0 + b2) / common_denominator
        return True, (x0, y0, z0)

    def inverse_velocity_kinematics(self, vx, vy, vz, theta1_in_radian, theta2_in_radian, theta3_in_radian):
        """
        Reference:
        https://www.researchgate.net/file.PostFileLoader.html?id=58bd7d14404854027f09f5d0&assetKey=AS%3A468931250003970%401488813330544

        The robot naming convention and end effector shape in this reference is a little different from the one used for
        inverse and forward kinematics provided above. Please go through the reference to fully understand the formulas
        in this function. The conversion is provided as following:
        w_b = sqrt(3) / 6 * rf
        s_p = 0.5 * re
        u_p = sqrt(3) / 6 * re
        w_p = sqrt(3) / 12 * re

        a = w_b - u_p
        b = s_p * 0.5 - sqrt(3) * 0.5 * w_b
        c = w_p - 0.5 * w_b
        =>
        a = sqrt(3) / 6 * (rf - re)
        b = 0.25 * (re - rf)
        c = -0.5 * a
        :param vx:
        :param vy:
        :param vz:
        :param theta1_in_radian:
        :param theta2_in_radian:
        :param theta3_in_radian:
        :return:
        """
        a = math.sqrt(3) / 6 * (self.__rf - self.__re)
        b = 0.25 * (self.__re - self.__rf)
        c = -0.5 * a
        status, (x, y, z) = self.forward_kinematics(theta1_in_radian, theta2_in_radian, theta3_in_radian)
        if not status:
            return False, (0, 0, 0)
        b_11 = self.__rf * ((y + a) * math.sin(theta1_in_radian) - z * math.cos(theta1_in_radian))
        b_22 = -self.__rf * ((math.sqrt(3) * (x + b) + y + c) * math.sin(theta2_in_radian) +
                             2 * z * math.cos(theta2_in_radian))
        b_33 = self.__rf * ((math.sqrt(3) * (x - b) - y - c) * math.sin(theta3_in_radian) -
                            2 * z * math.cos(theta3_in_radian))

        # Here use j_ii to represent the Jacobian matrix of the velocity in Cartesian System
        j_11 = x
        j_12 = y + a + self.__rf * math.cos(theta1_in_radian)
        j_13 = z + self.__rf * math.sin(theta1_in_radian)
        j_21 = 2 * (x + b) - math.sqrt(3) * self.__rf * math.cos(theta2_in_radian)
        j_22 = 2 * (y + c) - self.__rf * math.cos(theta2_in_radian)
        j_23 = 2 * (z + self.__rf * math.sin(theta2_in_radian))
        j_31 = 2 * (x - b) + math.sqrt(3) * self.__rf * math.cos(theta3_in_radian)
        j_32 = 2 * (y + c) - self.__rf * math.cos(theta3_in_radian)
        j_33 = 2 * (z + self.__rf * math.sin(theta3_in_radian))

        omega1_in_rad_per_second = (j_11 * vx + j_12 * vy + j_13 * vz) / b_11
        omega2_in_rad_per_second = (j_21 * vx + j_22 * vy + j_23 * vz) / b_22
        omega3_in_rad_per_second = (j_31 * vx + j_32 * vy + j_33 * vz) / b_33
        return True, (omega1_in_rad_per_second, omega2_in_rad_per_second, omega3_in_rad_per_second)

    def forward_velocity_kinematics(self, omega1_in_rad_per_s, omega2_in_rad_per_s, omega3_in_rad_per_s,
                                    theta1_in_radian, theta2_in_radian, theta3_in_radian):
        a = math.sqrt(3) / 6 * (self.__rf - self.__re)
        b = 0.25 * (self.__re - self.__rf)
        c = -0.5 * a
        status, (x, y, z) = self.forward_kinematics(theta1_in_radian, theta2_in_radian, theta3_in_radian)
        if not status:
            return False, (0, 0, 0)
        b_11 = self.__rf * ((y + a) * math.sin(theta1_in_radian) - z * math.cos(theta1_in_radian))
        b_22 = -self.__rf * ((math.sqrt(3) * (x + b) + y + c) * math.sin(theta2_in_radian) +
                             2 * z * math.cos(theta2_in_radian))
        b_33 = self.__rf * ((math.sqrt(3) * (x - b) - y - c) * math.sin(theta3_in_radian) -
                            2 * z * math.cos(theta3_in_radian))

        # Here use j_ii to represent the Jacobian matrix of the velocity in Cartesian System
        j_11 = x
        j_12 = y + a + self.__rf * math.cos(theta1_in_radian)
        j_13 = z + self.__rf * math.sin(theta1_in_radian)
        j_21 = 2 * (x + b) - math.sqrt(3) * self.__rf * math.cos(theta2_in_radian)
        j_22 = 2 * (y + c) - self.__rf * math.cos(theta2_in_radian)
        j_23 = 2 * (z + self.__rf * math.sin(theta2_in_radian))
        j_31 = 2 * (x - b) + math.sqrt(3) * self.__rf * math.cos(theta3_in_radian)
        j_32 = 2 * (y + c) - self.__rf * math.cos(theta3_in_radian)
        j_33 = 2 * (z + self.__rf * math.sin(theta3_in_radian))

        J = np.array([[j_11, j_12, j_13], [j_21, j_22, j_23], [j_31, j_32, j_33]])
        B = np.array([b_11 * omega1_in_rad_per_s, b_22 * omega2_in_rad_per_s, b_33 * omega3_in_rad_per_s])
        result = np.linalg.solve(J, B)
        vx = result[0]
        vy = result[1]
        vz = result[2]
        return True, (vx, vy, vz)
