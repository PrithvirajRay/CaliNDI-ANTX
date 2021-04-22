class MotionControl:
    """
    motor_index = 0 means sending cmd to all motor
    """

    @staticmethod
    def enable_motor(motor_index=0):
        return "{}EN".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def disable_motor(motor_index=0):
        return "{}DI".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def initiate_motion(motor_index=0):
        return "{}M".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def set_maximum_speed_in_rpm(motor_index=0, speed=0):
        return "{}SP{}".format("" if motor_index == 0 else motor_index, speed)

    @staticmethod
    def set_acceleration(motor_index=0, acceleration=0):
        return "{}AC{}".format("" if motor_index == 0 else motor_index, acceleration)

    @staticmethod
    def set_deceleration(motor_index=0, deceleration=0):
        return "{}DEC{}".format("" if motor_index == 0 else motor_index, deceleration)

    @staticmethod
    def set_peak_current_limit_in_mA(motor_index=0, current_limit=0):
        return "{}LPC{}".format("" if motor_index == 0 else motor_index, current_limit)

    @staticmethod
    def set_continuous_current_limit_in_mA(motor_index=0, current_limit=0):
        return "{}LCC{}".format("" if motor_index == 0 else motor_index, current_limit)

    @staticmethod
    def set_current_integral_term(motor_index=0, integral=1):
        return "{}CI{}".format("" if motor_index == 0 else motor_index, integral)

    @staticmethod
    def set_target_absolute_position_in_qc(motor_index=0, absolute_position=0):
        return "{}LA{}".format("" if motor_index == 0 else motor_index, absolute_position)

    @staticmethod
    def set_target_relative_position_in_qc(motor_index=0, relative_position=0):
        return "{}LR{}".format("" if motor_index == 0 else motor_index, relative_position)

    @staticmethod
    def set_position_corridor_in_qc(motor_index=0, corridor=20):
        return "{}CORRIDOR{}".format("" if motor_index == 0 else motor_index, corridor)

    @staticmethod
    def set_home_position_in_qc(motor_index=0, absolute_position=""):
        return "{}HO{}".format("" if motor_index == 0 else motor_index, absolute_position)

    @staticmethod
    def set_position_limit_in_qc(motor_index=0, absolute_position=""):
        return "{}LL{}".format("" if motor_index == 0 else motor_index, absolute_position)

    @staticmethod
    def enable_position_limit(motor_index=0):
        return "{}APL1".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def disable_position_limit(motor_index=0):
        return "{}APL0".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def set_notify_absolute_position_in_qc(motor_index=0, absolute_position=""):
        return "{}NP{}".format("" if motor_index == 0 else motor_index, absolute_position)

    @staticmethod
    def clear_position_notification(motor_index=0):
        return "{}NPOFF".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def set_position_proportional_term(motor_index=0, amplification=1):
        return "{}PP{}".format("" if motor_index == 0 else motor_index, amplification)

    @staticmethod
    def set_position_differential_term(motor_index=0, d_term=1):
        return "{}PD{}".format("" if motor_index == 0 else motor_index, d_term)

    @staticmethod
    def set_target_velocity_in_rpm(motor_index=0, velocity=0):
        return "{}V{}".format("" if motor_index == 0 else motor_index, velocity)

    @staticmethod
    def set_notify_velocity_in_rpm(motor_index=0, velocity=0):
        return "{}NV{}".format("" if motor_index == 0 else motor_index, velocity)

    @staticmethod
    def clear_velocity_notification(motor_index=0):
        return "{}NVOFF".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def set_velocity_proportional_term(motor_index=0, amplification=1):
        return "{}POR{}".format("" if motor_index == 0 else motor_index, amplification)

    @staticmethod
    def set_velocity_integral_term(motor_index=0, integral=1):
        return "{}I{}".format("" if motor_index == 0 else motor_index, integral)

    @staticmethod
    def set_velocity_sampling_rate(motor_index=0, sampling_rate=1):
        return "{}SR{}".format("" if motor_index == 0 else motor_index, sampling_rate)

    @staticmethod
    def set_velocity_deviation_in_rpm(motor_index=0, deviation=10):
        return "{}DEV{}".format("" if motor_index == 0 else motor_index, deviation)


class Query:
    @staticmethod
    def get_configuration_status(motor_index):
        return "{}CST".format(motor_index)

    @staticmethod
    def get_controller_type(motor_index):
        return "{}GTYP".format(motor_index)

    @staticmethod
    def get_motor_serial_number(motor_index):
        return "{}GSER".format(motor_index)

    @staticmethod
    def get_actual_position_in_qc(motor_index):
        return "{}POS".format(motor_index)

    @staticmethod
    def get_target_position_in_qc(motor_index):
        return "{}TPOS".format(motor_index)

    @staticmethod
    def get_actual_velocity_in_rpm(motor_index):
        return "{}GN".format(motor_index)

    @staticmethod
    def get_target_velocity_in_rpm(motor_index):
        return "{}GV".format(motor_index)

    @staticmethod
    def get_actual_current_in_mA(motor_index):
        return "{}GRC".format(motor_index)

    @staticmethod
    def get_operation_status(motor_index):
        return "{}OST".format(motor_index)

    @staticmethod
    def get_position_limit_positive_in_qc(motor_index=0):
        return "{}GPL".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_position_limit_negative_in_qc(motor_index=0):
        return "{}GNL".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_maximum_speed_in_rpm(motor_index=0):
        return "{}GSP".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_acceleration(motor_index=0):
        return "{}GAC".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_deceleration(motor_index=0):
        return "{}GDEC".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_velocity_sampling_rate(motor_index=0):
        return "{}GSR".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_velocity_proportional_term(motor_index=0):
        return "{}GPOR".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_velocity_integral_term(motor_index=0):
        return "{}GI".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_position_proportional_term(motor_index=0):
        return "{}GPP".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_position_differential_term(motor_index=0):
        return "{}GPD".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_peak_current_limit_in_mA(motor_index=0):
        return "{}GPC".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_continuous_current_limit_in_mA(motor_index=0):
        return "{}GCC".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_current_integral_term(motor_index=0):
        return "{}GCI".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_velocity_deviation_in_rpm(motor_index=0):
        return "{}GDEV".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_position_corridor_in_qc(motor_index=0):
        return "{}GCORRIDOR".format("" if motor_index == 0 else motor_index)

    @staticmethod
    def get_temperature_in_celsius(motor_index=0):
        return "{}TEM".format("" if motor_index == 0 else motor_index)


class Configuration:
    """Be careful to use these commands"""

    @staticmethod
    def enable_command_acknowledgement():
        return "ANSW2"

    @staticmethod
    def disable_command_acknowledgement():
        return "ANSW0"

    @staticmethod
    def set_controller_baudrate(baudrate=9600):
        return "BAUD{}".format(baudrate)


class CMDParserOST:
    """Parsing "OST" command, please check FaulHaber Serial Communication Manual for full description"""

    # TODO: check if bit 4 is testing continuous current limit or peak current limit
    @staticmethod
    def is_continuous_current_limit_activated(result):
        result = int(result)
        if result & pow(2, 4) == 0:
            return False
        else:
            return True

    @staticmethod
    def is_target_position_attained(result):
        result = int(result)
        if result & pow(2, 16) == 0:
            return False
        else:
            return True


class CMDParserCST:
    """Parsing "CST" command, please check FaulHaber Serial Communication Manual for full description"""

    @staticmethod
    def is_cmd_acknowledgement_enabled(result: int) -> bool:
        if result & pow(2, 2) == 0:
            return False
        else:
            return True

    @staticmethod
    def is_motor_power_enabled(result: int) -> bool:
        if result & pow(2, 10) == 0:
            return False
        else:
            return True

    @staticmethod
    def is_position_limit_enabled(result: int) -> bool:
        if result & pow(2, 13) == 0:
            return False
        else:
            return True


class MotorCMD:
    MOTION_CONTROL = MotionControl()
    QUERY = Query()
    CONFIGURATION = Configuration()
    CMD_PARSER_OST = CMDParserOST()
    CMD_PARSER_CST = CMDParserCST()


if __name__ == '__main__':
    import inspect

    ALL_MOTION_CONTROL_CMD = [m[0] for m in inspect.getmembers(MotorCMD.MOTION_CONTROL, inspect.isfunction)]
    ALL_QUERY_CMD = [m[0] for m in inspect.getmembers(MotorCMD.QUERY, inspect.isfunction)]
    ALL_CONFIGURATION_CMD = [m[0] for m in inspect.getmembers(MotorCMD.CONFIGURATION, inspect.isfunction)]

    print("\nMotion control cmd:\n")
    for cmd in ALL_MOTION_CONTROL_CMD:
        print(cmd)
    print("\nQuery cmd:\n")
    for cmd in ALL_QUERY_CMD:
        print(cmd)
    print("\nConfiguration cmd:\n")
    for cmd in ALL_CONFIGURATION_CMD:
        print(cmd)
