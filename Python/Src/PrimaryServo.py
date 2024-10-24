class Primary_Instruction:
    """
    This class stores the servo command types.
    """

    # Used to query the servo. The servo will return the present status and the model number of the servo.
    PING = 0x01

    # Read data from the servo's memory table
    READ_DATA = 0x02

    # Write data on the Servo Memory Table.
    WRITE_DATA = 0x03

    # A single Sync Write Instruction allows data to be written in memory tables of multiple servos simultaneously.
    SYNC_WRITE = 0X83

    # Reset the entire memory table of the servo to the factory default values. 
    FACTORY_RESET = 0x06

    # Reset the parameter settings in the servo's memory table to the factory default values.
    PARAMETER_RESET = 0x10

    # Calibrate the offset value of the servo midpoint (150 ° position).
    CALIBRATION = 0x15

    # Used to reboot servo.
    REBOOT = 0x64


class Primary_State:
    """
    This class stores the servo error statuses.
    """

    # No errors occurred
    SUCCESS = 0

    # Input voltage of the servo is out of range for the operating voltage set in the memory table.
    VOLTAGE_ERROR = 0x01

    # Target position of the servo is out of range for the maximum and minimum target positions set in the memory table.
    ANGLE_ERROR = 0x02

    # Internal temperature of the servo is over the upper limit set in the Memory Table.
    OVERHEATING_ERROR = 0x04

    # Length of Instruction Packet out of range for the specified length.
    RANGE_ERROR = 0x08

    # CheckSum of the Instruction Packet is parsed incorrectly.
    CHECKSUM_ERROR = 0x10

    # Servo occurs in a stall due to overcurrent or overload etc.
    STALL_ERROR = 0x20

    # Instruction Packet does not comply with the protocol format, resulting in a parsing error.
    PARSING_ERROR = 0x40

    # This is not a complete response package
    UNPACK_ERROR = 0x80

class Primary_Servo_Sync_Parameter:
    """
    This class stores the parameters for the sync write commands.
    """

    def __init__(self):
        self.id_counts = 0                    # Number of servos for sync write.
        self.id = [0] * 20                    # Ids of servos for sync write.id
        self.torque_switch = [0] * 20         # Torque Switch
        self.control_mode = [0] * 20          # Control Mode
        self.position = [0] * 20              # Target Position
        self.time = [0] * 20                  # Time Base Run Time
        self.velocity = [0] * 20              # Target Velocity
        self.acc_velocity = [0] * 20          # Target ACC
        self.dec_velocity = [0] * 20          # Target DEC
        self.acc_velocity_grade = [0] * 20    # Time Base Target ACC


class Primary_Address:
    """
    This class stores the primary memory table addresses of the servo.
    """

    # S/N Code
    MODEL_NUMBER = 0x00

    # S/N Code
    FIRMWARE_VERSION = 0x02

    # S/N Code
    MODEL_INFORMATION = 0x03

    # Servo ID
    SERVO_ID = 0x07

    # Baud Rate Number 7 is 1Mbps
    BAUD_RATE = 0x08

    # Response Delay Time for servo return to packet
    RETURN_DELAY_TIME = 0x09

    # Returned level of servo status.
    RETURN_LEVEL = 0x0A

    # Min Angle Limit for servo rotation
    MIN_ANGLE_LIMIT_L = 0x0B

    # Max Voltage Limit for operating Servo
    MAX_ANGLE_LIMIT_L = 0x0D

    # Maximum Temperature Limit for operating servo
    MAX_TEMPERATURE_LIMIT = 0x0F

    # Maximum Voltage Limit for operating Servo
    MAX_VOLTAGE_LIMIT = 0x10

    # Minimum Voltage Limit for operating Servo
    MIN_VOLTAGE_LIMIT = 0x11

    # Max PWM Limit Limit of servo
    MAX_PWM_LIMIT_L = 0x12

    # Max Current Limit for operating servo
    MAX_CURRENT_LIMIT_L = 0x14

    # Trigger Time for overload protection activation after reaching current limit
    CURRENT_TIME_L = 0x16

    # Dead Band for clockwise direction
    CW_DEADBAND = 0x18

    # Dead Band for counterclockwise direction
    CCW_DEADBAND = 0x19

    # PWM punch of the servo output.
    PWM_PUNCH = 0x1A

    # Gain Proportion (P) of Servo's PID Control
    POSITION_P_L = 0x1B

    # Gain Integration (I) of Servo's PID Control
    POSITION_I_L = 0x1D

    # Gain Differential (D) of Servo's PID Control
    POSITION_D_L = 0x1F

    # Conditions for Alarm LED
    LED_CONDITION = 0x21

    # Conditions for torque unloading
    SHUTDOWN_CONDITION = 0x22

    # Servo Control Mode: 0 Time Base Position Control, 1 Velocity Base Position Control, 2 Current, 3 PWM
    CONTROL_MODE = 0x23

    # Offset Value for midpoint Calibration of Servo.This bit is triggered by the Calibration instruction, and the value cannot be directly written.
    CALIBRATION_L = 0x24

    # Flash area write switch: 0 for write disabled, 1 for write enabled.
    FLASH_SW = 0x2E

    # Servo indicator light switch: 0 for off, 1 for on.
    LED_SW = 0x2F

    # Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode.
    TORQUE_SW = 0x30

    # Direct control of PWM output to the motor
    TARGET_PWM_L = 0x31

    # Target current for servo operation, by default, equal to the default value of the max current limit.
    TARGET_CURRENT_L = 0x33

    # Used in Velocity Base Position Control Mode.,Plan Profile Position
    VELOCITY_BASE_TARGET_POSITION_L = 0x35

    # Used in Velocity Base Position Control Mode.,Plan Profile Velocity
    VELOCITY_BASE_TARGET_VELOCITY_L = 0x37

    # Used in Velocity Base Position Control Mode.,Plan Profile Acceleration
    VELOCITY_BASE_TARGET_ACC = 0x39

    # Used in Velocity Base Position Control Mode.,Plan Profile Deceleration
    VELOCITY_BASE_TARGET_DEC = 0x3A

    # Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
    TIME_BASE_TARGET_ACC = 0x3B

    # Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
    TIME_BASE_TARGET_POSITION_L = 0x3C

    # Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
    TIME_BASE_TARGET_MOVINGTIME_L = 0x3E

    # Actual voltage at which the servo is currently operating.
    PRESENT_VOLTAGE = 0x40

    # Actual internal temperature of the servo.
    PRESENT_TEMPERATURE = 0x41

    # Present PWM value being output by the servo.
    PRESENT_PWM_L = 0x42

    # Present profile velocity of the Profile Planner.
    PRESENT_PROFILE_VELOCITY_L = 0x44

    # Present profile position of the Profile Planner.
    PRESENT_PROFILE_POSITION_L = 0x46

    # Present actual velocity of the Servo
    PRESENT_VELOCITY_L = 0x48

    # Present actual position of the Servo
    PRESENT_POSITION_L = 0x4A

    # Present actual current of the Servo
    PRESENT_CURRENT_L = 0x4C


class Primary_Servo:
    """
    This class contains all the methods for operating the servo.
    """

    @staticmethod
    def get_check(buffer: list, length: int) -> int:
        """
        Calculate the checksum of the instruction packet.
        :param buffer: Pointer for the buffer to calculate the checksum.
        :param length: Buffer Length.
        :return: Calculated Checksum.
        """

        total = 0
        for i in range(length):
            total += buffer[i]
        total = ~total & 0xFF
        return total

    @staticmethod
    def servo_pack(servo_id: int, instruction: int, address: int, byte_length: int, input_buffer: list,
                   output_buffer: list, output_length: list) -> int:
        """
        Package servo control command data.
        :param servo_id: ServoID.
        :param instruction: Servo instruction.
        :param address: The address to operate on.
        :param byte_length: Byte length.
        :param input_buffer: Instruction package parameter data.
        :param output_buffer: Pointer for the buffer that is used to store the generated instruction packet.
        :param output_length: Buffer Length.
        :return: success or error flag.
        """

        i = 0
        output_buffer[i] = 0xff
        i += 1
        output_buffer[i] = 0xff
        i += 1
        output_buffer[i] = servo_id
        i += 1

        if instruction == Primary_Instruction.PING:
            output_buffer[i] = 0x02
            i += 1
            output_buffer[i] = instruction
            i += 1
        elif instruction == Primary_Instruction.READ_DATA:
            output_buffer[i] = 0x04
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = address
            i += 1
            output_buffer[i] = byte_length
            i += 1
        elif instruction == Primary_Instruction.WRITE_DATA:
            output_buffer[i] = byte_length + 3
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = address
            i += 1
            for j in range(byte_length):
                output_buffer[i] = input_buffer[j]
                i += 1
        elif instruction == Primary_Instruction.SYNC_WRITE:
            output_buffer[i] = (input_buffer[1] + 1) * byte_length + 4
            i += 1
            output_buffer[i] = instruction
            i += 1
            for j in range((byte_length * input_buffer[1]) + 2 + byte_length):
                output_buffer[i] = input_buffer[j]
                i += 1
        elif instruction in [Primary_Instruction.FACTORY_RESET, Primary_Instruction.PARAMETER_RESET, Primary_Instruction.CALIBRATION,
                             Primary_Instruction.REBOOT]:
            output_buffer[i] = 0x04
            i += 1
            output_buffer[i] = instruction
            i += 1
            output_buffer[i] = 0xdf
            i += 1
            output_buffer[i] = 0xdf
            i += 1

        output_buffer[i] = Primary_Servo.get_check(output_buffer[2:], i - 2) & 0xff
        output_length[0] = i + 1
        return Primary_State.SUCCESS

    @staticmethod
    def servo_unpack(response_packet: list, data_buffer: list) -> int:
        """
        Parsing reply packet.
        :param response_packet: packet data of the steering gear.
        :param data_buffer: Data parsed from the status packet.
        :return: success or error flag.
        """

        length = response_packet[3]
        status = response_packet[4]

        checksum = Primary_Servo.get_check(response_packet[2:], length + 1)

        if response_packet[0] != 0xff or response_packet[1] != 0xff or checksum != response_packet[length + 3]:
            print("This is not a complete response package!")
            return Primary_State.UNPACK_ERROR

        if status != 0x00:
            if status & Primary_State.VOLTAGE_ERROR == Primary_State.VOLTAGE_ERROR:
                print("Voltage Error")
            if status & Primary_State.ANGLE_ERROR == Primary_State.ANGLE_ERROR:
                print("Angle Error")
            if status & Primary_State.OVERHEATING_ERROR == Primary_State.OVERHEATING_ERROR:
                print("Overheating Error")
            if status & Primary_State.RANGE_ERROR == Primary_State.RANGE_ERROR:
                print("Range Error")
            if status & Primary_State.CHECKSUM_ERROR == Primary_State.CHECKSUM_ERROR:
                print("CheckSum Error")
            if status & Primary_State.STALL_ERROR == Primary_State.STALL_ERROR:
                print("Stall Error")
            if status & Primary_State.PARSING_ERROR == Primary_State.PARSING_ERROR:
                print("Parsing Error")
            return status

        if length > 2:
            data_buffer[0] = response_packet[5]
            data_buffer[1] = response_packet[6]

        return Primary_State.SUCCESS

    @staticmethod
    def sync_write_data(address: int, servo_counts: int, input_buffer: list, output_buffer: list,
                        output_buffer_len: list) -> int:
        """
        Generate sync write instructions to write memory table data.
        :param address: The address of the memory table to write data to.
        :param servo_counts: The number of servos operated by sync write instructions.
        :param input_buffer: Written data.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(0xfe, Primary_Instruction.SYNC_WRITE, address, servo_counts, input_buffer, output_buffer,
                         output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_write(servo_id: int, address: int, write_data_len: int, input_buffer: list, output_buffer: list,
                    output_buffer_len: list) -> int:
        """
        Generate instructions to write memory table data.
        :param servo_id: ServoID.
        :param address: The address of the memory table to write data to.
        :param write_data_len: The length of the data to be written.
        :param input_buffer: Written data.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.WRITE_DATA, address, write_data_len, input_buffer, output_buffer,
                         output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read(servo_id: int, address: int, read_data_len: int, output_buffer: list,
                   output_buffer_len: list) -> int:
        """
        Generate instructions to read memory table data.
        :param servo_id: ServoID.
        :param address: The address of the memory table to read.
        :param read_data_len: The size of the bytes to read.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.READ_DATA, address, read_data_len, None, output_buffer,
                         output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_ping(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Generate the PING instruction package.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.PING, 0, 0, None, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_factory_reset(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Reset the servo to the factory default values.
        :param servo_id: ServoID. ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.FACTORY_RESET, 0, 0, None, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_parameter_reset(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Reset the parameter settings of the servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.PARAMETER_RESET, 0, 0, None, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_calibration(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Calibrate the midpoint of the servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.CALIBRATION, 0, 0, None, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_reboot(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Reboot the servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_pack(servo_id, Primary_Instruction.REBOOT, 0, 0, None, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_position_and_present_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present position and present current of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_POSITION_L, 4, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present current of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_CURRENT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present position of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_POSITION_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present velocity of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_profile_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present profile position of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_PROFILE_POSITION_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_profile_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present profile velocity of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_PROFILE_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_pwm(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present PWM of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """

        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_PWM_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_temperature(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present temperature of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_TEMPERATURE, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_voltage(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the present voltage of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.PRESENT_VOLTAGE, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_moving_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the time base target moving time of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TIME_BASE_TARGET_MOVINGTIME_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the time base target position of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TIME_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_acc(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the time base target ACC of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TIME_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_dec(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the velocity base target DEC of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.VELOCITY_BASE_TARGET_DEC, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_acc(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the velocity base target ACC of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.VELOCITY_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_velocity(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the velocity base target velocity of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.VELOCITY_BASE_TARGET_VELOCITY_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_position(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the velocity base target position of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.VELOCITY_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_target_current(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the target current of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TARGET_CURRENT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_target_pwm(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the target PWM of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TARGET_PWM_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_torque_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the torque switch of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.TORQUE_SW, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_led_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the LED switch of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.LED_SW, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_flash_switch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the Flash switch of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.FLASH_SW, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_calibration(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the calibration of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.CALIBRATION_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_control_mode(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the control mode of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.CONTROL_MODE, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_shutdown_condition(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the shutdown condition of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.SHUTDOWN_CONDITION, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_led_condition(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the LED condition of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.LED_CONDITION, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_d_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the position control D gain of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.POSITION_D_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_i_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the position control I gain of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.POSITION_I_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_p_gain(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the position control P gain of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.POSITION_P_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_pwm_punch(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the PWM punch of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.PWM_PUNCH, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_ccw_deadband(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the CCW deadband of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.CCW_DEADBAND, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_cw_deadband(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the CW deadband of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.CW_DEADBAND, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_current_shutdown_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the current shutdown time of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.CURRENT_TIME_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_current_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the max current limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MAX_CURRENT_LIMIT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_pwm_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the max PWM limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MAX_PWM_LIMIT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_voltage_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the max voltage limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MAX_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_min_voltage_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the min voltage limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MIN_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_temperature_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the max temperature limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MAX_TEMPERATURE_LIMIT, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_angle_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the max angle limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MAX_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_min_angle_limit(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the min angle limit of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MIN_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_return_level(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the return level of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.RETURN_LEVEL, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_return_delay_time(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the return delay time of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.RETURN_DELAY_TIME, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_baud_rate(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the baud rate of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.BAUD_RATE, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_model_information(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the model information of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.MODEL_INFORMATION, 4, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_read_firmware_version(servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Read the firmware version of servo.
        :param servo_id: ServoID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        Primary_Servo.servo_read(servo_id, Primary_Address.FIRMWARE_VERSION, 1, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_ping_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the PING command.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Ping Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_factory_reset_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet for the factory reset command.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = None
        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Factory Reset Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_parameter_reset_analysis(response_packet) -> int:
        """
        Parsing the servo response packet for the parameter reset command.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = None
        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Parameter Reset Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_calibration_analysis(response_packet) -> int:
        """
        Parsing the servo response packet for the calibration command.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = None
        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Calibration Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_position_and_present_current_analysis(response_packet, position, current) -> int:
        """
        Parsing the servo response packet for the present position and current.
        :param response_packet: Servo response packet.Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 4

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            position[0] = (data_buffer[1] << 8) | data_buffer[0]
            current[0] = (data_buffer[3] << 8) | data_buffer[2]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_current_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present current.
        :param response_packet: Servo response packet.Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_position_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present position.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_velocity_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present velocity.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_profile_position_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present profile position.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_profile_velocity_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present profile velocity.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_pwm_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present pwm.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)

        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_temperature_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present temperature.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_present_voltage_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the present voltage.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_moving_time_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the time base target moving time.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_position_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the time base target position.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_time_base_target_acc_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the time base target accelerated speed.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_dec_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the velocity base target deceleration.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_acc_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the velocity base target accelerated speed.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_velocity_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the velocity base target velocity.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_velocity_base_target_position_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the velocity base target position.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_target_current_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the target current.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_target_pwm_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the target pwm.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_torque_switch_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the torque switch.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_led_switch_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the led switch.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_flash_switch_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the flash switch.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_current_offset_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the current offset.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_calibration_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the calibration.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)

        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_control_mode_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the control mode.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_shutdown_condition_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the shutdown condition.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_led_condition_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the led condition.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_d_gain_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the position control d gain.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_i_gain_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the position control i gain.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_position_control_p_gain_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the position control p gain.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_pwm_punch_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the pwm punch.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_ccw_deadband_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the ccw deadband.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_cw_deadband_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the cw deadband.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_current_shutdown_time_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the current shutdown time.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_current_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the max current limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_pwm_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the max pwm limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_voltage_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the max voltage limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_min_voltage_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the min voltage limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_temperature_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the max temperature limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_max_angle_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the max angle limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_min_angle_limit_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the min angle limit.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)

        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_return_level_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the return level.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_return_delay_time_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the return delay time.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_baud_rate_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the baud rate.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_model_information_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the model information.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 4

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = (data_buffer[3] << 24) | (data_buffer[2] << 16) | (data_buffer[1] << 8) | data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_read_firmware_version_analysis(response_packet, analysis_data) -> int:
        """
        Parsing the servo response packet for the firmware version.
        :param response_packet: Servo response packet.
        :param analysis_data: The data parsed from the servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Read Successful!")
            analysis_data[0] = data_buffer[0]
            return Primary_State.SUCCESS

    @staticmethod
    def servo_modify_known_id(servo_id: int, new_servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Change the servo ID.
        :param servo_id: ServoID.
        :param new_servo_id: The modified ID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = new_servo_id
        Primary_Servo.servo_write(servo_id, Primary_Address.SERVO_ID, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_modify_unknown_id(new_servo_id: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the servo ID of the servo with an unknown ID.
        :param new_servo_id: The modified ID.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = new_servo_id
        Primary_Servo.servo_write(0xfe, Primary_Address.SERVO_ID, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_return_delay_time(servo_id: int, response_delay_time: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        Set the return delay time of the servo.
        :param servo_id: ServoID.
        :param response_delay_time: Response Delay Time for servo return to packet，Value range is:0~255，unit is 2μs.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = response_delay_time
        Primary_Servo.servo_write(servo_id, Primary_Address.RETURN_DELAY_TIME, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_return_level(servo_id: int, return_level: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the return level of servo.
        :param servo_id: ServoID.
        :param return_level: Returned level of servo status，Value range is:0 1 2.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = return_level
        Primary_Servo.servo_write(servo_id, Primary_Address.RETURN_LEVEL, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_baud_rate(servo_id: int, baud_rate_number: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the baud rate of servo.
        :param servo_id: ServoID.
        :param baud_rate_number: Baud Rate Number 7 is 1Mbps，Value range is:1~7.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = baud_rate_number
        Primary_Servo.servo_write(servo_id, Primary_Address.BAUD_RATE, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_min_angle_limit(servo_id: int, min_angle_limit: int, output_buffer: list,
                                  output_buffer_len: list) -> int:
        """
        Set the min angle limit of servo.
        :param servo_id: ServoID.
        :param min_angle_limit: Min Angle Limit for servo rotation，Value range is:0~3000，unit is 0.1°.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = min_angle_limit & 0xff
        buffer[1] = (min_angle_limit >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.MIN_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_angle_limit(servo_id: int, max_angle_limit: int, output_buffer: list,
                                  output_buffer_len: list) -> int:
        """
        Set the max angle limit of servo.
        :param servo_id: ServoID.
        :param max_angle_limit: Max Angle Limit for servo rotation，Value range is:0~3000，unit is 0.1°.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = max_angle_limit & 0xff
        buffer[1] = (max_angle_limit >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.MAX_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_temperature_limit(servo_id: int, max_temperature_limit: int, output_buffer: list,
                                        output_buffer_len: list) -> int:
        """
        Set the max temperature limit of servo.
        :param servo_id: ServoID.
        :param max_temperature_limit: Max Temperature Limit for operating servo，Value range is:0~127，unit is 1℃.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = max_temperature_limit
        Primary_Servo.servo_write(servo_id, Primary_Address.MAX_TEMPERATURE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_voltage_limit(servo_id: int, max_voltage_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        Set the max voltage limit of servo.
        :param servo_id: ServoID.
        :param max_voltage_limit: Max Voltage Limit for operating Servo，Value range is:33~90，unit is 0.1V.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = max_voltage_limit
        Primary_Servo.servo_write(servo_id, Primary_Address.MAX_VOLTAGE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_min_voltage_limit(servo_id: int, min_voltage_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        Set the min voltage limit of servo.
        :param servo_id: ServoID.
        :param min_voltage_limit: Min Voltage Limit for operating Servo，Value range is:33~90，unit is 0.1V.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = min_voltage_limit
        Primary_Servo.servo_write(servo_id, Primary_Address.MIN_VOLTAGE_LIMIT, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_pwm_limit(servo_id: int, max_pwm_limit: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the max PWM limit of servo.
        :param servo_id: ServoID.
        :param max_pwm_limit: Max PWM Limit Limit of servo，Value range is:0~1000，unit is 0.1%.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = max_pwm_limit & 0xff
        buffer[1] = (max_pwm_limit >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.MAX_PWM_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_current_limit(servo_id: int, max_current_limit: int, output_buffer: list,
                                    output_buffer_len: list) -> int:
        """
        Set the max current limit of servo.
        :param servo_id: ServoID.
        :param max_current_limit: Max Current Limit for operating servo，Value range is:0~1500，unit is 1mA.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = max_current_limit & 0xff
        buffer[1] = (max_current_limit >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.MAX_CURRENT_LIMIT_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_current_shutdown_time(servo_id: int, current_shutdown_time: int, output_buffer: list,
                                        output_buffer_len: list) -> int:
        """
        Set the current shutdown time of servo.
        :param servo_id: ServoID.
        :param current_shutdown_time: Trigger Time for overload protection activation after reaching current limit，Value range is:0~65536，unit is 1ms.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = current_shutdown_time & 0xff
        buffer[1] = (current_shutdown_time >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.CURRENT_TIME_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_cw_deadband(servo_id: int, cw_deadband: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the CW deadband of servo.
        :param servo_id: ServoID.
        :param cw_deadband: Dead Band for clockwise direction，Value range is:0~255，unit is 0.1°.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = cw_deadband
        Primary_Servo.servo_write(servo_id, Primary_Address.CW_DEADBAND, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_ccw_deadband(servo_id: int, ccw_deadband: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the CCW deadband of servo.
        :param servo_id: ServoID.
        :param ccw_deadband: Dead Band for counterclockwise direction，Value range is:0~255，unit is 0.1°.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = ccw_deadband
        Primary_Servo.servo_write(servo_id, Primary_Address.CCW_DEADBAND, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_pwm_punch(servo_id: int, pwm_punch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the PWM punch of servo.
        :param servo_id: ServoID.
        :param pwm_punch: PWM punch of the servo output.，Value range is:0~255，unit is 0.1%.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = pwm_punch
        Primary_Servo.servo_write(servo_id, Primary_Address.PWM_PUNCH, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_p_gain(servo_id: int, position_control_p_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        Set the position control P gain of servo.
        :param servo_id: ServoID.
        :param position_control_p_gain: Gain Proportion (P) of Servo's PID Control，Value range is:0~65535，Kp = Value/1000.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = position_control_p_gain & 0xff
        buffer[1] = (position_control_p_gain >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.POSITION_P_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_i_gain(servo_id: int, position_control_i_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        Set the position control I gain of servo.
        :param servo_id: ServoID.
        :param position_control_i_gain: Gain Integration (I) of Servo's PID Control，Value range is:0~65535，Ki = Value/10000.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = position_control_i_gain & 0xff
        buffer[1] = (position_control_i_gain >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.POSITION_I_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_d_gain(servo_id: int, position_control_d_gain: int, output_buffer: list,
                                          output_buffer_len: list) -> int:
        """
        Set the position control D gain of servo.
        :param servo_id: ServoID.
        :param position_control_d_gain: Gain Differential (D) of Servo's PID Control，Value range is:0~65535，Ki = Value/100.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = position_control_d_gain & 0xff
        buffer[1] = (position_control_d_gain >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.POSITION_D_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_led_condition(servo_id: int, led_condition: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the LED condition of servo.
        :param servo_id: ServoID.
        :param led_condition: Conditions for Alarm LED，Value range is:0~255.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = led_condition
        Primary_Servo.servo_write(servo_id, Primary_Address.LED_CONDITION, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_shutdown_conditions(servo_id: int, shutdown_conditions: int, output_buffer: list,
                                      output_buffer_len: list) -> int:
        """
        Set the shutdown condition of servo.
        :param servo_id: ServoID.
        :param shutdown_conditions: Conditions for torque unloading，Value range is:0~255.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = shutdown_conditions
        Primary_Servo.servo_write(servo_id, Primary_Address.SHUTDOWN_CONDITION, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_control_mode(servo_id: int, control_mode: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the control mode of servo.
        :param servo_id: ServoID.
        :param control_mode: Servo Control Mode: 0 Time Base Position Control, 1 Velocity Base Position Control, 2 Current, 3 PWM.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = control_mode
        Primary_Servo.servo_write(servo_id, Primary_Address.CONTROL_MODE, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_flash_switch(servo_id: int, flash_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the Flash switch of servo.
        :param servo_id: ServoID.
        :param flash_switch: Flash area write switch: 0 for write disabled, 1 for write enabled.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = flash_switch
        Primary_Servo.servo_write(servo_id, Primary_Address.FLASH_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_led_switch(servo_id: int, led_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the LED switch of servo.
        :param servo_id: ServoID.
        :param led_switch: Servo indicator light switch: 0 for off, 1 for on.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = led_switch
        Primary_Servo.servo_write(servo_id, Primary_Address.LED_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_torque_switch(servo_id: int, torque_switch: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the torque switch of servo.
        :param servo_id: ServoID.
        :param torque_switch: Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = torque_switch
        Primary_Servo.servo_write(servo_id, Primary_Address.TORQUE_SW, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_target_pwm(servo_id: int, target_pwm: int, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the target PWM of servo.
        :param servo_id: ServoID.
        :param target_pwm: Direct control of PWM output to the motor.，Value range is:-1000~1000，unit is 0.1%.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = target_pwm & 0xff
        buffer[1] = (target_pwm >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.TARGET_PWM_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_target_current(servo_id: int, target_current: int, output_buffer: list,
                                 output_buffer_len: list) -> int:
        """
        Set the target current of servo.
        :param servo_id: ServoID.
        :param target_current: Target current for servo operation, by default, equal to the default value of the max current limit.，Value range is:-1000~1000，unit is 1mA.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = target_current & 0xff
        buffer[1] = (target_current >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.TARGET_CURRENT_L, 2, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_position(servo_id: int, target_position: int, output_buffer: list,
                                                output_buffer_len: list) -> int:
        """
        Set the velocity base target position of servo.
        :param servo_id: ServoID.
        :param target_position: Used in Velocity Base Position Control Mode.Value range is:0~3000，unit is 0.1°.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = target_position & 0xff
        buffer[1] = (target_position >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.VELOCITY_BASE_TARGET_POSITION_L, 2, buffer, output_buffer,
                          output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_velocity(servo_id: int, target_velocity: int, output_buffer: list,
                                                output_buffer_len: list) -> int:
        """
        Set the velocity base target velocity of servo.
        :param servo_id: ServoID.
        :param target_velocity: Value range is:0~65535，unit is 0.1°/s.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 2

        buffer[0] = target_velocity & 0xff
        buffer[1] = (target_velocity >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.VELOCITY_BASE_TARGET_VELOCITY_L, 2, buffer, output_buffer,
                          output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_acc(servo_id: int, target_acc: int, output_buffer: list,
                                           output_buffer_len: list) -> int:
        """
        Set the velocity base target ACC of servo.
        :param servo_id: ServoID.
        :param target_acc: Value range is:0~255，unit is 50°/s².
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = target_acc
        Primary_Servo.servo_write(servo_id, Primary_Address.VELOCITY_BASE_TARGET_ACC, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_dec(servo_id: int, target_dec: int, output_buffer: list,
                                           output_buffer_len: list) -> int:
        """
        Set the velocity base target DEC of servo.
        :param servo_id: ServoID.
        :param target_dec: Value range is:0~255，unit is unit is 50°/s².
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = target_dec
        Primary_Servo.servo_write(servo_id, Primary_Address.VELOCITY_BASE_TARGET_DEC, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_acc(servo_id: int, target_acc: int, output_buffer: list,
                                       output_buffer_len: list) -> int:
        """
        Set the time base target ACC of servo.
        :param servo_id: ServoID.
        :param target_acc: Value range is:0~5.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        input_buffer = [0]
        input_buffer[0] = target_acc
        Primary_Servo.servo_write(servo_id, Primary_Address.TIME_BASE_TARGET_ACC, 1, input_buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_position_and_moving_time(servo_id: int, target_position: int, moving_time: int,
                                                            output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the time base target position and moving time of servo.
        :param servo_id: ServoID.
        :param target_position: Value range is:0~3000，unit is 0.1°.
        :param moving_time: Value range is:0~65535，unit is 1ms.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        buffer = [0] * 4

        buffer[0] = target_position & 0xff
        buffer[1] = (target_position >> 8) & 0xff
        buffer[2] = moving_time & 0xff
        buffer[3] = (moving_time >> 8) & 0xff

        Primary_Servo.servo_write(servo_id, Primary_Address.TIME_BASE_TARGET_POSITION_L, 4, buffer, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_set_return_delay_time_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)

        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_return_level_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)

        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_baud_rate_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_min_angle_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_angle_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_temperature_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_voltage_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_min_voltage_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_pwm_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_max_current_limit_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_current_shutdown_time_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_cw_deadband_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_ccw_deadband_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_pwm_punch_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_p_gain_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_i_gain_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_position_control_d_gain_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_led_condition_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_shutdown_conditions_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_control_mode_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_flash_switch_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_led_switch_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_torque_switch_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_target_pwm_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_target_current_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_position_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_velocity_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_acc_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_velocity_base_target_dec_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_acc_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_set_time_base_target_position_and_moving_time_analysis(response_packet: list) -> int:
        """
        Parsing the servo response packet.
        :param response_packet: Servo response packet.
        :return: Function execution result, success or error flag.
        """
        data_buffer = [0] * 2

        ret = Primary_Servo.servo_unpack(response_packet, data_buffer)
        if ret != Primary_State.SUCCESS:
            return ret
        else:
            print("Write  Successful!")
            return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_torque_switch(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set torque switches for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 1 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.TORQUE_SW
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 1] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 1] = servo_sync_parameter.torque_switch[i] & 0xff

        Primary_Servo.sync_write_data(Primary_Address.TORQUE_SW, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_control_mode(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the control mode for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 1 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.CONTROL_MODE
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 1] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 1] = servo_sync_parameter.control_mode[i] & 0xff

        Primary_Servo.sync_write_data(Primary_Address.CONTROL_MODE, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_position(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the velocity base target position for multiple servos
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 2 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 2

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 2] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 2] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 2] = (servo_sync_parameter.position[i] >> 8) & 0xff

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the target position and speed for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 4 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 4

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 4] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 4] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 4] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[i + 5 + i * 4] = servo_sync_parameter.velocity[i] & 0xff
            parameter[i + 6 + i * 4] = (servo_sync_parameter.velocity[i] >> 8) & 0xff

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the target acceleration, deceleration, speed, and position for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 6 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_POSITION_L
        parameter[1] = 6

        for i in range(servo_sync_parameter.id_counts):
            parameter[2 + i * 7] = servo_sync_parameter.id[i]
            parameter[3 + i * 7] = servo_sync_parameter.position[i] & 0xff
            parameter[4 + i * 7] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[5 + i * 7] = servo_sync_parameter.velocity[i] & 0xff
            parameter[6 + i * 7] = (servo_sync_parameter.velocity[i] >> 8) & 0xff
            parameter[7 + i * 7] = servo_sync_parameter.acc_velocity[i] & 0xff
            parameter[8 + i * 7] = servo_sync_parameter.dec_velocity[i] & 0xff

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_velocity(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the velocity base target velocity for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 2 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_VELOCITY_L
        parameter[1] = 2

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 2] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 2] = servo_sync_parameter.velocity[i] & 0xff
            parameter[i + 4 + i * 2] = (servo_sync_parameter.velocity[i] >> 8) & 0xff

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_VELOCITY_L, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_acc(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the target acceleration for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_ACC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.acc_velocity[i]

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_ACC, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_velocity_base_target_dec(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Set the target deceleration for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.VELOCITY_BASE_TARGET_DEC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.dec_velocity[i]

        Primary_Servo.sync_write_data(Primary_Address.VELOCITY_BASE_TARGET_DEC, servo_sync_parameter.id_counts, parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_time_base_target_acc(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Change the time base target ACC for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.TIME_BASE_TARGET_ACC
        parameter[1] = 1

        for i in range(servo_sync_parameter.id_counts):
            parameter[i * 2 + 2] = servo_sync_parameter.id[i]
            parameter[i * 2 + 3] = servo_sync_parameter.acc_velocity_grade[i]

        Primary_Servo.sync_write_data(Primary_Address.TIME_BASE_TARGET_ACC, servo_sync_parameter.id_counts , parameter, output_buffer, output_buffer_len)
        return Primary_State.SUCCESS

    @staticmethod
    def servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter: Primary_Servo_Sync_Parameter, output_buffer: list, output_buffer_len: list) -> int:
        """
        Change the time base target position and moving time for multiple servos.
        :param servo_sync_parameter: Servo sync write modification class.
        :param output_buffer: Pointer for the output buffer that is used to store instruction packets.
        :param output_buffer_len: The length of the instruction packet.
        :return: Function execution result, success or error flag.
        """
        parameter = [0] * (servo_sync_parameter.id_counts * 4 + 2 + servo_sync_parameter.id_counts)

        parameter[0] = Primary_Address.TIME_BASE_TARGET_POSITION_L
        parameter[1] = 4

        for i in range(servo_sync_parameter.id_counts):
            parameter[i + 2 + i * 4] = servo_sync_parameter.id[i]
            parameter[i + 3 + i * 4] = servo_sync_parameter.position[i] & 0xff
            parameter[i + 4 + i * 4] = (servo_sync_parameter.position[i] >> 8) & 0xff
            parameter[i + 5 + i * 4] = servo_sync_parameter.time[i] & 0xff
            parameter[i + 6 + i * 4] = (servo_sync_parameter.time[i] >> 8) & 0xff

        Primary_Servo.sync_write_data(Primary_Address.TIME_BASE_TARGET_POSITION_L, servo_sync_parameter.id_counts, parameter, output_buffer,
                              output_buffer_len)
        return Primary_State.SUCCESS
