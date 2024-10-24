#include "PrimaryServo.h"
#include <stdio.h>
#include <stdlib.h>

/**
 * @brief Calculate the checksum of the instruction packet.
 * @param buffer: Pointer for the buffer to calculate the checksum.
 * @param length: Buffer Length.
 * @return Calculated Checksum.
 */
uint8_t primary_get_check(const uint8_t* buffer, uint8_t length)
{
    uint8_t sum = 0, i = 0;
    for (i = 0; i < length; i++)
    {
        sum += buffer[i];
    }
    sum = ~sum;
    return sum;
}

/**
 * @brief Package servo control command data.
 * @param id: ServoID.
 * @param instruction: Servo instruction.
 * @param address: The address to operate on.
 * @param byte_length: Byte length.
 * @param input_buffer: Instruction package parameter data.
 * @param output_buffer: Pointer for the buffer that is used to store the generated instruction packet.
 * @param output_length: Buffer Length.
 * @return success or error flag.
 */
uint8_t primary_servo_pack(uint8_t id, uint8_t instruction, uint8_t address, uint8_t byte_length, const uint8_t* input_buffer,
    uint8_t* output_buffer, uint8_t* output_length)
{
    uint8_t i = 0, j = 0;

    output_buffer[i++] = 0xff;
    output_buffer[i++] = 0xff;
    output_buffer[i++] = id;

    switch (instruction)
    {
    case PRIMARY_PING:
        output_buffer[i++] = 0x02;
        output_buffer[i++] = instruction;
        break;
    case PRIMARY_READ_DATA:
        output_buffer[i++] = 4;
        output_buffer[i++] = instruction;
        output_buffer[i++] = address;
        output_buffer[i++] = byte_length;
        break;
    case PRIMARY_WRITE_DATA:
        output_buffer[i++] = byte_length + 3;
        output_buffer[i++] = instruction;
        output_buffer[i++] = address;
        for (j = 0; j < byte_length; j++)
        {
            output_buffer[i++] = input_buffer[j];
        }
        break;
    case PRIMARY_SYNC_WRITE:
        output_buffer[i++] = (input_buffer[1] + 1) * byte_length + 4;
        output_buffer[i++] = instruction;
        for (j = 0; j < ((byte_length * input_buffer[1]) + 2 + byte_length); j++)
        {
            output_buffer[i++] = input_buffer[j];
        }
        break;
    case PRIMARY_FACTORY_RESET:
    case PRIMARY_PARAMETER_RESET:
    case PRIMARY_CALIBRATION:
    case PRIMARY_REBOOT:
        output_buffer[i++] = 0x04;
        output_buffer[i++] = instruction;
        output_buffer[i++] = 0xdf;
        output_buffer[i++] = 0xdf;
        break;
    default:
        return PRIMARY_FAILURE;
    }
    output_buffer[i] = primary_get_check(output_buffer + 2, i - 2);
    *output_length = i + 1;
    return PRIMARY_SUCCESS;
}

/**
 * @brief Parsing reply packet.
 * @param response_packet: packet data of the steering gear.
 * @param data_buffer: Data parsed from the status packet.
 * @return success or error flag.
 */
uint8_t primary_servo_unpack(uint8_t* response_packet, uint8_t** data_buffer)
{
    uint8_t length;
    uint8_t status;
    uint8_t checksum;

    length = response_packet[3];
    status = response_packet[4];

    checksum = primary_get_check(response_packet + 2, length + 1);

    if (response_packet[0] != 0xff || response_packet[1] != 0xff || checksum != response_packet[length + 3])
    {
        PRINTF("This is not a complete response package!\r\n");
        return PRIMARY_UNPACK_ERROR;
    }

    if (status != 0x00)
    {
        if ((status & PRIMARY_VOLTAGE_ERROR) == PRIMARY_VOLTAGE_ERROR)
        {
            PRINTF("Voltage Error\r\n");
        }
        if ((status & PRIMARY_ANGLE_ERROR) == PRIMARY_ANGLE_ERROR)
        {
            PRINTF("Angle Error\r\n");
        }
        if ((status & PRIMARY_OVERHEATING_ERROR) == PRIMARY_OVERHEATING_ERROR)
        {
            PRINTF("Overheating Error\r\n");
        }
        if ((status & PRIMARY_RANGE_ERROR) == PRIMARY_RANGE_ERROR)
        {
            PRINTF("Range Error\r\n");
        }
        if ((status & PRIMARY_CHECKSUM_ERROR) == PRIMARY_CHECKSUM_ERROR)
        {
            PRINTF("CheckSum Error\r\n");
        }
        if ((status & PRIMARY_STALL_ERROR) == PRIMARY_STALL_ERROR)
        {
            PRINTF("Stall Error\r\n");
        }
        if ((status & PRIMARY_PARSING_ERROR) == PRIMARY_PARSING_ERROR)
        {
            PRINTF("Parsing Error\r\n");
        }
        return status;
    }

    if (length > 2) {
        *data_buffer = &response_packet[5];
    }

    return PRIMARY_SUCCESS;
}

/**
 * @brief Generate the PING instruction package.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_ping(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_PING, 0, 0, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Generate instructions to read memory table data.
 * @param id: ServoID.
 * @param address: The address of the memory table to read.
 * @param read_data_len: The size of the bytes to read.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read(uint8_t id, uint8_t address, uint8_t read_data_len, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_READ_DATA, address, read_data_len, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Generate instructions to write memory table data.
 * @param id: ServoID.
 * @param address: The address of the memory table to write data to.
 * @param write_data_len: The length of the data to be written.
 * @param input_buffer: Written data.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_write(uint8_t id, uint8_t address, uint8_t write_data_len, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_WRITE_DATA, address, write_data_len, input_buffer, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Generate sync write instructions to write memory table data.
 * @param address: The address of the memory table to write data to.
 * @param servo_counts: The number of servos operated by sync write instructions.
 * @param input_buffer: Written data.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_sync_write_data(uint8_t address, uint8_t servo_counts, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(0xfe, PRIMARY_SYNC_WRITE, address, servo_counts, input_buffer, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Reset the servo to the factory default values.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_factory_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_FACTORY_RESET, 0, 0, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Reset the parameter settings of the servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_parameter_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_PARAMETER_RESET, 0, 0, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Calibrate the midpoint of the servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_CALIBRATION, 0, 0, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Reboot the servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_reboot(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_pack(id, PRIMARY_REBOOT, 0, 0, NULL, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Change the servo ID.
 * @param id: ServoID.
 * @param new_id The modified ID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_modify_known_id(uint8_t id, uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_SERVO_ID, 1, &new_id, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the servo ID of the servo with an unknown ID.
 * @param new_id The modified ID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_modify_unknown_id(uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(0xfe, PRIMARY_SERVO_ID, 1, &new_id, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the return delay time of the servo.
 * @param id: ServoID.
 * @param response_delay_time: Response Delay Time for servo return to packet，Value range is:0~255，unit is 2μs.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_return_delay_time(uint8_t id, uint8_t response_delay_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_RETURN_DELAY_TIME, 1, &response_delay_time, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the return level of servo.
 * @param id: ServoID.
 * @param return_level Returned level of servo status，Value range is:0 1 2.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_return_level(uint8_t id, uint8_t return_level, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_RETURN_LEVEL, 1, &return_level, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the baud rate of servo.
 * @param id: ServoID.
 * @param baud_rate_number Baud Rate Number 7 is 1Mbps，Value range is:1~7.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_baud_rate(uint8_t id, uint8_t baud_rate_number, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_BAUD_RATE, 1, &baud_rate_number, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the min angle limit of servo.
 * @param id: ServoID.
 * @param min_angle_limit Min Angle Limit for servo rotation，Value range is:0~3000，unit is 0.1°.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_min_angle_limit(uint8_t id, uint16_t min_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = min_angle_limit & 0xff;
    buffer[1] = (min_angle_limit >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_MIN_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the max angle limit of servo.
 * @param id: ServoID.
 * @param max_angle_limit Max Angle Limit for servo rotation，Value range is:0~3000，unit is 0.1°.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_angle_limit(uint8_t id, uint16_t max_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = max_angle_limit & 0xff;
    buffer[1] = (max_angle_limit >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_MAX_ANGLE_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the max temperature limit of servo.
 * @param id: ServoID.
 * @param max_temperature_limit: Max Temperature Limit for operating servo，Value range is:0~127，unit is 1℃.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_temperature_limit(uint8_t id, uint8_t max_temperature_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_MAX_TEMPERATURE_LIMIT, 1, &max_temperature_limit, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the max voltage limit of servo.
 * @param id: ServoID.
 * @param max_voltage_limit Max Voltage Limit for operating Servo，Value range is:33~90，unit is 0.1V.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_voltage_limit(uint8_t id, uint8_t max_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_MAX_VOLTAGE_LIMIT, 1, &max_voltage_limit, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the min voltage limit of servo.
 * @param id: ServoID.
 * @param min_voltage_limit Min Voltage Limit for operating Servo，Value range is:33~90，unit is 0.1V.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_min_voltage_limit(uint8_t id, uint8_t min_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_MIN_VOLTAGE_LIMIT, 1, &min_voltage_limit, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the max PWM limit of servo.
 * @param id: ServoID.
 * @param max_pwm_limit Max PWM Limit Limit of servo，Value range is:0~1000，unit is 0.1%.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_pwm_limit(uint8_t id, uint16_t max_pwm_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = max_pwm_limit & 0xff;
    buffer[1] = (max_pwm_limit >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_MAX_PWM_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the max current limit of servo.
 * @param id: ServoID.
 * @param max_current_limit Max Current Limit for operating servo，Value range is:0~1500，unit is 1mA.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_current_limit(uint8_t id, uint16_t max_current_limit, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = max_current_limit & 0xff;
    buffer[1] = (max_current_limit >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_MAX_CURRENT_LIMIT_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the current shutdown time of servo.
 * @param id: ServoID.
 * @param current_shutdown_time Trigger Time for overload protection activation after reaching current limit，Value range is:0~65536，unit is 1ms.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_current_shutdown_time(uint8_t id, uint16_t current_shutdown_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = current_shutdown_time & 0xff;
    buffer[1] = (current_shutdown_time >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_CURRENT_TIME_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the CW deadband of servo.
 * @param id: ServoID.
 * @param cw_deadband Dead Band for clockwise direction，Value range is:0~255，unit is 0.1°.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_cw_deadband(uint8_t id, uint8_t cw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_CW_DEADBAND, 1, &cw_deadband, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the CCW deadband of servo.
 * @param id: ServoID.
 * @param ccw_deadband Dead Band for counterclockwise direction，Value range is:0~255，unit is 0.1°.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_ccw_deadband(uint8_t id, uint8_t ccw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_CCW_DEADBAND, 1, &ccw_deadband, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the PWM punch of servo.
 * @param id: ServoID.
 * @param pwm_punch PWM punch of the servo output.，Value range is:0~255，unit is 0.1%.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_pwm_punch(uint8_t id, uint8_t pwm_punch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_PWM_PUNCH, 1, &pwm_punch, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the position control P gain of servo.
 * @param id: ServoID.
 * @param position_control_P_gain Gain Proportion (P) of Servo's PID Control，Value range is:0~65535，Kp = Value/1000.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_p_gain(uint8_t id, uint16_t position_control_P_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = position_control_P_gain & 0xff;
    buffer[1] = (position_control_P_gain >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_POSITION_P_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the position control I gain of servo.
 * @param id: ServoID.
 * @param position_control_I_gain Gain Integration (I) of Servo's PID Control，Value range is:0~65535，Ki = Value/10000.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_i_gain(uint8_t id, uint16_t position_control_I_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = position_control_I_gain & 0xff;
    buffer[1] = (position_control_I_gain >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_POSITION_I_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the position control D gain of servo.
 * @param id: ServoID.
 * @param position_control_D_gain Gain Differential (D) of Servo's PID Control，Value range is:0~65535，Ki = Value/100.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_d_gain(uint8_t id, uint16_t position_control_D_gain, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = position_control_D_gain & 0xff;
    buffer[1] = (position_control_D_gain >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_POSITION_D_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the LED condition of servo.
 * @param id: ServoID.
 * @param led_condition Conditions for Alarm LED，Value range is:0~255.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_led_condition(uint8_t id, uint8_t led_condition, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_LED_CONDITION, 1, &led_condition, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the shutdown condition of servo.
 * @param id: ServoID.
 * @param shutdown_conditions Conditions for torque unloading，Value range is:0~255.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_shutdown_conditions(uint8_t id, uint8_t shutdown_conditions, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_SHUTDOWN_CONDITION, 1, &shutdown_conditions, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the control mode of servo.
 * @param id: ServoID.
 * @param control_mode: Servo Control Mode: 0 Time Base Position Control, 1 Velocity Base Position Control, 2 Current, 3 PWM.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_control_mode(uint8_t id, uint8_t control_mode, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_CONTROL_MODE, 1, &control_mode, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the Flash switch of servo.
 * @param id: ServoID.
 * @param flash_switch: Flash area write switch: 0 for write disabled, 1 for write enabled.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_flash_switch(uint8_t id, uint8_t flash_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_FLASH_SW, 1, &flash_switch, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the LED switch of servo.
 * @param id: ServoID.
 * @param led_switch: Servo indicator light switch: 0 for off, 1 for on.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_led_switch(uint8_t id, uint8_t led_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_LED_SW, 1, &led_switch, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the torque switch of servo.
 * @param id: ServoID.
 * @param torque_switch: Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_torque_switch(uint8_t id, uint8_t torque_switch, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_TORQUE_SW, 1, &torque_switch, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the target PWM of servo.
 * @param id: ServoID.
 * @param target_pwm Direct control of PWM output to the motor.，Value range is:-1000~1000，unit is 0.1%.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_target_pwm(uint8_t id, int16_t target_pwm, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = target_pwm & 0xff;
    buffer[1] = (target_pwm >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_TARGET_PWM_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the target current of servo.
 * @param id: ServoID.
 * @param target_current Target current for servo operation, by default, equal to the default value of the max current limit.，Value range is:-1000~1000，unit is 1mA.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_target_current(uint8_t id, int16_t target_current, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = target_current & 0xff;
    buffer[1] = (target_current >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_TARGET_CURRENT_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the velocity base target position of servo.
 * @param id: ServoID.
 * @param target_position: Used in Velocity Base Position Control Mode.Value range is:0~3000，unit is 0.1°.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_position(uint8_t id, uint16_t target_position, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = target_position & 0xff;
    buffer[1] = (target_position >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_VELOCITY_BASE_TARGET_POSITION_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the velocity base target velocity of servo.
 * @param id: ServoID.
 * @param target_velocity Value range is:0~65535，unit is 0.1°/s.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_velocity(uint8_t id, uint16_t target_velocity, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[2] = { 0 };

    buffer[0] = target_velocity & 0xff;
    buffer[1] = (target_velocity >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_VELOCITY_BASE_TARGET_VELOCITY_L, 2, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the velocity base target ACC of servo.
 * @param id: ServoID.
 * @param target_acc Value range is:0~255，unit is 50°/s².
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_VELOCITY_BASE_TARGET_ACC, 1, &target_acc, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the velocity base target DEC of servo.
 * @param id: ServoID.
 * @param target_dec Value range is:0~255，unit is unit is 50°/s².
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_dec(uint8_t id, uint8_t target_dec, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_VELOCITY_BASE_TARGET_DEC, 1, &target_dec, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the time base target ACC of servo.
 * @param id: ServoID.
 * @param target_acc Value range is:0~5.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_time_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_write(id, PRIMARY_TIME_BASE_TARGET_ACC, 1, &target_acc, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the time base target position and moving time of servo.
 * @param id: ServoID.
 * @param target_position: Value range is:0~3000，unit is 0.1°.
 * @param moving_time: Value range is:0~65535，unit is 1ms.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_time_base_target_position_and_moving_time(uint8_t id, uint16_t target_position, uint16_t moving_time, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t buffer[4] = { 0 };

    buffer[0] = target_position & 0xff;
    buffer[1] = (target_position >> 8) & 0xff;
    buffer[2] = moving_time & 0xff;
    buffer[3] = (moving_time >> 8) & 0xff;

    primary_servo_write(id, PRIMARY_TIME_BASE_TARGET_POSITION_L, 4, buffer, output_buffer, output_buffer_len);
    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present position and present current of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_position_and_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_POSITION_L, 4, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present current of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_CURRENT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present position of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_POSITION_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present velocity of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present profile position of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_profile_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_PROFILE_POSITION_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present profile velocity of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_profile_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_PROFILE_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present PWM of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_PWM_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present temperature of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_temperature(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_TEMPERATURE, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the present voltage of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_voltage(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PRESENT_VOLTAGE, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the time base target moving time of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_moving_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TIME_BASE_TARGET_MOVINGTIME_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the time base target position of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TIME_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the time base target ACC of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TIME_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the velocity base target DEC of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_dec(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_VELOCITY_BASE_TARGET_DEC, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the velocity base target ACC of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_VELOCITY_BASE_TARGET_ACC, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the velocity base target velocity of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_VELOCITY_BASE_TARGET_VELOCITY_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the velocity base target position of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_VELOCITY_BASE_TARGET_POSITION_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the target current of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_target_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TARGET_CURRENT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the target PWM of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_target_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TARGET_PWM_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the torque switch of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_torque_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_TORQUE_SW, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the LED switch of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_led_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_LED_SW, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the Flash switch of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_flash_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_FLASH_SW, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the calibration of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_CALIBRATION_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the control mode of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_control_mode(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_CONTROL_MODE, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the shutdown condition of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_shutdown_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_SHUTDOWN_CONDITION, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the LED condition of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_led_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_LED_CONDITION, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the position control D gain of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_d_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_POSITION_D_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the position control I gain of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_i_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_POSITION_I_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the position control P gain of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_p_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_POSITION_P_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the PWM punch of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_pwm_punch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_PWM_PUNCH, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the CCW deadband of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_ccw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_CCW_DEADBAND, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the CW deadband of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_cw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_CW_DEADBAND, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the current shutdown time of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_current_shutdown_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_CURRENT_TIME_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the max current limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_current_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MAX_CURRENT_LIMIT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the max PWM limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_pwm_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MAX_PWM_LIMIT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the max voltage limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MAX_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the min voltage limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_min_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MIN_VOLTAGE_LIMIT, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the max temperature limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_temperature_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MAX_TEMPERATURE_LIMIT, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the max angle limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MAX_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the min angle limit of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_min_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MIN_ANGLE_LIMIT_L, 2, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the return level of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_return_level(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_RETURN_LEVEL, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the return delay time of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_return_delay_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_RETURN_DELAY_TIME, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the baud rate of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_baud_rate(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_BAUD_RATE, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the model information of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_model_information(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_MODEL_INFORMATION, 4, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Read the firmware version of servo.
 * @param id: ServoID.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_firmware_version(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    primary_servo_read(id, PRIMARY_FIRMWARE_VERSION, 1, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set torque switches for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_torque_switch(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 6 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_TORQUE_SW;
    parameter[1] = 1;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 2] = servo.id[i];
        parameter[3 + i * 2] = servo.torque_switch[i] & 0xff;
    }

    primary_sync_write_data(PRIMARY_TORQUE_SW, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the control mode for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_control_mode(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 6 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_CONTROL_MODE;
    parameter[1] = 1;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 2] = servo.id[i];
        parameter[3 + i * 2] = servo.control_mode[i] & 0xff;
    }

    primary_sync_write_data(PRIMARY_CONTROL_MODE, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the target acceleration, deceleration, speed, and position for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 6 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 6;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 7] = servo.id[i];
        parameter[3 + i * 7] = servo.position[i] & 0xff;
        parameter[4 + i * 7] = (servo.position[i] >> 8) & 0xff;
        parameter[5 + i * 7] = servo.velocity[i] & 0xff;
        parameter[6 + i * 7] = (servo.velocity[i] >> 8) & 0xff;
        parameter[7 + i * 7] = servo.acc_velocity[i];
        parameter[8 + i * 7] = servo.dec_velocity[i];
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Change the velocity base target position for multiple servos
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_position(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 2 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 2;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 3] = servo.id[i];
        parameter[3 + i * 3] = servo.position[i] & 0xff;
        parameter[4 + i * 3] = (servo.position[i] >> 8) & 0xff;
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the velocity base target velocity for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_velocity(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 2 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_VELOCITY_L;
    parameter[1] = 2;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 3] = servo.id[i];
        parameter[3 + i * 3] = servo.velocity[i] & 0xff;
        parameter[4 + i * 3] = (servo.velocity[i] >> 8) & 0xff;
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_VELOCITY_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the target position and speed for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_position_and_velocity(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 4 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_POSITION_L;
    parameter[1] = 4;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 5] = servo.id[i];
        parameter[3 + i * 5] = servo.position[i] & 0xff;
        parameter[4 + i * 5] = (servo.position[i] >> 8) & 0xff;
        parameter[5 + i * 5] = servo.velocity[i] & 0xff;
        parameter[6 + i * 5] = (servo.velocity[i] >> 8) & 0xff;
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;

}

/**
 * @brief Set the target acceleration for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_acc(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_ACC;
    parameter[1] = 1;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 2] = servo.id[i];
        parameter[3 + i * 2] = servo.acc_velocity[i];
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_ACC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Set the target deceleration for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_velocity_base_target_dec(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_VELOCITY_BASE_TARGET_DEC;
    parameter[1] = 1;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 2] = servo.id[i];
        parameter[3 + i * 2] = servo.dec_velocity[i];
    }

    primary_sync_write_data(PRIMARY_VELOCITY_BASE_TARGET_DEC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Change the time base target ACC for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_time_base_target_acc(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_TIME_BASE_TARGET_ACC;
    parameter[1] = 1;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 2] = servo.id[i];
        parameter[3 + i * 2] = servo.acc_velocity_grade[i];
    }

    primary_sync_write_data(PRIMARY_TIME_BASE_TARGET_ACC, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Change the time base target position and moving time for multiple servos.
 * @param servo: Structure for storing servo sync write parameters.
 * @param output_buffer: Pointer for the output buffer that is used to store instruction packets.
 * @param output_buffer_len: The length of the instruction packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_sync_write_time_base_target_position_and_moving_time(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len)
{
    uint8_t i = 0;
    uint8_t parameter[MAX_SERVERS * 4 + 2 + MAX_SERVERS];

    parameter[0] = PRIMARY_TIME_BASE_TARGET_POSITION_L;
    parameter[1] = 4;
    for (i = 0; i < servo.id_counts; i++)
    {
        parameter[2 + i * 5] = servo.id[i];
        parameter[3 + i * 5] = servo.position[i] & 0xff;
        parameter[4 + i * 5] = (servo.position[i] >> 8) & 0xff;
        parameter[5 + i * 5] = servo.time[i] & 0xff;
        parameter[6 + i * 5] = (servo.time[i] >> 8) & 0xff;
    }

    primary_sync_write_data(PRIMARY_TIME_BASE_TARGET_POSITION_L, servo.id_counts, parameter, output_buffer, output_buffer_len);

    return PRIMARY_SUCCESS;
}

/**
 * @brief Parsing the servo response packet for the PING command.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_ping_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Ping Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the factory reset command.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_factory_reset_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Factory Reset Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the parameter reset command.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_parameter_reset_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Parameter Reset Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the calibration command.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_calibration_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Calibration Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_return_delay_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_return_level_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_baud_rate_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_min_angle_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_angle_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_temperature_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_voltage_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag
 */
uint8_t primary_servo_set_min_voltage_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_pwm_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_max_current_limit_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_current_shutdown_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_cw_deadband_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_ccw_deadband_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_pwm_punch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_p_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_i_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_position_control_d_gain_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_led_condition_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_shutdown_conditions_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_control_mode_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_flash_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_led_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_torque_switch_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_target_pwm_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_target_current_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_position_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_velocity_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_acc_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_velocity_base_target_dec_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_time_base_target_acc_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet.
 * @param response_packet: Servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_set_time_base_target_position_and_moving_time_analysis(uint8_t* response_packet)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Write Successful!\r\n");
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present current.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_current_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present position.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_position_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present position and current.
 * @param response_packet: Servo response packet.
 * @param position: present position.
 * @param current: present current.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_position_and_present_current_analysis(uint8_t* response_packet, uint16_t* position, uint16_t* current)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *position = data_buffer[1];
        *position = *position << 8;
        *position = *position | data_buffer[0];
        *current = data_buffer[3];
        *current = *current << 8;
        *current = *current | data_buffer[2];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present velocity.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present profile position.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_profile_position_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present profile velocity.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_profile_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present pwm.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_pwm_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present temperature.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_temperature_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the present voltage.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_present_voltage_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the time base target moving time.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_moving_time_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the time base target position.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_position_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the time base target accelerated speed.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_time_base_target_acc_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;


    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the velocity base target deceleration.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_dec_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the velocity base target accelerated speed.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_acc_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the velocity base target velocity.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the velocity base target position.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_velocity_base_target_position_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the target current.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_target_current_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the target pwm.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_target_pwm_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the torque switch.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_torque_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the led switch.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_led_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the flash switch.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_flash_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the current offset.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_current_offset_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the calibration.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_calibration_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the control mode.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_control_mode_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the shutdown condition.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_shutdown_condition_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
    }

    return PRIMARY_SUCCESS;
}

/**
 * @brief Parsing the servo response packet for the led condition.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_led_condition_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
    }

    return PRIMARY_SUCCESS;
}

/**
 * @brief Parsing the servo response packet for the position control d gain.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_d_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the position control i gain.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_i_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the position control p gain.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_position_control_p_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the pwm punch.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_pwm_punch_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the ccw deadband.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_ccw_deadband_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the cw deadband.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_cw_deadband_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the current shutdown time.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_current_shutdown_time_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the max current limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_current_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the max pwm limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_pwm_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the max voltage limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_voltage_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the min voltage limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_min_voltage_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the max temperature limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_temperature_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the max angle limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_max_angle_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the min angle limit.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_min_angle_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the return level.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_return_level_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the return delay time.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_return_delay_time_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the baud rate.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_baud_rate_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the model information.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_model_information_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[3];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[2];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[1];
        *analysis_data = *analysis_data << 8;
        *analysis_data = *analysis_data | data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}

/**
 * @brief Parsing the servo response packet for the firmware version.
 * @param response_packet: Servo response packet.
 * @param analysis_data: The data parsed from the servo response packet.
 * @return Function execution result, success or error flag.
 */
uint8_t primary_servo_read_firmware_version_analysis(uint8_t* response_packet, uint32_t* analysis_data)
{
    uint8_t ret;
    uint8_t* data_buffer = NULL;

    ret = primary_servo_unpack(response_packet, &data_buffer);

    if (ret != PRIMARY_SUCCESS) {
        return ret;
    }
    else {
        PRINTF("Read Successful!\r\n");
        *analysis_data = data_buffer[0];
        return PRIMARY_SUCCESS;
    }
}







