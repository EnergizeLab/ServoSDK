#include "PrimaryServo.h"
#include "CSerialPort.h"

#define READ_TEST 0             //Read Servo Data Test
#define WRITE_TEST 0            //Write Servo Data Test
#define SYNC_WRITE_TEST 0       //Sync Write Test
#define PING_TEST 0             //PING Instruction Test
#define FACTORY_RESET_TEST 0    //Factory Reset Test
#define PARAMETER_RESET_TEST 0  //Parameter Reset Test
#define REBOOT_TEST 0           //Reboot Test
#define CALIBRATION_TEST 0      //Calibration Test
#define MODIFY_ID 0             //Change Known Servo ID Test
#define MODIFY_UNKNOWN_ID 0     //Change Unknown Servo ID Test

struct primary_servo_sync_parameter servo;

int main()
{

    uint8_t ret;                                                    //Status Flag
    uint8_t order_buffer[40];                                       //Store Generated Instructions
    uint8_t order_len = 0;                                          //Instruction Length
    uint8_t pack[40];                                               //Store the received status packet
    uint32_t analysis_data = 0;                                     //Data parsed from the status packet
    uint16_t position = 0;                                          //Present position of the servo
    uint16_t current = 0;                                           //Present current of the servo
    uint8_t write_buffer[20] = { 0 };                               //Write data to the memory table                                                                      

    CSerialPort serialPort;                                         //Create a serial port class.

    DWORD bytesRead;                                                //The actual number of bytes read from the serial port.
    DWORD bytesWritten;                                             //The actual number of bytes written to the serial port.


    if (serialPort.Open(20, 1000000))
    {
        PRINTF("open serial complete\r\n");
    }
    else
    {
        PRINTF("failed to open serial port\r\n");
        return -1;
    }

#if PING_TEST
    //Query the model number of servo ID1.
    primary_servo_ping(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_ping_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("model number: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data.\r\n");
    }
    Sleep(20);
#endif

#if CALIBRATION_TEST
    //Calibrate the midpoint of the servo.
    primary_servo_calibration(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_calibration_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("calibration complete\r\n");
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

#endif

#if FACTORY_RESET_TEST
    //Reset the servo to the factory default values.
    primary_servo_factory_reset(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_factory_reset_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("factory reset complete\r\n");
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);
#endif

#if PARAMETER_RESET_TEST
    //Reset the parameter settings of the servo.
    primary_servo_parameter_reset(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_parameter_reset_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("parameter reset complete\r\n");
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);
#endif

#if REBOOT_TEST
    //Reboot the servo.
    primary_servo_reboot(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(20);
#endif

#if MODIFY_ID
    //Change the servo ID of servo ID1 to 2.
    primary_servo_modify_known_id(1, 2, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(20);
#endif

#if MODIFY_UNKNOWN_ID
    //Change the servo ID of the servo with an unknown ID to 1.
    primary_servo_modify_unknown_id(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(20);
#endif

#if READ_TEST
    //Read the present current of servo ID1.
    primary_servo_read_present_current(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_current_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present current: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present position of servo ID1.
    primary_servo_read_present_position(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_position_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present position: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present position and present current of servo ID1.
    primary_servo_read_present_position_and_present_current(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_position_and_present_current_analysis(pack, &position, &current);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present position: %d, present current: %d\r\n", position, current);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present velocity of servo ID1.
    primary_servo_read_present_velocity(1, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_velocity_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present velocity: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present profile position of servo ID1.
    primary_servo_read_present_profile_position(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_profile_position_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present profile position: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present profile velocity of servo ID1.
    primary_servo_read_present_profile_velocity(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_profile_velocity_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present profile velocity: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present PWM of servo ID1.
    primary_servo_read_present_pwm(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_pwm_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present pwm: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present temperature of servo ID1.
    primary_servo_read_present_temperature(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_temperature_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present temperature: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the present voltage of servo ID1.
    primary_servo_read_present_voltage(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_present_voltage_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("present voltage: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the time base target moving time of servo ID1.
    primary_servo_read_time_base_target_moving_time(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_time_base_target_moving_time_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("time base target moving time: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the time base target position of servo ID1.
    primary_servo_read_time_base_target_position(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_time_base_target_position_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("time base target position: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the time base target ACC of servo ID1.
    primary_servo_read_time_base_target_acc(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_time_base_target_acc_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("time base target acc: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the time base target position and moving time of servo ID1.
    primary_servo_read(1, 0x3C, 4, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read time base target position and moving time status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the time base target ACC, position and moving time of servo ID1.
    primary_servo_read(1, 0x3B, 5, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read time base target acc, position and moving time status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target DEC of servo ID1.
    primary_servo_read_velocity_base_target_dec(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_velocity_base_target_dec_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("velocity base target dec: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target ACC of servo ID1.
    primary_servo_read_velocity_base_target_acc(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_velocity_base_target_acc_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("velocity base target acc: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target velocity of servo ID1.
    primary_servo_read_velocity_base_target_velocity(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_velocity_base_target_velocity_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("velocity base target velocity: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target position of servo ID1.
    primary_servo_read_velocity_base_target_position(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_velocity_base_target_position_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("velocity base target position: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target position and velocity of servo ID1.
    primary_servo_read(1, 0x35, 4, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read velocity base target position and velocity status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
    primary_servo_read(1, 0x35, 6, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read velocity base target position, velocity, acc and dec status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the target current of servo ID1.
    primary_servo_read_target_current(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_target_current_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("target current: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the target PWM of servo ID1.
    primary_servo_read_target_pwm(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_target_pwm_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("target pwm: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the torque switch of servo ID1.
    primary_servo_read_torque_switch(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_torque_switch_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("torque switch: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the LED switch of servo ID1.
    primary_servo_read_led_switch(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_led_switch_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("led switch: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the Flash switch of servo ID1.
    primary_servo_read_flash_switch(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_flash_switch_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("flash switch: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the calibration of servo ID1.
    primary_servo_read_calibration(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_calibration_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("calibration: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the control mode of servo ID1.
    primary_servo_read_control_mode(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_control_mode_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("control mode: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the shutdown condition of servo ID1.
    primary_servo_read_shutdown_condition(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_shutdown_condition_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("shutdown condition: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the LED condition of servo ID1.
    primary_servo_read_led_condition(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_led_condition_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("led condition: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the position control D gain of servo ID1.
    primary_servo_read_position_control_d_gain(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_position_control_d_gain_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("position control d gain: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the position control I gain of servo ID1.
    primary_servo_read_position_control_i_gain(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_position_control_i_gain_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("position control i gain: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the position control P gain of servo ID1.
    primary_servo_read_position_control_p_gain(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_position_control_p_gain_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("position control p gain: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the position control PID gain of servo ID1.
    primary_servo_read(1, 0x1B, 6, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read position control pid gain status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the PWM punch of servo ID1.
    primary_servo_read_pwm_punch(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_pwm_punch_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("pwm punch: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the ccw deadband of servo ID1.
    primary_servo_read_ccw_deadband(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_ccw_deadband_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("ccw deadband: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the cw deadband of servo ID1.
    primary_servo_read_cw_deadband(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_cw_deadband_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("cw deadband: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the current shutdown time of servo ID1.
    primary_servo_read_current_shutdown_time(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_current_shutdown_time_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("current shutdown time: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the max current limit of servo ID1.
    primary_servo_read_max_current_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_max_current_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("max current limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the max PWM limit of servo ID1.
    primary_servo_read_max_pwm_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_max_pwm_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("max pwm limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the max voltage limit of servo ID1.
    primary_servo_read_max_voltage_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_max_voltage_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("max voltage limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the min voltage limit of servo ID1.
    primary_servo_read_min_voltage_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_min_voltage_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("min voltage limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the voltage limit of servo ID1.
    primary_servo_read(1, 0x10, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read voltage limit status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the max temperature limit of servo ID1.
    primary_servo_read_max_temperature_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_max_temperature_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("max temperature limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the max angle limit of servo ID1.
    primary_servo_read_max_angle_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_max_angle_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("max angle limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the min angle limit of servo ID1.
    primary_servo_read_min_angle_limit(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_min_angle_limit_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("min angle limit: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the angle limit of servo ID1.
    primary_servo_read(1, 0x0B, 4, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("read angle limit status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the return level of servo ID1.
    primary_servo_read_return_level(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_return_level_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("return level: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the return delay time of servo ID1.
    primary_servo_read_return_delay_time(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_return_delay_time_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("return delay time: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the baud rate of servo ID1.
    primary_servo_read_baud_rate(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_baud_rate_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("baud rate: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the model information of servo ID1.
    primary_servo_read_model_information(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_model_information_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("model information: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Read the firmware version of servo ID1.
    primary_servo_read_firmware_version(1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_read_firmware_version_analysis(pack, &analysis_data);
        if (ret == PRIMARY_SUCCESS)
        {
            PRINTF("firmware version: %d\r\n", analysis_data);
        }
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);
#endif

#if WRITE_TEST
    //Change the return delay time of servo ID1 to 500us.
    primary_servo_set_return_delay_time(1, 250, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_return_delay_time_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write return delay time complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the return level of servo ID1 to respond to all instruction.
    primary_servo_set_return_level(1, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_return_level_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write return level complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the baud rate of servo ID1 to 1000000.
    primary_servo_set_baud_rate(1, 7, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_baud_rate_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write baud rate complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the min angle limit of servo ID1 to 0°.
    primary_servo_set_min_angle_limit(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_min_angle_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write min angle limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the max angle limit of servo ID1 to 300°.
    primary_servo_set_max_angle_limit(1, 3000, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_max_angle_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write max angle limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the angle limit of servo ID1 to 0°~300°.
    write_buffer[0] = 0 & 0xff;;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3000 & 0xff;
    write_buffer[3] = (3000 >> 8) & 0xff;

    primary_servo_write(1, 0x0B, 4, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write angle limit status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the max temperature limit of servo ID1 to 65℃.
    primary_servo_set_max_temperature_limit(1, 65, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_max_temperature_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write max temperature limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the max voltage limit of servo ID1 to 8.4V.
    primary_servo_set_max_voltage_limit(1, 84, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_max_voltage_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write max voltage limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the min voltage limit of servo ID1 to 3.5V.
    primary_servo_set_min_voltage_limit(1, 35, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_min_voltage_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write min voltage limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the voltage limit of servo ID1 to 3.5~8.4V.
    write_buffer[0] = 84 & 0xff;
    write_buffer[1] = 35 & 0xff;

    primary_servo_write(1, 0x10, 2, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write voltage limit status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the max PWM limit of servo ID1 to 90%.
    primary_servo_set_max_pwm_limit(1, 900, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_max_pwm_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write max pwm limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the max current limit of servo ID1 to 900mA.
    primary_servo_set_max_current_limit(1, 900, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_max_current_limit_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write max current limit complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the current shutdown time of servo ID1 to 500ms.
    primary_servo_set_current_shutdown_time(1, 500, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_current_shutdown_time_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write current shutdown time complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the CW deadband of servo ID1 to 0.2°.
    primary_servo_set_cw_deadband(1, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_cw_deadband_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write cw deadband complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the CCW deadband of servo ID1 to 0.2°.
    primary_servo_set_ccw_deadband(1, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_ccw_deadband_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write ccw deadband complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the CW and CCW deadband of servo ID1 to 0.2°.
    write_buffer[0] = 2 & 0xff;
    write_buffer[1] = 2 & 0xff;

    primary_servo_write(1, 0x18, 2, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write cw deadband and ccw deadband status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the PWM punch of servo ID1 to 1%.
    primary_servo_set_pwm_punch(1, 10, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_pwm_punch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write pwm punch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the position control P gain of servo ID1 to 5995.
    primary_servo_set_position_control_p_gain(1, 5995, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_position_control_p_gain_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write position control p gain complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the position control D gain of servo ID1 to 5.
    primary_servo_set_position_control_i_gain(1, 5, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_position_control_i_gain_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write position control i gain complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the position control D gain of servo ID1 to 145.
    primary_servo_set_position_control_d_gain(1, 145, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_position_control_d_gain_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write position control d gain complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
    write_buffer[0] = 5995 & 0xff;
    write_buffer[1] = (5995 >> 8) & 0xff;
    write_buffer[2] = 5 & 0xff;
    write_buffer[3] = (5 >> 8) & 0xff;
    write_buffer[4] = 145 & 0xff;
    write_buffer[5] = (145 >> 8) & 0xff;

    primary_servo_write(1, 0x1B, 6, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write position control pid gain status packet: ");
        for (uint8_t i = 0; i < bytesRead; i++)
        {
            PRINTF("0x%02x ", pack[i]);
        }
        PRINTF("\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
    primary_servo_set_led_condition(1, 38, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_led_condition_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write led condition complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
    primary_servo_set_shutdown_conditions(1, 39, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_shutdown_conditions_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write shutdown conditions complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the Flash switch of servo ID1 to ON.
    primary_servo_set_flash_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_flash_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write flash switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the Flash switch of servo ID1 to OFF.
    primary_servo_set_flash_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_flash_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write flash switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the LED switch of servo ID1 to ON.
    primary_servo_set_led_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_led_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write led switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the LED switch of servo ID1 to OFF.
    primary_servo_set_led_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_led_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write led switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the control mode of servo ID1 to the PWM control mode.
    primary_servo_set_control_mode(1, 3, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_control_mode_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the target PWM of servo ID1 to -50%.
    primary_servo_set_target_pwm(1, -500, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_target_pwm_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write target pwm complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(3000);

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the control mode of servo ID1 to the current control mode.
    primary_servo_set_control_mode(1, 2, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_control_mode_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the target current of servo ID1 to -400mA.
    primary_servo_set_target_current(1, -400, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_target_current_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write target current complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(3000);

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the control mode of servo ID1 to the velocity base position control mode.
    primary_servo_set_control_mode(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_control_mode_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the velocity base target velocity of servo ID1 to 360°/s.
    primary_servo_set_velocity_base_target_velocity(1, 3600, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_velocity_base_target_velocity_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target velocity complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the velocity base target ACC of servo ID1 to 500°/s².
    primary_servo_set_velocity_base_target_acc(1, 10, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_velocity_base_target_acc_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target acc complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the velocity base target DEC of servo ID1 to 50°/s².
    primary_servo_set_velocity_base_target_dec(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_velocity_base_target_dec_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target dec complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the velocity base target position of servo ID1 to 150°.
    primary_servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_velocity_base_target_position_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target position complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(1000);

    //Change the torque switch of servo ID1 to OFF.
    primary_servo_set_torque_switch(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the control mode of servo ID1 to the time base position control mode.
    primary_servo_set_control_mode(1, 0, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_control_mode_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write control mode complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the torque switch of servo ID1 to ON.
    primary_servo_set_torque_switch(1, 1, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_torque_switch_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write torque switch complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the time base target ACC of servo ID1 to 5.
    primary_servo_set_time_base_target_acc(1, 5, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_time_base_target_acc_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write time base target acc complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(20);

    //Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively.
    primary_servo_set_time_base_target_position_and_moving_time(1, 3000, 500, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_time_base_target_position_and_moving_time_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write time base target position and moving time complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(1000);
#endif

#if SYNC_WRITE_TEST
    servo.id_counts = 2;            //Sync write two servos
    servo.id[0] = 1;                //Set the ID of the first servo to 1
    servo.id[1] = 2;                //Set the ID of the second servo to 2

    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write torque witch complete\r\n");
    Sleep(20);

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write control mode complete\r\n");
    Sleep(20);

    //Change the velocity base target velocity of the servo ID1, ID2 to 360°/s² and 720°/s², respectively.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;

    primary_servo_sync_write_velocity_base_target_velocity(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target velocity complete\r\n");
    Sleep(20);

    //Change the velocity base target ACC of servo ID1, ID2 to 500°/s² and 50°/s², respectively.
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;

    primary_servo_sync_write_velocity_base_target_acc(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target acc complete\r\n");
    Sleep(20);

    //Change the velocity base target DEC of servo ID1, ID2 to 50°/s² and 500°/s², respectively.
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;

    primary_servo_sync_write_velocity_base_target_dec(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target dec complete\r\n");
    Sleep(20);

    //Change the velocity base target velocity of the servo ID1, ID2 to 150° midpoint and 0° position, respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;

    primary_servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target position complete\r\n");
    Sleep(1000);

    //Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
    servo.velocity[0] = 1800;
    servo.velocity[1] = 3600;
    servo.position[0] = 3000;
    servo.position[1] = 3000;

    primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target position and velocity complete\r\n");
    Sleep(1000);

    //SChange the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 500°/s², 500°/s², deceleration to 500°/s², 500°/s², respectively
    servo.velocity[0] = 3600;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 0;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 10;
    servo.dec_velocity[0] = 10;
    servo.dec_velocity[1] = 10;

    primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target acc, dec, velocity and position complete\r\n");
    Sleep(1000);


    //Change the torque switch of the servo ID1, ID2 to OFF respectively.
    servo.torque_switch[0] = 0;
    servo.torque_switch[1] = 0;
    primary_servo_sync_write_torque_switch(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write torque witch complete\r\n");
    Sleep(20);

    //Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
    servo.control_mode[0] = 0;
    servo.control_mode[1] = 0;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write control mode complete\r\n");
    Sleep(20);

    //Change the time base target ACC of servo ID1 to 1 and 5 respectively.
    servo.acc_velocity_grade[0] = 1;
    servo.acc_velocity_grade[1] = 5;

    primary_servo_sync_write_time_base_target_acc(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write time base target acc complete\r\n");
    Sleep(20);

    //Change the time base target position and moving time of servo ID1 to 150° midpoint and 1s, 0° and 500ms respectively.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 1000;
    servo.time[1] = 500;


    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write time base target position and moving time complete\r\n");
    Sleep(1000);
#endif

    serialPort.Close();

    return 0;
}
