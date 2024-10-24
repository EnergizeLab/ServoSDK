#ifndef _PRIMARYSERVO_H_
#define _PRIMARYSERVO_H_

#define MAX_SERVERS 20               	//Maximum number of servos for sync write.
#define PRINTF_ENABLE 1             	//Print output enable.

#if PRINTF_ENABLE
#define PRINTF printf
#else
#define PRINTF
#endif

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;
typedef short int16_t;

struct primary_servo_sync_parameter
{
    uint8_t id_counts;                           //Number of servos for sync write.
    uint8_t id[MAX_SERVERS];                     //Ids of servos for sync write.
    uint8_t torque_switch[MAX_SERVERS];          //Torque Switch
    uint8_t control_mode[MAX_SERVERS];           //Control Mode
    uint8_t acc_velocity[MAX_SERVERS];          //Target ACC
    uint8_t dec_velocity[MAX_SERVERS];          //Target DEC
    uint8_t acc_velocity_grade[MAX_SERVERS];    //Time Base Target ACC
    uint16_t position[MAX_SERVERS];              //Target Position
    uint16_t time[MAX_SERVERS];                  //Time Base Run Time
    uint16_t velocity[MAX_SERVERS];              //Target Velocity
};

//Basic instruction types
#define PRIMARY_PING                0x01    //Used to query the servo. The servo will return the present status and the model number of the servo.
#define PRIMARY_READ_DATA           0x02    //Read data from the servo's memory table
#define PRIMARY_WRITE_DATA          0x03    //Write data on the Servo Memory Table.
#define PRIMARY_SYNC_WRITE          0X83    //A single Sync Write Instruction allows data to be written in memory tables of multiple servos simultaneously.
#define PRIMARY_FACTORY_RESET       0x06    //Reset the entire memory table of the servo to the factory default values.
#define PRIMARY_PARAMETER_RESET     0x10    //Reset the parameter settings in the servo's memory table to the factory default values.
#define PRIMARY_CALIBRATION         0x15    //Calibrate the offset value of the servo midpoint (150 ° position).
#define PRIMARY_REBOOT              0x64    //Used to reboot servo.

#define PRIMARY_SUCCESS             0x00     //No errors
#define PRIMARY_FAILURE             0x01     //Instruction types error

#define PRIMARY_VOLTAGE_ERROR           0x01    //Input voltage of the servo is out of range for the operating voltage set in the memory table.
#define PRIMARY_ANGLE_ERROR             0x02    //Target position of the servo is out of range for the maximum and minimum target positions set in the memory table.
#define PRIMARY_OVERHEATING_ERROR       0x04    //Internal temperature of the servo is over the upper limit set in the Memory Table.
#define PRIMARY_RANGE_ERROR             0x08    //Length of Instruction Packet out of range for the specified length.
#define PRIMARY_CHECKSUM_ERROR          0x10    //CheckSum of the Instruction Packet is parsed incorrectly.
#define PRIMARY_STALL_ERROR             0x20    //Servo occurs in a stall due to overcurrent or overload etc.
#define PRIMARY_PARSING_ERROR           0x40    //Instruction Packet does not comply with the protocol format, resulting in a parsing error.
#define PRIMARY_UNPACK_ERROR            0x80    //This is not a complete response package

//Memory address
#define PRIMARY_MODEL_NUMBER                0x00    //S/N Code
#define PRIMARY_FIRMWARE_VERSION            0x02    //S/N Code
#define PRIMARY_MODEL_INFORMATION           0x03    //S/N Code
#define PRIMARY_SERVO_ID                    0x07    //Servo ID
#define PRIMARY_BAUD_RATE                   0x08    //Baud Rate Number 7 is 1Mbps
#define PRIMARY_RETURN_DELAY_TIME           0x09    //Response Delay Time for servo return to packet
#define PRIMARY_RETURN_LEVEL                0x0A    //Returned level of servo status.
#define PRIMARY_MIN_ANGLE_LIMIT_L           0x0B    //Min Angle Limit for servo rotation
#define PRIMARY_MAX_ANGLE_LIMIT_L           0x0D    //Max Voltage Limit for operating Servo
#define PRIMARY_MAX_TEMPERATURE_LIMIT       0x0F    //Maximum Temperature Limit for operating servo
#define PRIMARY_MAX_VOLTAGE_LIMIT           0x10    //Maximum Voltage Limit for operating Servo
#define PRIMARY_MIN_VOLTAGE_LIMIT           0x11    //Minimum Voltage Limit for operating Servo
#define PRIMARY_MAX_PWM_LIMIT_L             0x12    //Max PWM Limit Limit of servo
#define PRIMARY_MAX_CURRENT_LIMIT_L         0x14    //Max Current Limit for operating servo
#define PRIMARY_CURRENT_TIME_L              0x16    //Trigger Time for overload protection activation after reaching current limit
#define PRIMARY_CW_DEADBAND                 0x18    //Dead Band for clockwise direction
#define PRIMARY_CCW_DEADBAND                0x19    //Dead Band for counterclockwise direction
#define PRIMARY_PWM_PUNCH                   0x1A    //PWM punch of the servo output.
#define PRIMARY_POSITION_P_L                0x1B    //Gain Proportion (P) of Servo's PID Control
#define PRIMARY_POSITION_I_L                0x1D    //Gain Integration (I) of Servo's PID Control
#define PRIMARY_POSITION_D_L                0x1F    //Gain Differential (D) of Servo's PID Control
#define PRIMARY_LED_CONDITION               0x21    //Conditions for Alarm LED
#define PRIMARY_SHUTDOWN_CONDITION          0x22    //Conditions for torque unloading
#define PRIMARY_CONTROL_MODE                0x23    //Servo Control Mode: 0 Time Base Position Control, 1 Velocity Base Position Control, 2 Current, 3 PWM
#define PRIMARY_CALIBRATION_L               0x24    //Offset Value for midpoint Calibration of Servo.This bit is triggered by the Calibration instruction, and the value cannot be directly written.
#define PRIMARY_FLASH_SW                    0x2E    //Flash area write switch: 0 for write disabled, 1 for write enabled.
#define PRIMARY_LED_SW                      0x2F    //Servo indicator light switch: 0 for off, 1 for on.
#define PRIMARY_TORQUE_SW                   0x30    //Servo torque switch: 0 for torque disabled, 1 for torque enabled, 2 for brake mode.
#define PRIMARY_TARGET_PWM_L                0x31    //Direct control of PWM output to the motor.
#define PRIMARY_TARGET_CURRENT_L            0x33    //Target current for servo operation, by default, equal to the default value of the max current limit.
#define PRIMARY_VELOCITY_BASE_TARGET_POSITION_L     0x35    //Used in Velocity Base Position Control Mode.
#define PRIMARY_VELOCITY_BASE_TARGET_VELOCITY_L     0x37    //Used in Velocity Base Position Control Mode.
#define PRIMARY_VELOCITY_BASE_TARGET_ACC            0x39    //Used in Velocity Base Position Control Mode.
#define PRIMARY_VELOCITY_BASE_TARGET_DEC            0x3A    //Used in Velocity Base Position Control Mode.
#define PRIMARY_TIME_BASE_TARGET_ACC               0x3B    //Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
#define PRIMARY_TIME_BASE_TARGET_POSITION_L        0x3C    //Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
#define PRIMARY_TIME_BASE_TARGET_MOVINGTIME_L      0x3E    //Used in Time Base Position Control Mode. Target position and moving time must be written simultaneously.
#define PRIMARY_PRESENT_VOLTAGE             0x40    //Actual voltage at which the servo is currently operating.
#define PRIMARY_PRESENT_TEMPERATURE         0x41    //Actual internal temperature of the servo.
#define PRIMARY_PRESENT_PWM_L               0x42    //Present PWM value being output by the servo.
#define PRIMARY_PRESENT_PROFILE_VELOCITY_L  0x44    //Present profile velocity of the Profile Planner.
#define PRIMARY_PRESENT_PROFILE_POSITION_L  0x46    //Present profile position of the Profile Planner.
#define PRIMARY_PRESENT_VELOCITY_L          0x48    //Present actual velocity of the Servo
#define PRIMARY_PRESENT_POSITION_L          0x4A    //Present actual position of the Servo
#define PRIMARY_PRESENT_CURRENT_L           0x4C    //Present actual current of the Servo

//Command packaging
uint8_t primary_servo_pack(uint8_t id, uint8_t instruction, uint8_t address, uint8_t byte_length, const uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_length);

//Reply packet unpack
uint8_t primary_servo_unpack(uint8_t* response_packet, uint8_t** data_buffer);

//PING
uint8_t primary_servo_ping(uint8_t id, uint8_t* output_buffer, uint8_t* len);

//Read instruction
uint8_t primary_servo_read(uint8_t id, uint8_t address, uint8_t read_data_len, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Write instruction
uint8_t primary_servo_write(uint8_t id, uint8_t address, uint8_t write_data_len, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Sync write instruction
uint8_t primary_sync_write_data(uint8_t address, uint8_t servo_counts, uint8_t* input_buffer, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Factory reset instruction
uint8_t primary_servo_factory_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Parameter reset instruction
uint8_t primary_servo_parameter_reset(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Calibration instruction
uint8_t primary_servo_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Reboot instruction
uint8_t primary_servo_reboot(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Change the servo ID
uint8_t primary_servo_modify_known_id(uint8_t id, uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the servo ID of the servo with an unknown ID
uint8_t primary_servo_modify_unknown_id(uint8_t new_id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the return delay time of the servo
uint8_t primary_servo_set_return_delay_time(uint8_t id, uint8_t response_delay_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the return level of servo
uint8_t primary_servo_set_return_level(uint8_t id, uint8_t return_level, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the baud rate of servo
uint8_t primary_servo_set_baud_rate(uint8_t id, uint8_t baud_rate_number, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the min angle limit of servo
uint8_t primary_servo_set_min_angle_limit(uint8_t id, uint16_t min_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the max angle limit of servo
uint8_t primary_servo_set_max_angle_limit(uint8_t id, uint16_t max_angle_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the max temperature limit of servo
uint8_t primary_servo_set_max_temperature_limit(uint8_t id, uint8_t max_temperature_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the max voltage limit of servo
uint8_t primary_servo_set_max_voltage_limit(uint8_t id, uint8_t max_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the min voltage limit of servo
uint8_t primary_servo_set_min_voltage_limit(uint8_t id, uint8_t min_voltage_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the max PWM limit of servo
uint8_t primary_servo_set_max_pwm_limit(uint8_t id, uint16_t max_pwm_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the max current limit of servo
uint8_t primary_servo_set_max_current_limit(uint8_t id, uint16_t max_current_limit, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the current shutdown time of servo
uint8_t primary_servo_set_current_shutdown_time(uint8_t id, uint16_t current_shutdown_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the CW deadband of servo
uint8_t primary_servo_set_cw_deadband(uint8_t id, uint8_t cw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the CCW deadband of servo
uint8_t primary_servo_set_ccw_deadband(uint8_t id, uint8_t ccw_deadband, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the PWM punch of servo
uint8_t primary_servo_set_pwm_punch(uint8_t id, uint8_t pwm_punch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the position control P gain of servo
uint8_t primary_servo_set_position_control_p_gain(uint8_t id, uint16_t position_control_P_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the position control I gain of servo
uint8_t primary_servo_set_position_control_i_gain(uint8_t id, uint16_t position_control_I_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the position control D gain of servo
uint8_t primary_servo_set_position_control_d_gain(uint8_t id, uint16_t position_control_D_gain, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the LED condition of servo
uint8_t primary_servo_set_led_condition(uint8_t id, uint8_t led_condition, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the shutdown condition of servo
uint8_t primary_servo_set_shutdown_conditions(uint8_t id, uint8_t shutdown_conditions, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the control mode of servo
uint8_t primary_servo_set_control_mode(uint8_t id, uint8_t control_mode, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the Flash switch of servo
uint8_t primary_servo_set_flash_switch(uint8_t id, uint8_t flash_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the LED switch of servo
uint8_t primary_servo_set_led_switch(uint8_t id, uint8_t led_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the torque switch of servo
uint8_t primary_servo_set_torque_switch(uint8_t id, uint8_t torque_switch, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target PWM of servo
uint8_t primary_servo_set_target_pwm(uint8_t id, int16_t target_pwm, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target current of servo
uint8_t primary_servo_set_target_current(uint8_t id, int16_t target_current, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the velocity base target position of servo
uint8_t primary_servo_set_velocity_base_target_position(uint8_t id, uint16_t target_position, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the velocity base target velocity of servo
uint8_t primary_servo_set_velocity_base_target_velocity(uint8_t id, uint16_t target_velocity, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the velocity base target ACC of servo
uint8_t primary_servo_set_velocity_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the velocity base target DEC of servo
uint8_t primary_servo_set_velocity_base_target_dec(uint8_t id, uint8_t target_dec, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the time base target ACC of servo
uint8_t primary_servo_set_time_base_target_acc(uint8_t id, uint8_t target_acc, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the time base target position and moving time of servo
uint8_t primary_servo_set_time_base_target_position_and_moving_time(uint8_t id, uint16_t target_position, uint16_t moving_time, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present position and present current of servo
uint8_t primary_servo_read_present_position_and_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present current of servo
uint8_t primary_servo_read_present_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present position of servo
uint8_t primary_servo_read_present_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present velocity of servo
uint8_t primary_servo_read_present_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present profile position of servo
uint8_t primary_servo_read_present_profile_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present profile velocity of servo
uint8_t primary_servo_read_present_profile_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present PWM of servo
uint8_t primary_servo_read_present_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present temperature of servo
uint8_t primary_servo_read_present_temperature(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the present voltage of servo
uint8_t primary_servo_read_present_voltage(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the time base target moving time of servo
uint8_t primary_servo_read_time_base_target_moving_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the time base target position of servo
uint8_t primary_servo_read_time_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the time base target ACC of servo
uint8_t primary_servo_read_time_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the velocity base target DEC of servo
uint8_t primary_servo_read_velocity_base_target_dec(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the velocity base target ACC of servo
uint8_t primary_servo_read_velocity_base_target_acc(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the velocity base target velocity of servo
uint8_t primary_servo_read_velocity_base_target_velocity(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the velocity base target position of servo
uint8_t primary_servo_read_velocity_base_target_position(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the target current of servo
uint8_t primary_servo_read_target_current(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the target PWM of servo
uint8_t primary_servo_read_target_pwm(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the torque switch of servo
uint8_t primary_servo_read_torque_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the LED switch of servo
uint8_t primary_servo_read_led_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the Flash switch of servo
uint8_t primary_servo_read_flash_switch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the calibration of servo
uint8_t primary_servo_read_calibration(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the control mode of servo
uint8_t primary_servo_read_control_mode(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the shutdown condition of servo
uint8_t primary_servo_read_shutdown_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the LED condition of servo
uint8_t primary_servo_read_led_condition(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the position control D gain of servo
uint8_t primary_servo_read_position_control_d_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the position control I gain of servo
uint8_t primary_servo_read_position_control_i_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the position control P gain of servo
uint8_t primary_servo_read_position_control_p_gain(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the PWM punch of servo
uint8_t primary_servo_read_pwm_punch(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the CCW deadband of servo
uint8_t primary_servo_read_ccw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the CW deadband of servo
uint8_t primary_servo_read_cw_deadband(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the current shutdown time of servo
uint8_t primary_servo_read_current_shutdown_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the max current limit of servo
uint8_t primary_servo_read_max_current_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the max PWM limit of servo
uint8_t primary_servo_read_max_pwm_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the max voltage limit of servo
uint8_t primary_servo_read_max_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the min voltage limit of servo
uint8_t primary_servo_read_min_voltage_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the max temperature limit of servo
uint8_t primary_servo_read_max_temperature_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the max angle limit of servo
uint8_t primary_servo_read_max_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the min angle limit of servo
uint8_t primary_servo_read_min_angle_limit(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the return level of servo
uint8_t primary_servo_read_return_level(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the return delay time of servo
uint8_t primary_servo_read_return_delay_time(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the baud rate of servo
uint8_t primary_servo_read_baud_rate(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the model information of servo
uint8_t primary_servo_read_model_information(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Read the firmware version of servo
uint8_t primary_servo_read_firmware_version(uint8_t id, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set torque switches for multiple servos
uint8_t primary_servo_sync_write_torque_switch(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the control mode for multiple servos
uint8_t primary_servo_sync_write_control_mode(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target acceleration, deceleration, speed, and position for multiple servos.
uint8_t primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Change the velocity base target position for multiple servos
uint8_t primary_servo_sync_write_velocity_base_target_position(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Change the velocity base target velocity for multiple servos
uint8_t primary_servo_sync_write_velocity_base_target_velocity(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target position and speed for multiple servos
uint8_t primary_servo_sync_write_velocity_base_target_position_and_velocity(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target acceleration for multiple servos
uint8_t primary_servo_sync_write_velocity_base_target_acc(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Set the target deceleration for multiple servos
uint8_t primary_servo_sync_write_velocity_base_target_dec(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Change the time base target ACC for multiple servos
uint8_t primary_servo_sync_write_time_base_target_acc(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Change the time base target position and moving time for multiple servos
uint8_t primary_servo_sync_write_time_base_target_position_and_moving_time(struct primary_servo_sync_parameter servo, uint8_t* output_buffer, uint8_t* output_buffer_len);

//Parsing the servo response packet for the PING command
uint8_t primary_servo_ping_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the factory reset command
uint8_t primary_servo_factory_reset_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the parameter reset command
uint8_t primary_servo_parameter_reset_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the calibration command
uint8_t primary_servo_calibration_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the return delay time
uint8_t primary_servo_set_return_delay_time_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the return level
uint8_t primary_servo_set_return_level_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the baud rate
uint8_t primary_servo_set_baud_rate_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the min angle limit
uint8_t primary_servo_set_min_angle_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the max limit analysis
uint8_t primary_servo_set_max_angle_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the max temperature limit
uint8_t primary_servo_set_max_temperature_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the max voltage limit
uint8_t primary_servo_set_max_voltage_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the min voltage limit
uint8_t primary_servo_set_min_voltage_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the max pwm limit
uint8_t primary_servo_set_max_pwm_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the max current limit
uint8_t primary_servo_set_max_current_limit_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the current shutdown time
uint8_t primary_servo_set_current_shutdown_time_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the cw deadband
uint8_t primary_servo_set_cw_deadband_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the ccw deadband
uint8_t primary_servo_set_ccw_deadband_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the pwm punch
uint8_t primary_servo_set_pwm_punch_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the position control p gain
uint8_t primary_servo_set_position_control_p_gain_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the position control i gain
uint8_t primary_servo_set_position_control_i_gain_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the position control d gain
uint8_t primary_servo_set_position_control_d_gain_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the led condition
uint8_t primary_servo_set_led_condition_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the shutdown conditions
uint8_t primary_servo_set_shutdown_conditions_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the control mode
uint8_t primary_servo_set_control_mode_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the flash switch
uint8_t primary_servo_set_flash_switch_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the led switch
uint8_t primary_servo_set_led_switch_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the torque switch
uint8_t primary_servo_set_torque_switch_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the target pwm
uint8_t primary_servo_set_target_pwm_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the target current
uint8_t primary_servo_set_target_current_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the velocity base target position
uint8_t primary_servo_set_velocity_base_target_position_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the velocity base target velocity
uint8_t primary_servo_set_velocity_base_target_velocity_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the velocity base target accelerated speed
uint8_t primary_servo_set_velocity_base_target_acc_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the velocity base target deceleration
uint8_t primary_servo_set_velocity_base_target_dec_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the time base target accelerated speed
uint8_t primary_servo_set_time_base_target_acc_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the time base target position and moving time
uint8_t primary_servo_set_time_base_target_position_and_moving_time_analysis(uint8_t* response_packet);

//Parsing the servo response packet for the present current
uint8_t primary_servo_read_present_current_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present position
uint8_t primary_servo_read_present_position_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present position and current
uint8_t primary_servo_read_present_position_and_present_current_analysis(uint8_t* response_packet, uint16_t* position, uint16_t* current);

//Parsing the servo response packet for the present velocity
uint8_t primary_servo_read_present_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present profile position
uint8_t primary_servo_read_present_profile_position_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present profile velocity
uint8_t primary_servo_read_present_profile_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present pwm
uint8_t primary_servo_read_present_pwm_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present temperature
uint8_t primary_servo_read_present_temperature_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the present voltage
uint8_t primary_servo_read_present_voltage_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the time base target moving time
uint8_t primary_servo_read_time_base_target_moving_time_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the time base target position
uint8_t primary_servo_read_time_base_target_position_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the time base target accelerated speed
uint8_t primary_servo_read_time_base_target_acc_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the velocity base target deceleration
uint8_t primary_servo_read_velocity_base_target_dec_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the velocity base target accelerated speed
uint8_t primary_servo_read_velocity_base_target_acc_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the velocity base target velocity
uint8_t primary_servo_read_velocity_base_target_velocity_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the velocity base target position
uint8_t primary_servo_read_velocity_base_target_position_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the target current
uint8_t primary_servo_read_target_current_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the target pwm
uint8_t primary_servo_read_target_pwm_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the torque switch
uint8_t primary_servo_read_torque_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the led switch
uint8_t primary_servo_read_led_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the flash switch
uint8_t primary_servo_read_flash_switch_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the current offset
uint8_t primary_servo_read_current_offset_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the calibration
uint8_t primary_servo_read_calibration_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the control mode
uint8_t primary_servo_read_control_mode_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the shutdown condition
uint8_t primary_servo_read_shutdown_condition_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the led condition
uint8_t primary_servo_read_led_condition_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the position control d gain
uint8_t primary_servo_read_position_control_d_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the position control i gain
uint8_t primary_servo_read_position_control_i_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the position control p gain
uint8_t primary_servo_read_position_control_p_gain_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the pwm punch
uint8_t primary_servo_read_pwm_punch_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the ccw deadband
uint8_t primary_servo_read_ccw_deadband_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the cw deadband
uint8_t primary_servo_read_cw_deadband_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the current shutdown time
uint8_t primary_servo_read_current_shutdown_time_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the max current limit
uint8_t primary_servo_read_max_current_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the max pwm limit
uint8_t primary_servo_read_max_pwm_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the max voltage limit
uint8_t primary_servo_read_max_voltage_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the min voltage limit
uint8_t primary_servo_read_min_voltage_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the max temperature limit
uint8_t primary_servo_read_max_temperature_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the max angle limit
uint8_t primary_servo_read_max_angle_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the min angle limit
uint8_t primary_servo_read_min_angle_limit_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the return level
uint8_t primary_servo_read_return_level_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the return delay time
uint8_t primary_servo_read_return_delay_time_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the baud rate
uint8_t primary_servo_read_baud_rate_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the model information
uint8_t primary_servo_read_model_information_analysis(uint8_t* response_packet, uint32_t* analysis_data);

//Parsing the servo response packet for the firmware version
uint8_t primary_servo_read_firmware_version_analysis(uint8_t* response_packet, uint32_t* analysis_data);

#endif
