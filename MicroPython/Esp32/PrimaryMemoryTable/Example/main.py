import micropython
from machine import Pin, UART
import time
from PrimaryServo import *

PING_TEST = 0  # PING Instruction Test
READ_TEST = 0  # Read Servo Data Test
FACTORY_RESET_TEST = 0  # Factory Reset Test
PARAMETER_RESET_TEST = 0  # Parameter Reset Test
CALIBRATION_TEST = 0  # Calibration Test
REBOOT_TEST = 0  # Reboot Test
WRITE_TEST = 0  # Sync Write Test
SYNC_WRITE = 0  # Sync Write Test
MODIFY_ID = 0  # Change Known Servo ID Test
MODIFY_UNKNOWN_ID = 0  # Change Unknown Servo ID Test

ret = 0   # Status Flag
output_buffer = bytearray(40)  # Store Generated Instructions
output_buffer_len = [0]  # Instruction Length
receive_data = bytearray(40)  # Store the received status packet
receive_data_len = 0  # Length of received data.
analysis_data = [0]  # Data parsed from the status packet
position = [0]    # Present position of the servo
current = [0]    # Present current of the servo
write_buffer = bytearray(20)    # Write data to the memory table

servo_sync_parameter = Primary_Servo_Sync_Parameter()  # Create sync write memory table class.

# Configure serial port 2 (UART2).
uart2 = UART(2, baudrate=1000000, tx=17, rx=16)

dir = Pin(4, Pin.OUT)    # It is used to control the uart transmission direction

while True:
    # Reset the servo to the factory default values.
    if FACTORY_RESET_TEST:
        Primary_Servo.servo_factory_reset(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_factory_reset_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("factory reset complete")
        time.sleep(1)

    # Reset the parameter settings of the servo.
    if PARAMETER_RESET_TEST:
        Primary_Servo.servo_parameter_reset(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_parameter_reset_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("parameter reset complete")
        time.sleep(1)

    # Calibrate the midpoint of the servo.
    if CALIBRATION_TEST:
        Primary_Servo.servo_calibration(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_calibration_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("calibration complete")
        time.sleep(1)

    # Reboot the servo.
    if REBOOT_TEST:
        Primary_Servo.servo_reboot(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        time.sleep_ms(1)
        time.sleep(1)

    # Change the servo ID of servo ID1 to 2.
    if MODIFY_ID:
        Primary_Servo.servo_modify_known_id(1, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        time.sleep_ms(1)
        time.sleep(1)

    # Change the servo ID of the servo with an unknown ID to 1.
    if MODIFY_ID:
        Primary_Servo.servo_modify_unknown_id(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        time.sleep_ms(1)
        time.sleep(1)
        
    # Query the model number of servo ID1.
    if PING_TEST:
        Primary_Servo.servo_ping(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_ping_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("model number:", analysis_data[0])
        time.sleep(1)
    
    # Read the servo data.
    if READ_TEST:
        # Read the present current of servo ID1.
        Primary_Servo.servo_read_present_current(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_current_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present current:", analysis_data[0])
        time.sleep(1)

        # Read the present position of servo ID1.
        Primary_Servo.servo_read_present_position(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_position_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present position:", analysis_data[0])
        time.sleep(1)
        
        # Read the present position and present current of servo ID1.
        Primary_Servo.servo_read_present_position(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_position_and_present_current_analysis(receive_data, position, current)
        if ret == Primary_State.SUCCESS:
            print(f"present position: {position[0]}, present current: {current[0]}")
        time.sleep(1)

        # Read the present velocity of servo ID1.
        Primary_Servo.servo_read_present_velocity(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_velocity_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present velocity:", analysis_data[0])
        time.sleep(1)

        # Read the present profile position of servo ID1.
        Primary_Servo.servo_read_present_profile_position(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_profile_position_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present profile position:", analysis_data[0])
        time.sleep(1)

        # Read the present profile velocity of servo ID1.
        Primary_Servo.servo_read_present_profile_velocity(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_profile_velocity_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present profile velocity:", analysis_data[0])
        time.sleep(1)

        # Read the present PWM of servo ID1.
        Primary_Servo.servo_read_present_pwm(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_pwm_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present pwm:", analysis_data[0])
        time.sleep(1)

        # Read the present temperature of servo ID1.
        Primary_Servo.servo_read_present_temperature(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_temperature_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present temperature:", analysis_data[0])
        time.sleep(1)

        # Read the present voltage of servo ID1.
        Primary_Servo.servo_read_present_voltage(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_present_voltage_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("present voltage:", analysis_data[0])
        time.sleep(1)

        # Read the time base target moving time of servo ID1.
        Primary_Servo.servo_read_time_base_target_moving_time(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_time_base_target_moving_time_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("time base target moving time:", analysis_data[0])
        time.sleep(1)

        # Read the time base target position of servo ID1.
        Primary_Servo.servo_read_time_base_target_position(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_time_base_target_position_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("time base target position:", analysis_data[0])
        time.sleep(1)

        # Read the time base target ACC of servo ID1.
        Primary_Servo.servo_read_time_base_target_acc(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_time_base_target_acc_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("time base target acc:", analysis_data[0])
        time.sleep(1)
        
        # Read the time base target position and moving time of servo ID1.
        Primary_Servo.servo_read(1, 0x3C, 4, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read time base target position and moving time status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)
        
        # Read the time base target ACC, position and moving time of servo ID1.
        Primary_Servo.servo_read(1, 0x3B, 5, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read time base target acc, position and moving time status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Read the velocity base target DEC of servo ID1.
        Primary_Servo.servo_read_velocity_base_target_dec(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_velocity_base_target_dec_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("velocity base target dec:", analysis_data[0])
        uart2.flush()
        time.sleep(1)

        # Read the velocity base target ACC of servo ID1.
        Primary_Servo.servo_read_velocity_base_target_acc(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_velocity_base_target_acc_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("velocity base target acc:", analysis_data[0])
        uart2.flush()
        time.sleep(1)

        # Read the velocity base target velocity of servo ID1.
        Primary_Servo.servo_read_velocity_base_target_velocity(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_velocity_base_target_velocity_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("velocity base target velocity:", analysis_data[0])
        time.sleep(1)

        # Read the velocity base target position of servo ID1.
        Primary_Servo.servo_read_velocity_base_target_position(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_velocity_base_target_position_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("velocity base target position:", analysis_data[0])
        time.sleep(1)
        
        # Read the velocity base target position and velocity of servo ID1.
        Primary_Servo.servo_read(1, 0x35, 4, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read velocity base target position and velocity status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)
        
        # Read the velocity base target position, velocity, ACC, and DEC of servo ID1.
        Primary_Servo.servo_read(1, 0x35, 6, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read velocity base target position, velocity, acc and dec status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Read the target current of servo ID1.
        Primary_Servo.servo_read_target_current(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_target_current_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("target current:", analysis_data[0])
        time.sleep(1)

        # Read the target PWM of servo ID1.
        Primary_Servo.servo_read_target_pwm(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_target_pwm_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("target pwm:", analysis_data[0])
        time.sleep(1)

        # Read the torque switch of servo ID1.
        Primary_Servo.servo_read_torque_switch(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_torque_switch_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("torque switch:", analysis_data[0])
        time.sleep(1)

        # Read the LED switch of servo ID1.
        Primary_Servo.servo_read_led_switch(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_led_switch_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("led switch:", analysis_data[0])
        time.sleep(1)

        # Read the Flash switch of servo ID1.
        Primary_Servo.servo_read_flash_switch(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_flash_switch_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("flash switch:", analysis_data[0])
        time.sleep(1)

        # Read the calibration of servo ID1.
        Primary_Servo.servo_read_calibration(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_calibration_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("calibration:", analysis_data[0])
        time.sleep(1)

        # Read the control mode of servo ID1.
        Primary_Servo.servo_read_control_mode(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_control_mode_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("control mode:", analysis_data[0])
        time.sleep(1)

        # Read the shutdown condition of servo ID1.
        Primary_Servo.servo_read_shutdown_condition(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_shutdown_condition_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("shutdown condition:", analysis_data[0])
        time.sleep(1)

        # Read the LED condition of servo ID1.
        Primary_Servo.servo_read_led_condition(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_led_condition_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("led condition:", analysis_data[0])
        time.sleep(1)

        # Read the position control D gain of servo ID1.
        Primary_Servo.servo_read_position_control_d_gain(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_position_control_d_gain_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("position control d gain:", analysis_data[0])
        time.sleep(1)

        # Read the position control I gain of servo ID1.
        Primary_Servo.servo_read_position_control_i_gain(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_position_control_i_gain_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("position control i gain:", analysis_data[0])
        time.sleep(1)

        # Read the position control P gain of servo ID1.
        Primary_Servo.servo_read_position_control_p_gain(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_position_control_p_gain_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("position control p gain:", analysis_data[0])
        time.sleep(1)
        
         # Read the position control PID gain of servo ID1.
        Primary_Servo.servo_read(1, 0x1B, 6, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read position control pid gain status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Read the PWM punch of servo ID1.
        Primary_Servo.servo_read_pwm_punch(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_pwm_punch_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("pwm punch:", analysis_data[0])
        time.sleep(1)

        # Read the ccw deadband of servo ID1.
        Primary_Servo.servo_read_ccw_deadband(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_ccw_deadband_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("ccw deadband:", analysis_data[0])
        time.sleep(1)

        # Read the cw deadband of servo ID1.
        Primary_Servo.servo_read_cw_deadband(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_cw_deadband_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("cw deadband:", analysis_data[0])
        time.sleep(1)

        # Read the current shutdown time of servo ID1.
        Primary_Servo.servo_read_current_shutdown_time(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_current_shutdown_time_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("current shutdown time:", analysis_data[0])
        time.sleep(1)

        # Read the max current limit of servo ID1.
        Primary_Servo.servo_read_max_current_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_max_current_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("max current limit:", analysis_data[0])
        time.sleep(1)

        # Read the max PWM limit of servo ID1.
        Primary_Servo.servo_read_max_pwm_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_max_pwm_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("max pwm limit:", analysis_data[0])
        time.sleep(1)

        # Read the max voltage limit of servo ID1.
        Primary_Servo.servo_read_max_voltage_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_max_voltage_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("max voltage limit:", analysis_data[0])
        time.sleep(1)

        # Read the min voltage limit of servo ID1.
        Primary_Servo.servo_read_min_voltage_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_min_voltage_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("min voltage limit:", analysis_data[0])
        time.sleep(1)
        
        # Read the voltage limit of servo ID1.
        Primary_Servo.servo_read(1, 0x10, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read voltage limit status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Read the max temperature limit of servo ID1.
        Primary_Servo.servo_read_max_temperature_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_max_temperature_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("max temperature limit:", analysis_data[0])
        time.sleep(1)

        # Read the max angle limit of servo ID1.
        Primary_Servo.servo_read_max_angle_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_max_angle_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("max angle limit:", analysis_data[0])
        time.sleep(1)

        # Read the min angle limit of servo ID1.
        Primary_Servo.servo_read_min_angle_limit(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_min_angle_limit_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("min angle limit:", analysis_data[0])
        time.sleep(1)
        
        # Read the angle limit of servo ID1.
        Primary_Servo.servo_read(1, 0x0B, 4, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("read angle limit status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Read the return level of servo ID1.
        Primary_Servo.servo_read_return_level(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_return_level_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("return level:", analysis_data[0])
        time.sleep(1)

        # Read the return delay time of servo ID1.
        Primary_Servo.servo_read_return_delay_time(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_return_delay_time_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("return delay time:", analysis_data[0])
        time.sleep(1)

        # Read the baud rate of servo ID1.
        Primary_Servo.servo_read_baud_rate(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_baud_rate_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("baud rate:", analysis_data[0])
        time.sleep(1)

        # Read the model information of servo ID1.
        Primary_Servo.servo_read_model_information(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_model_information_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("model information:", analysis_data[0])
        time.sleep(1)

        # Read the firmware version of servo ID1.
        Primary_Servo.servo_read_firmware_version(1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_read_firmware_version_analysis(receive_data, analysis_data)
        if ret == Primary_State.SUCCESS:
            print("firmware version:", analysis_data[0])
        time.sleep(1)

    # Write the servo data.
    if WRITE_TEST:
        # Change the return delay time of servo ID1 to 500us.
        Primary_Servo.servo_set_return_delay_time(1, 250, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_return_delay_time_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write return delay time complete")
        time.sleep(1)

        # Change the return level of servo ID1 to respond to all instruction.
        Primary_Servo.servo_set_return_level(1, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_return_level_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write return level complete")
        time.sleep(1)

        # Change the baud rate of servo ID1 to 1000000.
        Primary_Servo.servo_set_baud_rate(1, 7, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_baud_rate_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write baud rate complete")
        time.sleep(1)

        # Change the min angle limit of servo ID1 to 0°.
        Primary_Servo.servo_set_min_angle_limit(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_min_angle_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write min angle limit complete")
        time.sleep(1)

        # Change the max angle limit of servo ID1 to 300°.°
        Primary_Servo.servo_set_max_angle_limit(1, 3000, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_max_angle_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write max angle limit complete")
        time.sleep(1)
        
        # Change the angle limit of servo ID1 to 0°~300°.
        write_buffer[0] = 0 & 0xff
        write_buffer[1] = (0 >> 8) & 0xff
        write_buffer[2] = 3000 & 0xff
        write_buffer[3] = (3000 >> 8) & 0xff
        
        Primary_Servo.servo_write(1, 0x0B, 4, write_buffer, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("write angle limit status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)
        
        # Change the max temperature limit of servo ID1 to 65℃.
        Primary_Servo.servo_set_max_temperature_limit(1, 65, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_max_temperature_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write max temperature limit complete")
        time.sleep(1)

        # Change the max voltage limit of servo ID1 to 8.4V.
        Primary_Servo.servo_set_max_voltage_limit(1, 84, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_max_voltage_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write max voltage limit complete")
        time.sleep(1)

        # Change the min voltage limit of servo ID1 to 3.5V.
        Primary_Servo.servo_set_min_voltage_limit(1, 35, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_min_voltage_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write min voltage limit complete")
        time.sleep(1)
        
        # Change the voltage limit of servo ID1 to 3.5~8.4V.
        write_buffer[0] = 84 & 0xff
        write_buffer[1] = 35 & 0xff

        Primary_Servo.servo_write(1, 0x10, 2, write_buffer, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("write voltage limit status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Change the max PWM limit of servo ID1 to 90%.
        Primary_Servo.servo_set_max_pwm_limit(1, 900, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_max_pwm_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write max pwm limit complete")
        time.sleep(1)

        # Change the max current limit of servo ID1 to 900mA.
        Primary_Servo.servo_set_max_current_limit(1, 900, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_max_current_limit_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write max current limit complete")
        time.sleep(1)

        # Change the current shutdown time of servo ID1 to 500ms.
        Primary_Servo.servo_set_current_shutdown_time(1, 500, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_current_shutdown_time_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write current shutdown time complete")
        time.sleep(1)

        # Change the CW deadband of servo ID1 to 0.2°.
        Primary_Servo.servo_set_cw_deadband(1, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_cw_deadband_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write cw deadband complete")
        time.sleep(1)

        # Change the CCW deadband of servo ID1 to 0.2°.
        Primary_Servo.servo_set_ccw_deadband(1, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_ccw_deadband_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write ccw deadband complete")
        time.sleep(1)
        
        # Change the CW and CCW deadband of servo ID1 to 0.2°.
        write_buffer[0] = 2 & 0xff
        write_buffer[1] = 2 & 0xff

        Primary_Servo.servo_write(1, 0x18, 2, write_buffer, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("write cw deadband and ccw deadband status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")

        # Change the PWM punch of servo ID1 to 1%.
        Primary_Servo.servo_set_pwm_punch(1, 10, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_pwm_punch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write pwm punch complete")
        time.sleep(1)

        # Change the position control P gain of servo ID1 to 5995.
        Primary_Servo.servo_set_position_control_p_gain(1, 5995, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_position_control_p_gain_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write position control p gain complete")
        time.sleep(1)

        # Change the position control D gain of servo ID1 to 5.
        Primary_Servo.servo_set_position_control_i_gain(1, 5, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_position_control_i_gain_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write position control i gain complete")
        time.sleep(1)

        # Change the position control D gain of servo ID1 to 145.
        Primary_Servo.servo_set_position_control_d_gain(1, 145, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_position_control_d_gain_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write position control d gain complete")
        time.sleep(1)
        
        # Change the position control PID gain of servo ID1 to 5995, 5, and 145 respectively.
        write_buffer[0] = 5995 & 0xff
        write_buffer[1] = (5995 >> 8) & 0xff
        write_buffer[2] = 5 & 0xff
        write_buffer[3] = (5 >> 8) & 0xff
        write_buffer[4] = 145 & 0xff
        write_buffer[5] = (145 >> 8) & 0xff

        Primary_Servo.servo_write(1, 0x1B, 6, write_buffer, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        print("write position control pid gain status packet:", end=' ')
        for i in range(receive_data_len):
            print(f"0x{receive_data[i]:02x}", end=' ')
        print("\r")
        time.sleep(1)

        # Change the LED condition of servo ID1 to turn on stall error, overheating error, and angle error.
        Primary_Servo.servo_set_led_condition(1, 38, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_led_condition_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write led condition complete")
        time.sleep(1)

        # Change the shutdown condition of servo ID1 to turn on stall error, overheating error, voltage error, and angle error.
        Primary_Servo.servo_set_shutdown_conditions(1, 39, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_shutdown_conditions_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write shutdown conditions complete")
        time.sleep(1)

        # Change the Flash switch of servo ID1 to ON.
        Primary_Servo.servo_set_flash_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_flash_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write flash switch complete")
        time.sleep(1)
        
         # Change the Flash switch of servo ID1 to OFF.
        Primary_Servo.servo_set_flash_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_flash_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write flash switch complete")
        time.sleep(1)

        # Change the LED switch of servo ID1 to ON.
        Primary_Servo.servo_set_led_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_led_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write led switch complete")
        time.sleep(1)
        
        # Change the LED switch of servo ID1 to OFF.
        Primary_Servo.servo_set_led_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_led_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write led switch complete")
        time.sleep(1)
        
        # Change the torque switch of servo ID1 to OFF.
        Primary_Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the control mode of servo ID1 to the PWM control mode.
        Primary_Servo.servo_set_control_mode(1, 3, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_control_mode_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write control mode complete")
        time.sleep(1)

        # Change the torque switch of servo ID1 to ON.
        Primary_Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the target PWM of servo ID1 to -50%.
        Primary_Servo.servo_set_target_pwm(1, -500, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_target_pwm_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write target pwm complete")
        time.sleep(3)
        
        # Change the torque switch of servo ID1 to OFF.
        Primary_Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the control mode of servo ID1 to the current control mode.
        Primary_Servo.servo_set_control_mode(1, 2, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_control_mode_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write control mode complete")
        time.sleep(1)

        # Change the torque switch of servo ID1 to ON.
        Primary_Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the target current of servo ID1 to -400mA.
        Primary_Servo.servo_set_target_current(1, -400, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_target_current_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write target current complete")
        time.sleep(3)
        
        # Change the torque switch of servo ID1 to OFF.
        Primary_Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the control mode of servo ID1 to the velocity base position control mode.
        Primary_Servo.servo_set_control_mode(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_control_mode_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write control mode complete")
        time.sleep(1)

        # Change the torque switch of servo ID1 to ON.
        Primary_Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the velocity base target velocity of servo ID1 to 360°/s.
        Primary_Servo.servo_set_velocity_base_target_velocity(1, 3600, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_velocity_base_target_velocity_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write velocity base target velocity complete")
        time.sleep(1)

        # Change the velocity base target ACC of servo ID1 to 500°/s².
        Primary_Servo.servo_set_velocity_base_target_acc(1, 10, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_velocity_base_target_acc_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write velocity base target acc complete")
        time.sleep(1)

        # Change the velocity base target DEC of servo ID1 to 50°/s².
        Primary_Servo.servo_set_velocity_base_target_dec(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_velocity_base_target_dec_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write elocity base target dec complete")
        time.sleep(1)

        # Change the velocity base target position of servo ID1 to 150°.
        Primary_Servo.servo_set_velocity_base_target_position(1, 1500, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_velocity_base_target_position_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write velocity base target position complete")
        time.sleep(1)
        
        # Change the torque switch of servo ID1 to OFF.
        Primary_Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the control mode of servo ID1 to the time base position control mode.
        Primary_Servo.servo_set_control_mode(1, 0, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_control_mode_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write control mode complete")
        time.sleep(1)

        # Change the torque switch of servo ID1 to ON.
        Primary_Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write torque switch complete")
        time.sleep(1)

        # Change the time base target ACC of servo ID1 to 5.
        Primary_Servo.servo_set_time_base_target_acc(1, 5, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_time_base_target_acc_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write time base target acc complete")
        time.sleep(1)

        # Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively.
        Primary_Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        uart2.flush()
        dir.value(0)
        time.sleep_ms(1)
        receive_data_len = uart2.readinto(receive_data)
        ret = Primary_Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
        if ret == Primary_State.SUCCESS:
            print("write time base target position and moving time complete")
        time.sleep(1)

    # Sync Write Test
    if SYNC_WRITE:
        # Sync write two servos
        servo_sync_parameter.id_counts = 2

        # Set the ID of the first servo to 1
        servo_sync_parameter.id[0] = 1

        # Set the ID of the second servo to 2
        servo_sync_parameter.id[1] = 2
    
        # Change the torque switch of the servo ID1, ID2 to OFF respectively.
        servo_sync_parameter.torque_switch[0] = 0
        servo_sync_parameter.torque_switch[1] = 0
        Primary_Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write torque witch complete")
        time.sleep(1)

        # Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
        servo_sync_parameter.control_mode[0] = 1
        servo_sync_parameter.control_mode[1] = 1
        Primary_Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write control mode complete")
        time.sleep(1)

        # Change the velocity base target ACC of servo ID1, ID2 to 500°/s² and 50°/s², respectively.
        servo_sync_parameter.acc_velocity[0] = 10       
        servo_sync_parameter.acc_velocity[1] = 1
    
        Primary_Servo.servo_sync_write_velocity_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target acc complete")
        time.sleep(1)

        # Change the velocity base target DEC of servo ID1, ID2 to 50°/s² and 500°/s², respectively.
        servo_sync_parameter.dec_velocity[0] = 1       
        servo_sync_parameter.dec_velocity[1] = 10
    
        Primary_Servo.servo_sync_write_velocity_base_target_dec(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target dec complete")
        time.sleep(1)

        # Change the velocity base target velocity of the servo ID1, ID2 to 360°/s² and 720°/s², respectively.
        servo_sync_parameter.velocity[0] = 3600
        servo_sync_parameter.velocity[1] = 1800
    
        Primary_Servo.servo_sync_write_velocity_base_target_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target velocity complete")
        time.sleep(1)

        # Change the velocity base target velocity of the servo ID1, ID2 to 150° midpoint and 0° position, respectively.
        servo_sync_parameter.position[0] = 1500
        servo_sync_parameter.position[1] = 0
    
        Primary_Servo.servo_sync_write_velocity_base_target_position(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target position complete")
        time.sleep(1)

        # Change the velocity base target velocity of servo ID1 ,ID2 to 1800 and 3600, and the position to 3000 and 3000, respectively
        servo_sync_parameter.velocity[0] = 1800
        servo_sync_parameter.velocity[1] = 3600
        servo_sync_parameter.position[0] = 3000
        servo_sync_parameter.position[1] = 3000
    
        Primary_Servo.servo_sync_write_velocity_base_target_position_and_velocity(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target position and velocity complete")
        time.sleep(1)
        
        # Change the velocity base target velocity of servo ID1 ,ID2 to 3600 and 3600, position to 0,0, acceleration to 500°/s², 500°/s², deceleration to 500°/s², 500°/s², respectively
        servo_sync_parameter.velocity[0] = 3600
        servo_sync_parameter.velocity[1] = 3600
        servo_sync_parameter.position[0] = 0
        servo_sync_parameter.position[1] = 0
        servo_sync_parameter.acc_velocity[0] = 10
        servo_sync_parameter.acc_velocity[1] = 10
        servo_sync_parameter.dec_velocity[0] = 10
        servo_sync_parameter.dec_velocity[1] = 10
        
        Primary_Servo.servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write velocity base target acc, dec, velocity and position complete")
        time.sleep(1)

        # Change the torque switch of the servo ID1, ID2 to OFF respectively.
        servo_sync_parameter.torque_switch[0] = 0
        servo_sync_parameter.torque_switch[1] = 0
        Primary_Servo.servo_sync_write_torque_switch(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write torque witch complete")
        time.sleep(1)

        # Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
        servo_sync_parameter.control_mode[0] = 0
        servo_sync_parameter.control_mode[1] = 0
        Primary_Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write control mode complete")
        time.sleep(1)

        # Change the time base target ACC of servo ID1 to 1 and 5 respectively.
        servo_sync_parameter.acc_velocity_grade[0] = 1
        servo_sync_parameter.acc_velocity_grade[1] = 5
    
        Primary_Servo.servo_sync_write_time_base_target_acc(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write time base target acc complete")
        time.sleep(1)

        # Change the time base target position and moving time of servo ID1 to 150° midpoint and 1s, 0° and 500ms respectively.
        servo_sync_parameter.position[0] = 1500
        servo_sync_parameter.position[1] = 0
        servo_sync_parameter.time[0] = 1000
        servo_sync_parameter.time[1] = 500
    
        Primary_Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
        dir.value(1)
        uart2.write(bytes(output_buffer[:output_buffer_len[0]]))
        print("sync write time base target position and moving time complete")
        time.sleep(1)



