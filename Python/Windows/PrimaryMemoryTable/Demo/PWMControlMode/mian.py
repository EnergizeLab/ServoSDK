from PrimaryServo import *
import serial
import time

MAX_RECEIVE_LEN = 30  # Read the maximum data length from the serial port.

serial = serial.Serial('COM18', 1000000, timeout=0.01)  # Open the specified serial port and set the timeout.
if serial.isOpen():
    print("open successful")
else:
    print("open failed")

ret = 0   # Status Flag
output_buffer = [0] * 40    # Store Generated Instructions
output_buffer_len = [0]    # Instruction Length
receive_data = [0] * 40    # Store the received status packet
analysis_data = [0]    # Data parsed from the status packet

# Change the torque switch of servo ID1 to OFF.
Primary_Servo.servo_set_torque_switch(1, 0, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("write torque switch complete")
time.sleep(1)

# Change the control mode of servo ID1 to the PWM control mode.
Primary_Servo.servo_set_control_mode(1, 3, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_control_mode_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("write control mode complete")
time.sleep(1)

# Change the torque switch of servo ID1 to ON.
Primary_Servo.servo_set_torque_switch(1, 1, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_torque_switch_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("write torque switch complete")
time.sleep(1)

# Change the target PWM of servo ID1 to -50%.
Primary_Servo.servo_set_target_pwm(1, -500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_target_pwm_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("write target pwm complete")
time.sleep(3)

serial.close()
