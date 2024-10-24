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
write_buffer = [0] * 20    # Write data to the memory table

servo_sync_parameter = Primary_Servo_Sync_Parameter()  # Create sync write memory table class.

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
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("sync write torque witch complete")
time.sleep(1)

# Change the control mode of the servo ID1, ID2 to time base position control mode respectively.
servo_sync_parameter.control_mode[0] = 0
servo_sync_parameter.control_mode[1] = 0
Primary_Servo.servo_sync_write_control_mode(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("sync write control mode complete")
time.sleep(1)


# Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
Primary_Servo.servo_set_time_base_target_position_and_moving_time(1, 3000, 500, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
ret = Primary_Servo.servo_set_time_base_target_position_and_moving_time_analysis(receive_data)
if ret == Primary_State.SUCCESS:
    print("write time base target position and moving time complete")
time.sleep(1)

# Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
write_buffer[0] = 0
write_buffer[1] = 3000 & 0xff
write_buffer[2] = (3000 >> 8) & 0xff
write_buffer[3] = 1000 & 0xff
write_buffer[4] = (1000 >> 8) & 0xff

Primary_Servo.servo_write(1, 0x3B, 5, write_buffer, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
receive_data = serial.read(MAX_RECEIVE_LEN)
print("write angle limit status packet:", end=' ')
for i in range(len(receive_data)):
    print(f"0x{receive_data[i]:02x}", end=' ')
print("\r")
time.sleep(1)


# In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms,
# and let servo ID2 move to the 0° position at a constant velocity of 1s.
servo_sync_parameter.position[0] = 1500
servo_sync_parameter.position[1] = 0
servo_sync_parameter.time[0] = 500
servo_sync_parameter.time[1] = 1000

Primary_Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("sync write time base target position and moving time complete")
time.sleep(1)

# In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s,
# and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
servo_sync_parameter.position[0] = 0
servo_sync_parameter.position[1] = 3000
servo_sync_parameter.time[0] = 1000
servo_sync_parameter.time[1] = 500

Primary_Servo.servo_sync_write_time_base_target_position_and_moving_time(servo_sync_parameter, output_buffer, output_buffer_len)
serial.write(bytes(output_buffer[:output_buffer_len[0]]))
print("sync write time base target position and moving time complete")
time.sleep(1)

serial.close()
