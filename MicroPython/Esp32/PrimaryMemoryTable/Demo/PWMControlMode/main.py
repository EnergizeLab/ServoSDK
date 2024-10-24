import micropython
from machine import Pin, UART
import time
from PrimaryServo import *

ret = 0   # Status Flag
output_buffer = bytearray(20)  # 存放生成的指令
output_buffer_len = [0]  # 指令长度
receive_data = bytearray(20)  # 存放接收的应答包
receive_data_len = 0  # 接收数据的长度
analysis_data = [0]  # 应答包解析出来的数据

# Configure serial port 2 (UART2).
uart2 = UART(2, baudrate=1000000, tx=17, rx=16)

dir = Pin(4, Pin.OUT)    # It is used to control the uart transmission direction

while True:
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




