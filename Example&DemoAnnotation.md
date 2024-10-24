# Example & Demo Annotation


## 1. Preliminary Notes
- In the memory table, some parameters use the value ranges for high bytes and low bytes, with addresses as **the lower byte first and the higher byte next**. Negative values are expressed as the complement of the original code.
- Instruction Packet: If before the change of the data in the Flash area **Flash switch is not turned on, it will not be saved when the power is turned off.**
- Status Packet: Different servos may respond with different information in different states.
    - If there are no errors during the servo instruction packet's processing, the status packet's status value will be 0x00.
    - If a specific error occurs, the corresponding bit is set to 1, and a status packet without parameters is returned.
    - When multiple errors occur simultaneously, the corresponding Bit bits are all set to 1, that is,**values are added together**.
- The following analysis of instruction packets is  **based on actual test results** and is for reference only!


## 2. Example Analysis
### 2.1 Ping
#### Query the model number of servo ID1.
- Instruction Packet: Use ID, Ping instruction for a direct query.
- Status Packet: The parameters queried as 0x4E 0x4F (0x4F4E), indicating that the model number of the servo ID1 is 20302.

---
### 2.2 Read Data
#### Read the firmware version of servo ID1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 1 byte of data starting from the firmware version 0x02 in the memory table.
    - Note: The address is hexadecimal, and bytes read are decimal.
- Status Packet: The parameter read as 0x09, indicating that the firmware version of servo ID1 is 9.
    - Note: Parameters are hexadecimal, descriptions are decimal.


#### Read the model information of servo ID1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 4 bytes of data starting from the model information address 0x03 in the memory table.
- Status Packet: The parameters read as 0x01 0x00 0x00 0x00 (0x00000001), indicating that the factory number of the servo ID1 is 1.


#### Read the baud rate of servo ID1.  
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 1 byte of data starting from the baud rate address 0x08 in the memory table.
- Status Packet: The parameter read as 0x07, indicating that the baud rate of the servo ID1 is 7, which corresponds to a serial communication speed of 1 Mbps according to the table.
    - Note: The default baud rate is 1 Mbps, and the corresponding value in the memory table is 7.


#### Read the return delay time of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0xFA(250), indicating that the return delay time of servo ID1 is 500us.
    - Note: Parameters with units need to be converted into specific decimal values.


#### Read the return level of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x02, indicating that the return level of servo ID1 is 2. It can be determined from the table that it responds to all instructions.
    - Note: The return level values 0, 1, and 2 represent respectively: responding only to the Ping instruction, responding to the Ping instruction and read data instruction, and responding to all instructions.


#### Read the angle limit of servo ID1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 4 bytes of data starting from the min angle limit 0x08 in the memory table.
- Status Packet: The parameter read as 0x00 0x00 0xB8 0x0B, that is, the min angle limit is 0x0000(0), the max angle limit is 0x0BB8(3000), indicating that the angle limit of servo ID1 is 0~3000°.


#### Read the voltage limit of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x5A 0x21, that is, the max voltage limit is 0x5A(90), the min voltage limit is 0x21(33), indicating that the voltage limit of servo ID1 is 3.3V~9V


#### Read the max voltage limit of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: Taking EM-2030 as an example, the parameter read as 0x90 0x01, that is, 0x0190(400), indicating that the max voltage limit of servo ID1 is 400mA.
    - Note: Different servo models have varying current ranges and default values, which need to be specified with examples for each model.


#### Read the position control PID gain of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x70 0x17 0x00 0x00 0x..., indicating that the position control P gain, position control I gain, and position control D gain of servo ID1 is 0x1770(6000), ...(0), ...(150), respectively.


#### Read the LED condition of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x24, indicating that the LED condition of servo ID1 enables the stall error and overheating error.
    - Note: The values for LED conditions 0x01, 0x02, 0x04, ..., represent enabling voltage errors, angle errors, ... , respectively. The parameter values can be superimposed, indicating multiple types of errors are enabled.


#### Read the shutdown condition of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x24, indicating that the shutdown condition of servo ID1 enables the stall error and overheating error.
    - Note: The values for shutdown condition 0x01、0x02、0x04..., represent enabling voltage errors, angle errors, ... , respectively. The parameter values can be superimposed, indicating multiple types of errors are enabled.


#### Read the control mode of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x01, indicating that the control mode of servo ID1 is 1. It can be determined from the table that it's velocity base position control mode.
    - Note: The control mode values 0, 1, 2, 3 respectively represent...


#### Read the Flash switch status of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x00, indicating that the Flash switch status of servo ID1 is OFF.
    - Note: The Flash switch values 0 and 1 represent OFF and ON respectively.


#### Read the LED switch status of servo ID1. 
*Take the primary memory table as an example.*
- Note: The LED switch status values 0 and 1 represent OFF and ON respectively.


#### Read the torque switch status of servo ID1. 
*Take the primary memory table as an example.*
- Note: The torque switch status values 0, 1, and 2 represent OFF, ON, and brake mode respectively.


#### Read the velocity base target position, velocity, ACC, and DEC of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: Taking the EM-2030 as an example, the parameter read as 0xDC 0x05 0x20 0x1C 0x96 0x96, that is, velocity base target position is 0x05DC(1500), velocity base target velocity is 0x1C20(7200), velocity base target ACC is 0x96(150), velocity base target DEC is 0x96(150), indicating that the velocity base target position of servo ID1 is 150°, velocity base target velocity is 720°/s, velocity base target ACC is 7500°/s², and velocity base target DEC is 7500°/s².


#### Read the time base target ACC of servo ID1. 
*Take the primary memory table as an example.*
- Status Packet: The parameter read as 0x00, indicating that the time base target ACC of servo ID1 is 1. It can be determined from the table that there is no acceleration.
    - Note: The values for the time base target ACC 0, 1, 2, 3, 4, 5 represent that a higher acceleration level results in faster acceleration and deceleration.


#### Read the present position of servo ID1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 2 bytes of data starting from the present position 0x4A in the memory table.
- Status Packet: The parameter read as 0xDC 0x05, that is, 0x05DC(1500), indicating that the present position of servo ID1 is 150°.


#### Read the present position and present current of servo ID1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, read 4 bytes of data starting from the present position 0x4A in the memory table.
- Status Packet: The parameter read as 0xDC 0x05 0x90 0x01, that is, present position 0x05DC(1500),  and present current 0x0190(400), indicating that the present position of servo ID1 is 150° and present current is 400mA.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 2.3 Write Data
#### Change the servo ID of servo ID1 to 2. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, write data 0x02 starting from the servo ID 0x07 in the memory table.
- Status Packet: Since the servo ID has been changed to 2, no status packet will be returned. Sending Ping instruction to the servo ID2 can be used to verify if the change was successful.


#### Change the servo ID of the servo with an unknown ID to 1. 
*Take the primary memory table as an example.*
- Instruction Packet: Use roadcast ID, write data 0x01 starting from the servo ID 0x07 in the memory table.
- Status Packet: Since broadcast ID (0xFE) is used to send instructions, no status packet will be returned.  Sending Ping instruction to the servo ID1 can be used to verify if the change was successful.


#### Change the baud rate of servo ID1 to 9600. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, write data 0x01 starting from the return delay time 0x08 in the memory table (9600 corresponds to number 1).
- Status Packet: Since the baud rate has been changed to 1, no status packet will be returned. The baud rate to 9600 can be reselected to complete the device connection, then send Ping instruction to the servo ID1 can be used to verify if the change was successful.


#### Change the return delay time of servo ID1 to 500us. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, write data 0xFA(250) starting from the return delay time 0x09 in the memory table.
- Status Packet: Reading the response delay time of the servo can be used to verify if the change was successful.


#### Change the return level of servo ID1 to only respond to the Ping instruction. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, write data 0x00 starting from the return level 0x0A in the memory table.
- Status Packet: Since the return level has been changed to only respond to the Ping instruction, no status packet will be returned. Sending the read data instruction can be used to verify if the change was successful. If there is no response to the read data instruction, it indicates that the change was successful.


#### Change the angle limit of servo ID1 to 100°~150°. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1, sequentially write data 0xE8 0x03 0xDC 0x05 (1000, 1500) starting from the min angle limit 0x0B in the memory table.
- Status Packet: Reading the angle limit of the servo can be used to verify if the change was successful.
    - Note: The servo position limits have been changed, and the target position (including the velocity base target position and time base target position) will be forcibly set within the range of position limits. If it exceeds the set max angle limit or lowers the set min angle limit, the angle alarm bit (Bit: 0x02) in the servo's present status will be set to 1. If the LED condition has the angle alarm enabled, the servo will cycle the LED switch between 1 and 0, causing the LED light to flash; if the shutdown condition is enabled for angle protection, the servo will set the torque switch to 0, thus disabling the torque. 


#### Change the max temperature limit of servo ID1 to 65℃. 
*Take the primary memory table as an example.*
- Note: The max temperature limit has been changed. When the present internal temperature of the servo exceed the upper limit, the temperature alarm bit (Bit: 0x04) in the servo's present status will be set to 1. If the LED condition has overheat alarm enabled (default is enabled), the servo will cycle the LED switch between 1 and 0, causing the LED light to flash. If the shutdown condition has overheat protection enabled (default is enabled), the servo will set the torque switch to 0. That is, disabling the torque.


#### Change the voltage limit of servo ID1 to 3.5~8.4V. 
*Take the primary memory table as an example.*
- Note: The voltage limit has been changed. When the present input voltage of the servo exceeds the set max voltage limit or lower than the set min voltage limit, the voltage alarm bit (Bit: 0x01) in the servo's present status will be set to 1. If the LED condition has voltage alarm enabled, the servo will cycle the LED switch between 1 and 0, causing the LED light to flash. If the shutdown condition has voltage protection enabled, the servo will set the torque switch to 0. That is, disabling the torque.


#### Change the max PWM limit of servo ID1 to 90%. 
*Take the primary memory table as an example.*
- Note: The high level time accounts for 90% of the entire cycle, but the servo outputs a PWM of less than 100%. Reducing the PWM output of the servo will decrease its torque and velocity.


#### Change the max current limit of servo ID1 to 900mA. 
*Take the primary memory table as an example.*
- Note: The max current limit has been changed. When the servo's present current exceeds the set current limit and the duration is longer than the specified current protection time, the stall alarm bit (Bit: 0x20) for the servo's present status will be set to 1. If the LED condition has stall alarm enabled (default is enabled), the servo will cycle the LED switch between 1 and 0, causing the LED to flash. If the shutdown condition has stall protection enabled (default is enabled), the servo will set the torque switch to 0, thus disabling the torque.


#### Change the current shutdown time of servo ID1 to 500ms. 
*Take the primary memory table as an example.*
- Note: The current shutdown time has been changed. When the servo's current exceeds the set current limit and the duration is longer than the specified current protection time, the stall alarm bit (Bit: 0x20) for the servo's present status will be set to 1. If the LED condition has stall alarm enabled, the servo will cycle the LED switch between 1 and 0, causing the LED to flash. If the shutdown condition has stall protection enabled, the servo will set the torque switch to 0, thus disabling the torque.


#### Change the CW deadband of servo ID1 to 0.2°. 
*Take the primary memory table as an example.*
- Note: When the servo is within 0.2° clockwise of the target position, the motor will not output any torque.


#### Change the PWM punch of servo ID1 to 20%. 
*Take the primary memory table as an example.*
- Note: The PWM Punch is factory-set based on the motor characteristics and is generally used to compensate for the servo's control deadband of the servo or overcome static friction.
    - Note: It can be observed that increasing the single PWM overlay value causes the motor's velocity to decrease, accompanied by jitter.


#### Change the position control P gain of servo ID1 to 5500. 
*Take the primary memory table as an example.*
- Note: The PID parameters can influence each other, and excessive adjustments may lead to new problems, so adjustments should be made with caution.
- Status Packet: It can be observed that increasing the single P value makes the control system more sensitive to errors and faster in response, but if set too high, it may cause overshoot and oscillation.


#### Change the position control D gain of servo ID1 to 100. 
*Take the primary memory table as an example.*
- Note: The PID parameters can influence each other, and excessive adjustments may lead to new problems, so adjustments should be made with caution.
- Status Packet: It can be observed that increasing the single I value reduces the system's steady-state error, making the system more precise; however, if it is too large, it may lead to system instability.


#### Change the position control D gain of servo ID1 to 250. 
*Take the primary memory table as an example.*
- Note: The PID parameters can influence each other, and excessive adjustments may lead to new problems, so adjustments should be made with caution.
- Status Packet: It can be observed that increasing the single D value suppresses excessive response speed and oscillation, making the system smoother; however, if it is too large, it may lead to jitter and instability.


#### Change the control mode of servo ID1 to the time base position control mode. 
*Take the primary memory table as an example.*
- Note: Before switching the servo ID1 control mode, please power on the servo ID1 again or turn off the torque switch. In the same control mode, you only need to set the control mode once to set different movements for the servo.

#### Change the Flash switch of servo ID1 to ON. 
*Take the primary memory table as an example.*
- Instruction Packet: Use ID1 to write data 0x01 starting from the Flash switch address 0x2E in the memory table.
- Status Packet: Reading the Flash switch status of the servo can be used to verify if the change was successful.


#### Change the LED switch of servo ID1 to ON. 
*Take the primary memory table as an example.*
- Note: When operating the torque switch, the current status flag bits of the servo will be cleared. For example, when an overcurrent condition triggers stall protection, the servo torque will be disabled, and writing a value to the plan position will not elicit a response from the servo.
- Warning: If the servo needs to be forcibly working again by operating the torque switch to clear the stall protection flag bits, there is a certain risk. Please ensure that appropriate protections are in place (such as reducing the load) before performing this operation.


#### Change the target PWM of servo ID1 to -50%. 
*Take the primary memory table as an example.*
- Note: When using PWM control mode to move the servo, it is essential to ensure that the control mode is set to 3 and that the torque switch is turned on.
- Instruction Packet: Use ID1 to write data sequentially starting from the target PWM address 0x31 in the memory table... (Negative values are expressed as the complement of the original code.)
- Status Packet: Reading the target PWM of the servo or observing whether the rotates counterclockwise with a 50% target PWM can be used to verify if the change was successful.


#### Change the target current of servo ID1 to -100mA. 
*Take the primary memory table as an example.*
- Note: When using the current control mode to move the servo, it is essential to ensure that the control mode is set to 2 and that the torque switch is turned on.
- Status Packet: Reading the target current of the servo or observing if the servo rotates counterclockwise with a target current of 100mA can be used to verify if the change was successful.
- Additional Note: In position control mode (including time base position control and velocity base position control), the target current is used to limit the servo's maximum output current. A smaller current results in less locking force for the servo, and at this point, the target current has no positive or negative values. Setting this value too high may cause slipping, while setting it too low may lead to overshooting.


#### Change the velocity base target position of servo ID1 to 150°. 
*Take the primary memory table as an example.*
- Note: When using velocity base position control mode to move the servo, it is essential to ensure that the control mode is set to 1. When writing values to the target position address, the torque switch will automatically be set to 1, enabling the torque.
- Instruction Packet: Use ID1 to write data 0xDC, 0x05 sequentially starting from the velocity base target position address 0x35 in the memory table.
- Status Packet: Reading the velocity base target position of the servo or observing if the servo moves to the midpoint can be used to verify if the change was successful.
    - Note: When the present position is the same as the target position, the servo will not move.


#### Change the time base target position and moving time of servo ID1 to 300°, 500ms respectively. 
*Take the primary memory table as an example.*
- Note: When using time base position control mode to move the servo, it is essential to ensure that the control mode is set to 0. When writing values to the target position address, the torque switch will automatically be set to 1, enabling the torque.
- Instruction Packet: Use ID1 to write data 0x... sequentially starting from the time base target position address 0x3C ... in the memory table.
- Status Packet: Reading the time base target position and moving time of the servo or observing if the servo moves to the 300° position in 500ms can be used to verify if the change was successful.
    - Note: When the present position is the same as the target position, the servo will not move.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 2.4 Sync Write
#### Change the velocity base target position of the servo ID1, ID2 to 150° midpoint and 0° position, respectively. 
*Take the primary memory table as an example.*
- Note: When using velocity base position control mode to move the servo, it is essential to ensure that the control mode is set to 1. When writing values to the target position address, the torque switch will automatically be set to 1, enabling the torque.
- Instruction Packet: Use the broadcast ID to synchronously write 2 bytes starting from the velocity base target position address 0x35 in the memory table. For ID1, write data 0xDC, 0x05; for ID2, write data 0x00, 0x00.
- Status Packet: Since the instruction is sent using the broadcast ID (0xFE), no status packet will be returned. Reading the velocity base target position of each servo or observing whether each servo moves to the corresponding position can be used to verify if the change was successful.
    - Note: When the present position is the same as the target position, the servo will not move.


#### Change the time base target position and moving time of servo ID1, ID2 to 150° midpoint, 1s, and 500ms respectively. 
*Take the primary memory table as an example.*
- Note: When using time base position control mode to move the servo, it is essential to ensure that the control mode is set to 0. When writing values to the target position address, the torque switch will automatically be set to 1, enabling the torque.
- Instruction Packet: Use the broadcast ID to synchronously write 4 bytes starting from the time base target position address 0x3C in the memory table. For ID1, write data 0xDC, 0x05, 0xE8, 0x03 in sequence; for ID2, write data 0x00, 0x00, 0xF4, 0x01 in sequence.
- Status Packet: Since the instruction is sent using the broadcast ID (0xFE), no status packet will be returned. Reading the time base target position of each servo or observing whether each servo moves to the corresponding position within the set time can be used to verify if the change was successful.
    - Note: When the present position is the same as the target position, the servo will not move.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 2.5 Factory Reset
#### Reset the servo to the factory default values.
- Note: This operation will reset all parameters, including baud rate, servo ID, offset value, and others.
- Instruction Packet: Use ID1 to send the factory reset instruction, with parameters 0xDF 0xDF to unlock this function.
- Status Packet: Servo will reboot after this operation is performed. Ping instruction or Read Data instruction can be used to verify whether all data has been restored.


---
### 2.6 Parameter Reset
#### Reset the servo to the factory default values.
- Note: This operation will reset all parameters, including baud rate, servo ID, offset value, and others.
- Instruction Packet: Use ID1 to send the factory reset instruction, with parameters 0xDF 0xDF to unlock this function.
- Status Packet: Servo will reboot after this operation is performed. Ping instruction or Read Data instruction can be used to verify whether all data has been restored.


---
### 2.7 Calibration
#### Calibrate the midpoint of the servo.
- Warning: Generally, calibration is done at the factory condition. If this function is not operated according to the specified steps or is performed multiple times consecutively, it may result in excessive deviation of the servo's target position, causing it to enter the dead band and operate abnormally. Please use it with caution. If this situation occurs, it can be fixed by performing a Factory Reset Instruction.
- Note: There is a scale line at the 150° position on the horn. When aligned with the scale line on the outer shell, it is the 150° middle position.
- Instruction Packet: Use ID1 to send the calibration instruction, with parameters 0xDF 0xDF to unlock this function.
- Status Packet: Reading if the servo's present position is close to 150° can be used to verify if the change was successful.


---
### 2.8 Reboot
### Reboot the servo.
- Instruction Packet: Use ID1 to send the reboot instruction, with parameters 0xDF 0xDF to unlock this function.
- Status Packet: Since the servo will immediately reboot upon receiving the instruction, no status packet will be returned. Observing if the servo's LED light flashes once can be used to verify if the reboot was successful.


## 3. Demo Analysis
### 3.1 Velocity Base Position Control Mode
#### Let servo ID1 move to 150° midpoint. 
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "1" to set the control mode, switching to the velocity base position control mode. (Only a one-time setting is required for the same control mode.)

**Implementation Steps:**
1. Change the torque switch status of servo ID1 to OFF.
    - Instruction Packet: Use ID1 to write data 0x00 starting from the torque switch address 0x30 in the memory table.
    - Status Packet: Reading the servo's torque switch status can be used to verify if the change was successful.
2. Change the control mode of servo ID1 to velocity base position control mode.
    - Instruction Packet: Use ID1 to write data 0x01 starting from the control mode address 0x23 in the memory table.
    - Status Packet: Reading the servo's control mode can be used to verify if the change was successful.
3. Change the velocity base target position of servo ID1 to 150°.
    - Instruction Packet: Use ID1 to write data 0xDC, 0x05 sequentially starting from the velocity base target position address 0x35 in the memory table.
        - Note: When writing values to the target position address, the torque switch will automatically be set to 1, enabling the torque.
    - Status Packet: Reading the servo's velocity base target position or observing if the servo moves to the midpoint can be used to verify if the change was successful.
        - Note: When the present position is the same as the target position, the servo will not move.


#### Let servo ID1 move to the 0° position at a velocity base target velocity of 360°/s, a velocity base target ACC of 500°/s², and a velocity base target DEC of 50°/s².
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "1" to set the control mode, switching to the velocity base position control mode. (Only a one-time setting is required for the same control mode.)
3. Set the velocity base parameters, allowing servo ID1 to move as specified.

**Implementation Steps:**
1. Change the torque switch of servo ID1 to OFF.
2. Change the control mode of servo ID1 to the velocity base position control mode.
3. Change the velocity base target position, velocity base target velocity, velocity base target ACC, and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 3.2 Time Base Position Control Mode
#### Let servo ID1 move to the 300° position at a velocity of 500ms.  
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "0" to set the control mode, switching to the time base position control mode. (Only a one-time setting is required for the same control mode.)
3. Set the time base target position and moving time, allowing servo ID1 to move as specified.

**Implementation Steps:**
1. Change the torque switch of servo ID1 to OFF.
2. Change the control mode of servo ID1 to time base position control mode.
3. Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.


#### Let servo ID1 move to the 0° position at a constant velocity of 1s. 
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "1" to set the control mode, switching to the velocity base position control mode. (Only a one-time setting is required for the same control mode.)
3. Set the time base parameters, allowing servo ID1 to move as specified.

**Implementation Steps:**
1. Change the torque switch of servo ID1 to OFF.
2. Change the control mode of servo ID1 to velocity base position control mode.
3. Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
        - Note: The default value for the time case target ACC is 0, which means no acceleration (constant speed). Levels 1 to 5 have higher values for acceleration, resulting in faster increases and decreases in speed.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 3.3 PWM Control Mode
#### Let the servo ID1 rotate counterclockwise with a 50% PWM output.  
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "3" to set the control mode, switching to the PWM control mode.
3. Write "1" to the torque switch, enabling the torque.
4. Set the target PWM, allowing servo ID1 to move as specified.

**Implementation Steps:**
1. Change the torque switch of servo ID1 to OFF.
    - 指Instruction Packet: Use ID1 to write data 0x00 starting from the torque switch address 0x30 in the memory table.
    - Status Packet: Reading the servo's torque switch status can be used to verify if the change was successful.
2. Change the control mode of servo ID1 to the PWM control mode.
    - Instruction Packet: Use ID1 to write data 0x03 starting from the control mode address 0x23 in the memory table.
    - Status Packet: Reading the servo's control mode can be used to verify if the change was successful.
3. Change the torque switch of servo ID1 to ON.
    - Instruction Packet: Use ID1 to write data 0x01 starting from the torque switch address 0x30 in the memory table.
    - Status Packet: Reading the servo's torque switch status can be used to verify if the change was successful.
4. Change the target PWM of servo ID1 to -50%.
    - Instruction Packet: Use ID1 to write data... sequentially starting from the target PWM address 0x31 in the memory table.
        - Note: Negative values are expressed as the complement of the original code.
    - Status Packet: Reading the servo's target PWM or observing if the servo rotates counterclockwise with a 50% target PWM output can be used to verify if the change was successful.


#### Intermediate/advanced memory tables: under continuous development, explained in the future


---
### 3.4 Current Control Mode
#### Let the servo ID1 rotate clockwise with a target current of 100mA. 
*Take the primary memory table as an example.*

**Operation Steps:**
1. Write "0" to the torque switch, turning off the torque.
2. Write "2" to set the control mode, switching to the current control mode.
3. Write "1" to the torque switch, enabling the torque.
4. Set the target current, allowing servo ID1 to move as specified.

**Implementation Steps:**
1. Change the torque switch of servo ID1 to OFF.
2. Change the control mode of servo ID1 to the current control mode.
3. Change the target PWM of servo ID1 to 100mA.
4. Change the torque switch of servo ID1 to ON.


#### Intermediate/advanced memory tables: under continuous development, explained in the future
