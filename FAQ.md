# FAQ

## Q1: How to control multiple servos simultaneously?
A: By using the Sync Write Instruction, you can write data to the memory tables of multiple servos at once. The length of the data and the starting address for the data written on multiple servos must be the same, but the data itself can differ.

## Q2: Under the time base position control mode, if a new instruction is sent while the servo is in motion, will the new instruction be executed?
A: This depends on the situation:
- If the new instruction does not affect its motion,e.g., reading position or temperature, the servo will continue its current motion.
- If the new instruction affects its motion,e.g., a motion instruction.:
    - If the new position is the same as the previous motion position, the instruction will not be executed.
    - If the new position is different from the previous motion position, the current instruction will be interrupted immediately to execute the new instruction.

## Q3: After running the example code, the error “This is not a complete response package!” pops up.
A: This depends on the situation:
- If there is no response package data, it might be because the servo ID has not been changed to 1 or the servo is not connected. You can verify this using ServoStudio.
    - <img src="images/Q3 no response packet data.jpg?raw=true" alt="no response package data" width="100%">
- If there is incorrect response package data, it might be due to multiple servos being connected with the same ID of 1. You can also verify this using ServoStudio.
    - <img src="images/Q3 incorrect response package data.jpg?raw=true" alt="incorrect response package data" width="100%">
- If the response package data is intermittent, it may indicate an unstable hardware connection. Try replacing the servo cables or other hardware connections.
    - <img src="images/Q3 the response package data is intermittent.jpg?raw=true" alt="Servo and PC Connection Diagram" width="100%">
