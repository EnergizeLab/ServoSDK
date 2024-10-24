# FAQ

## Q1: How to control multiple servos simultaneously?
A: By using the Sync Write Instruction, you can write data to the memory tables of multiple servos at once. The length of the data and the starting address for the data written on multiple servos must be the same, but the data itself can differ.

## Q2: Under the time base position control mode, if a new instruction is sent while the servo is in motion, will the new instruction be executed?
A: This depends on the situation:
- If the new instruction does not affect its motion,e.g., reading position or temperature, the servo will continue its current motion.
- If the new instruction affects its motion,e.g., a motion instruction.:
    - If the new position is the same as the previous motion position, the instruction will not be executed.
    - If the new position is different from the previous motion position, the current instruction will be interrupted immediately to execute the new instruction.
