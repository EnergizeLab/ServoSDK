# FAQ

## Q1: Stuck Mid-Operation
A: Due to hardware limitations of the 51, which has only one serial port, you need to disable the output in the servo library file servo. h.

## Q2: 51 sent data successfully, but there was no data or the data was wrong
A: Check the hardware connections and ensure that the servo's baud rate is set to level 3, which is 115200. Due to hardware limitations, this 51 cannot reach a baud rate of 1,000,000, so you need to use PZ_ISP to enable double speed at 6T to achieve the maximum baud rate of 115200.
