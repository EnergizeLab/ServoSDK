#include <iostream>
#include "PrimaryServo.h"
#include "CSerialPort.h"

int main()
{
    uint8_t ret;                        //Status Flag
    uint8_t order_buffer[40] = { 0 };     //Store Generated Instructions
    uint8_t order_len = 0;              //Instruction Length
    uint8_t pack[20] = { 0 };             //Store the received status packet
    uint8_t write_buffer[20] = { 0 };   //Write data to the memory table

    struct primary_servo_sync_parameter servo;

    servo.id_counts = 2;                //Sync write two servos
    servo.id[0] = 1;                    //Set the ID of the first servo to 1
    servo.id[1] = 2;                    //Set the ID of the second servo to 2

    CSerialPort serialPort;             //Create a serial port class.

    DWORD bytesRead;                    //The actual number of bytes read from the serial port.
    DWORD bytesWritten;                 //The actual number of bytes written to the serial port.

    if (serialPort.Open(18, 1000000))
    {
        PRINTF("open serial complete\r\n");
    }
    else
    {
        PRINTF("failed to open serial port\r\n");
        return -1;
    }

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

    //Change the time base target position, and moving time of servo ID1 to 300°, and 500ms, respectively.
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

    //Change the time base target ACC, position, and moving time of servo ID1 to 0°, 300°, and 1s, respectively.
    write_buffer[0] = 0;
    write_buffer[1] = 3000 & 0xff;
    write_buffer[2] = (3000 >> 8) & 0xff;
    write_buffer[3] = 1000 & 0xff;
    write_buffer[4] = (1000 >> 8) & 0xff;

    primary_servo_write(1, 0x3B, 5, write_buffer, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write time base target acc, position and moving time status packet: ");
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
    Sleep(1000);

    //In time base position control mode, let servo ID1 move to the 150° position at a velocity of 500ms,
    //and let servo ID2 move to the 0° position at a constant velocity of 1s.
    servo.position[0] = 1500;
    servo.position[1] = 0;
    servo.time[0] = 500;
    servo.time[1] = 1000;

    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write time base target position and moving time complete\r\n");
    Sleep(1000);

    //In time base position control mode, let servo ID1 move to the 0° position at a velocity of 1s,
    //and let servo ID2 move to the 3000° position at a constant velocity of 500ms.
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.time[0] = 1000;
    servo.time[1] = 500;

    primary_servo_sync_write_time_base_target_position_and_moving_time(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write time base target position and moving time complete\r\n");
    Sleep(1000);

    serialPort.Close();

    return 0;
}
