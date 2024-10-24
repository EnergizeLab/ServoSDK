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

    //Change the control mode of the servo ID1, ID2 to velocity base position control mode respectively.
    servo.control_mode[0] = 1;
    servo.control_mode[1] = 1;
    primary_servo_sync_write_control_mode(servo, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write control mode complete\r\n");
    Sleep(20);

    //Change the velocity base target position of servo ID1 to 150°.
    primary_servo_set_velocity_base_target_position(1, 1500, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        ret = primary_servo_set_velocity_base_target_position_analysis(pack);
        if (ret == PRIMARY_SUCCESS)
            PRINTF("write velocity base target position complete\r\n");
    }
    else
    {
        PRINTF("failed to read data\r\n");
    }
    Sleep(1000);

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s.
    write_buffer[0] = 3000 & 0xff;
    write_buffer[1] = (3000 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;

    primary_servo_write(1, 0x35, 4, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write velocity base target position and velocity status packet: ");
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

    //Change the velocity base target position, velocity base target velocity, velocity base target ACC,
    //and velocity base target DEC of servo ID1 to 0° position, 360°/s, 500°/s², and 50°/s², respectively.
    write_buffer[0] = 0 & 0xff;
    write_buffer[1] = (0 >> 8) & 0xff;
    write_buffer[2] = 3600 & 0xff;
    write_buffer[3] = (3600 >> 8) & 0xff;
    write_buffer[4] = 10 & 0xff;
    write_buffer[5] = 1 & 0xff;

    primary_servo_write(1, 0x35, 6, write_buffer, order_buffer, &order_len);

    serialPort.Write(order_buffer, order_len, &bytesWritten);
    Sleep(1);

    if (serialPort.Read(pack, &bytesRead))
    {
        PRINTF("write velocity base target acc, dec, velocity and position status packet: ");
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

    //In velocity base position control mode, let servo ID1 move to the 150° midpoint and let servo ID2 move to the 0° position.
    servo.position[0] = 1500;
    servo.position[1] = 0;

    primary_servo_sync_write_velocity_base_target_position(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target position complete\r\n");
    Sleep(1000);

    //In velocity base position control mode, let servo ID1 move to the 300° position at a velocity base target velocity of 360°/s,
    //and let servo ID2 move to the 150° position at a velocity base target velocity of 720°/s.
    servo.velocity[0] = 3600;
    servo.velocity[1] = 7200;
    servo.position[0] = 3000;
    servo.position[1] = 1500;

    primary_servo_sync_write_velocity_base_target_position_and_velocity(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target position and velocity complete\r\n");
    Sleep(1000);

    //let servo ID1 move to the 0° position at a velocity base target velocity of 720°/s, a velocity base target ACC of 500°/s², and a velocity base target DEC of 50°/s².
    //Let servo ID2 move to the 300° position at a velocity base target velocity of 360°/s, a velocity base target ACC of 50°/s², and a velocity base target DEC of 500°/s².
    servo.velocity[0] = 7200;
    servo.velocity[1] = 3600;
    servo.position[0] = 0;
    servo.position[1] = 3000;
    servo.acc_velocity[0] = 10;
    servo.acc_velocity[1] = 1;
    servo.dec_velocity[0] = 1;
    servo.dec_velocity[1] = 10;

    primary_servo_sync_write_velocity_base_target_acc_dec_velocity_and_position(servo, order_buffer, &order_len);
    serialPort.Write(order_buffer, order_len, &bytesWritten);
    PRINTF("sync write velocity base target acc, dec, velocity and position complete\r\n");
    Sleep(1000);

    serialPort.Close();

    return 0;
}
