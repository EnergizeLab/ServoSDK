using ServoLib;
using System;
using System.IO.Ports;
using System.Threading;

namespace EM3BaseDemo
{
    class Program
    {
        static void Main(string[] args)
        {
            SerialPort SerialPort = new SerialPort("COM4", 125000);
            Servo Servo = new Servo(ServoSeries.EM3);

            SerialPort.Open();

            byte[] data;
            byte[] receive;

            //Ping
            data = Servo.PackPing(1);
            SerialPort.DiscardInBuffer();
            SerialPort.DiscardOutBuffer();
            SerialPort.Write(data, 0, data.Length);
            Thread.Sleep(10);
            int count = SerialPort.BytesToRead;
            if (count > 0)
            {
                receive = new byte[count];
                SerialPort.Read(receive, 0, count);
                if (Servo.CheckPack(receive))
                {
                    Console.WriteLine("ping success 1");
                }
                else
                {
                    Console.WriteLine("ping failed");
                }

            }
            else
            {
                Console.WriteLine("ping failed");
            }

            //Write  "Plan Profile Position" 1000
            data = Servo.PackData(1, 0x03, 53, Servo.GetBytes(1000, 2));

            //here is a bug, if the firmware version is greater than 4, add a comment
            /////////////////bug start
            SerialPort.DiscardInBuffer();
            SerialPort.DiscardOutBuffer();
            SerialPort.Write(data, 0, data.Length);
            Thread.Sleep(10);
            /////////////////bug end

            SerialPort.DiscardInBuffer();
            SerialPort.DiscardOutBuffer();
            SerialPort.Write(data, 0, data.Length);
            Thread.Sleep(400);

            //Write  "Plan Profile Position" 2000
            data = Servo.PackData(1, 0x03, 53, Servo.GetBytes(2000, 2));
            SerialPort.DiscardInBuffer();
            SerialPort.DiscardOutBuffer();
            SerialPort.Write(data, 0, data.Length);
            Thread.Sleep(400);



            //Read   "Present Profile Position"
            data = Servo.PackData(1, 0x02, 70, Servo.GetBytes(1, 1));
            SerialPort.DiscardInBuffer();
            SerialPort.DiscardOutBuffer();
            SerialPort.Write(data, 0, data.Length);
            Thread.Sleep(10);
            count = SerialPort.BytesToRead;
            if (count > 0)
            {
                receive = new byte[count];
                SerialPort.Read(receive, 0, count);
                if (Servo.CheckPack(receive))
                {
                    string str = null;
                    for(int i=0; i<count; i++)
                    {
                        str += "0x" + i.ToString("X") + "  ";
                    }
                    Console.WriteLine("read  " + str);
                }
                else
                {
                    Console.WriteLine("read failed");
                }

            }
            else
            {
                Console.WriteLine("read failed");
            }

            Console.WriteLine("Press any key to exit...");
            Console.ReadKey();
        }
    }
}
