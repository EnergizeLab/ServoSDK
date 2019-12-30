using Newtonsoft.Json;
using System.Collections.Generic;
using System.IO;

namespace ServoLib
{
    public class Table
    {
        public int Address { get; set; }
        public string Name { get; set; }
        public string RW { get; set; }
        public int Value { get; set; }
        public int Min { get; set; }
        public int Max { get; set; }
        public int Length { get; set; }
    }
    public class Tables
    {
        public List<Table> ServoTables { get; set; }
    }

    /// <summary>
    /// Series
    /// </summary>
    public enum ServoSeries
    {
        EM3,
    }

    public class Servo
    {
        public Servo(ServoSeries series)
        {
            switch (series)
            {
                case ServoSeries.EM3:
                    string file = File.ReadAllText("JsonFile/EM3.json");
                    ServoTables = JsonConvert.DeserializeObject<Tables>(file);
                    break;
            }
        }
        private List<byte> DataBuf = new List<byte>();
        private int ReceiveLength;
        public Tables ServoTables { get; set; }
        public Table FindTable(byte address)
        {
            foreach (Table item in ServoTables.ServoTables)
            {
                if (item.Address == address)
                    return item;
            }
            return null;
        }

        public byte[] PackData(byte id, byte inst, byte address, byte[] data)
        {
            List<byte> buf = new List<byte>();

            byte length = 0;


            buf.Add(0xff);
            buf.Add(0xff);
            buf.Add(id);

            //length
            buf.Add(0);
            buf.Add(inst);
            switch (inst)
            {
                case 1:
                    length = 2;
                    break;
                case 2:
                    if (FindTable(address) == null)
                    {
                        return null;
                    }
                    length = 1 + 2 + 1;
                    buf.Add(address);
                    buf.Add(data[0]);
                    break;
                case 3:
                    if (FindTable(address) == null)
                    {
                        return null;
                    }
                    length = (byte)(data.Length + 2 + 1);
                    buf.Add(address);

                    for (int i = 0, iL = data.Length; i < iL; i++)
                    {
                        buf.Add(data[i]);
                    }
                    break;
                case 0x15:
                case 0x6:
                    length = 4;
                    buf.Add(0xdf);
                    buf.Add(0xdf);
                    break;

                //校准偏移值
                //ff ff 01 04 63 47 63 ED
                case 0x63:
                    length = 4;
                    buf.Add(0x47);
                    buf.Add(0x63);
                    break;

                //重启
                case 0x64:
                    length = 4;
                    buf.Add(0xDF);
                    buf.Add(0xDF);
                    break;
                default:
                    return null;
            }
            buf[3] = length;
            buf.Add(0);

            buf[buf.Count - 1] = GetCheck(buf);

            return buf.ToArray();
        }
        public bool CheckPack(byte[] data)
        {
            for (int i = 0, iL = data.Length; i < iL; i++)
            {
                DataBuf.Add(data[i]);
                if (DataBuf.Count < 5)
                {
                    switch (DataBuf.Count)
                    {
                        case 1:
                        case 2:
                            if (!(DataBuf[DataBuf.Count - 1] == 0xff))
                                DataBuf.Clear();
                            break;
                        case 4:
                            ReceiveLength = DataBuf[3];
                            break;
                    }
                }
                else if (DataBuf.Count == ReceiveLength + 4)
                {
                    if (DataBuf[DataBuf.Count - 1] == GetCheck(DataBuf))
                    {
                        DataBuf.Clear();
                        if (i + 1 == iL)
                        {
                            return true;
                        }
                    }
                }
                else if (DataBuf.Count > ReceiveLength + 4)
                {
                    DataBuf.Clear();
                }
            }
            return false;
        }
        public byte[] GetBytes(int data, int length)
        {
            List<byte> buf = new List<byte>();
            switch (length)
            {
                case 1:
                case -1:
                    buf.Add((byte)(data & 0xff));
                    break;
                case 2:
                case -2:
                    buf.Add((byte)(data & 0xff));
                    buf.Add((byte)((data & 0xff00) >> 8));
                    break;
                case 4:
                case -4:
                    buf.Add((byte)(data & 0xff));
                    buf.Add((byte)((data & 0xff00) >> 8));
                    buf.Add((byte)((data & 0xff0000) >> 16));
                    buf.Add((byte)((data & 0xff00) >> 24));
                    break;
            }
            return buf.ToArray();
        }
        public byte[] GetBytes(int data1, int data2, int length)
        {
            List<byte> buf = new List<byte>();
            switch (length)
            {
                case 1:
                case -1:
                    buf.Add((byte)(data1 & 0xff));
                    buf.Add((byte)(data2 & 0xff));
                    break;
                case 2:
                case -2:
                    buf.Add((byte)(data1 & 0xff));
                    buf.Add((byte)((data1 & 0xff00) >> 8));
                    buf.Add((byte)(data2 & 0xff));
                    buf.Add((byte)((data2 & 0xff00) >> 8));
                    break;
                case 4:
                case -4:
                    buf.Add((byte)(data1 & 0xff));
                    buf.Add((byte)((data1 & 0xff00) >> 8));
                    buf.Add((byte)((data1 & 0xff0000) >> 16));
                    buf.Add((byte)((data1 & 0xff00) >> 24));
                    buf.Add((byte)(data2 & 0xff));
                    buf.Add((byte)((data2 & 0xff00) >> 8));
                    buf.Add((byte)((data2 & 0xff0000) >> 16));
                    buf.Add((byte)((data2 & 0xff00) >> 24));
                    break;
            }
            return buf.ToArray();
        }
        public byte GetCheck(List<byte> buf)
        {
            byte check = 0;

            int sum = 0;
            for (int i = 2, iL = buf.Count - 1; i < iL; i++)
            {
                sum += buf[i];
            }
            sum = ~sum;
            check = (byte)(sum & 0xff);

            return check;
        }

        public byte[] PackPing(byte id)
        {
            return PackData(id, 0x01, 0, null);
        }
    }
}
