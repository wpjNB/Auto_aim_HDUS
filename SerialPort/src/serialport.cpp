#include "serialport.h"

SerialPort::SerialPort(const string ID, const int BAUD)
{
  serial_id = ID;
  baud = BAUD;
  initSerialPort(ID);
}

/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率

 *@param  databits 类型  int  数据位   取值 为 7 或者8

 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S
 *@param  portchar 类型  char* 串口路径
 */
// dev_name, eg: "/dev/ttyUSB0" or "/dev/ttyS0", etc
bool SerialPort::open_port(std::string dev_name)
{
  // Open the serial port. Change device path as needed
  // Currently set to a standard FTDI USB-UART cable type device
  std::string device_name = dev_name;
  fd = open(device_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fd == -1)
  {
    perror(device_name.c_str());
    return false;
  }
  return true;
}
bool SerialPort::initSerialPort(std::string dev_name)
{

  speed = baud;
  databits = 8;
  stopbits = 1;
  parity = 'N';
  if (open_port(dev_name))
  {
    std::cout << "Opening " << dev_name.c_str() << "..." << std::endl;
  }
  else if (open_port("/dev/ttyACM1"))
  {
    std::cout << "Opening " << "/dev/ttyACM1" << "..." << std::endl;
  }
  else if (open_port("/dev/ttyUSB0"))
  {
    std::cout << "Opening " << "/dev/ttyUSB0" << "..." << std::endl;
  }
  else if (open_port("/dev/ttyUSB1"))
  {
    std::cout << "Opening " << "/dev/ttyUSB1" << "..." << std::endl;
  }
  else
  {
    return false;
  }

  set_Brate();

  if (set_Bit() == FALSE)
  {
    printf("Set Parity Error\n");
    exit(0);
  }
  printf("Open successed\n");

  last_fd = fd;
  need_init = false;
  return true;
}

// TODO: finish visual com port
/**
 *@brief   初始化数据
 *@param  fd       类型  int  打开的串口文件句柄
 *@param  speed    类型  int  波特率

 *@param  databits 类型  int  数据位   取值 为 7 或者8

 *@param  stopbits 类型  int  停止位   取值为 1 或者2
 *@param  parity   类型  int  效验类型 取值为N,E,O,S
 *@param  portchar 类型  char* 串口路径
 */
bool SerialPort::withoutSerialPort() { return true; }

/**
 *@brief   设置波特率
 */
void SerialPort::set_Brate()
{
  // int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
  // 				   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
  // 				  };
  // int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
  // 				  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
  // 				 };
  int speed_arr[] = {B921600, B460800, B230400, B115200, B38400, B19200,
                     B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {921600, 460800, 230400, 115200, 38400, 19200, 9600, 4800, 2400, 1200, 300};
  int i;
  int status;
  struct termios Opt;
  tcgetattr(fd, &Opt);

  for (i = 0; i < sizeof(speed_arr) / sizeof(int); i++)
  {
    if (speed == name_arr[i])
    {
      tcflush(fd, TCIOFLUSH);          // 清空缓冲区的内容
      cfsetispeed(&Opt, speed_arr[i]); // 设置接受和发送的波特率
      cfsetospeed(&Opt, speed_arr[i]);
      status = tcsetattr(fd, TCSANOW, &Opt); // 使设置立即生效

      if (status != 0)
      {
        perror("tcsetattr fd1");
        return;
      }

      tcflush(fd, TCIOFLUSH);
    }
  }
}

/**
 *@brief   设置串口数据位，停止位和效验位
 */
int SerialPort::set_Bit()
{
  struct termios termios_p;

  if (tcgetattr(fd, &termios_p) != 0)
  {
    perror("SetupSerial 1");
    return (FALSE);
  }

  termios_p.c_cflag |= (CLOCAL | CREAD); // 接受数据
  termios_p.c_cflag &= ~CSIZE;           // 设置数据位数

  switch (databits)
  {
  case 7:
    termios_p.c_cflag |= CS7;
    break;

  case 8:
    termios_p.c_cflag |= CS8;
    break;

  default:
    fprintf(stderr, "Unsupported data size\n");
    return (FALSE);
  }

  // 设置奇偶校验位double
  switch (parity)
  {
  case 'n':
  case 'N':
    termios_p.c_cflag &= ~PARENB; /* Clear parity enable */
    termios_p.c_iflag &= ~INPCK;  /* Enable parity checking */
    break;

  case 'o':
  case 'O':
    termios_p.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
    termios_p.c_iflag |= INPCK;             /* Disnable parity checking */
    break;

  case 'e':
  case 'E':
    termios_p.c_cflag |= PARENB;  /* Enable parity */
    termios_p.c_cflag &= ~PARODD; /* 转换为偶效验*/
    termios_p.c_iflag |= INPCK;   /* Disnable parity checking */
    break;

  case 'S':
  case 's': /*as no parity*/
    termios_p.c_cflag &= ~PARENB;
    termios_p.c_cflag &= ~CSTOPB;
    break;

  default:
    fprintf(stderr, "Unsupported parity\n");
    return (FALSE);
  }

  /* 设置停止位*/
  switch (stopbits)
  {
  case 1:
    termios_p.c_cflag &= ~CSTOPB;
    break;

  case 2:
    termios_p.c_cflag |= CSTOPB;
    break;

  default:
    fprintf(stderr, "Unsupported stop bits\n");
    return (FALSE);
  }

  /* Set input parity option */
  if (parity != 'n')
    termios_p.c_iflag |= INPCK;

  tcflush(fd, TCIFLUSH);                                // 清除输入缓存区
  termios_p.c_cc[VTIME] = 150;                          /* 设置超时15 seconds*/
  termios_p.c_cc[VMIN] = 0;                             // 最小接收字符
  termios_p.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); /*Input原始输入*/
  termios_p.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  termios_p.c_iflag &= ~(ICRNL | IGNCR);
  termios_p.c_oflag &= ~OPOST; /*Output禁用输出处理*/

  if (tcsetattr(fd, TCSANOW, &termios_p) != 0) /* Update the options and do it NOW */
  {
    perror("SetupSerial 3");
    return (FALSE);
  }

  return (TRUE);
}

/**
 *@brief   转换数据并发送
 *@param   data   类型  VisionData(union)  包含pitch,yaw,distance
 *@param   flag   类型  char   用于判断是否瞄准目标，0代表没有，1代表已经瞄准
 */
void SerialPort::TransformData(const VisionSendData &data)
{
  Tdata[0] = 0xA5;

  Tdata[1] = CmdID1;
  Append_CRC8_Check_Sum(Tdata, 3);

  Tdata[3] = data.pitch_angle.c[0];
  Tdata[4] = data.pitch_angle.c[1];
  Tdata[5] = data.pitch_angle.c[2];
  Tdata[6] = data.pitch_angle.c[3];

  Tdata[7] = data.yaw_angle.c[0];
  Tdata[8] = data.yaw_angle.c[1];
  Tdata[9] = data.yaw_angle.c[2];
  Tdata[10] = data.yaw_angle.c[3];

  Tdata[11] = data.isFindTarget;
  Tdata[12] = (int)data.state;
  Tdata[13] = 0x00;
  Append_CRC16_Check_Sum(Tdata, 16);
}

/////////////////////////////////////////////
/**
 * @brief 将4个uchar转换为float
 *
 * @param data data首地址指针
 * @return
 */
float SerialPort::exchange_data(unsigned char *data)
{
  float float_data;
  float_data = *((float *)data);
  return float_data;
};

/**
 * @brief 解算速度数据
 *
 * @param data 速度首地址指针
 * @return
 */
bool SerialPort::getSpeed(unsigned char *data)
{
  unsigned char *f1 = &data[0];

  bullet_speed = exchange_data(f1);

  // fmt::print(fmt::fg(fmt::color::white), "speed: {} \n", bullet_speed);
  return true;
}
//////////////////////////////////////////////

// 发送数据函数
void SerialPort::send(const VisionSendData &data)
{
  TransformData(data);
  auto write_stauts = write(fd, Tdata, 16);
}

// 关闭通讯协议接口
void SerialPort::closePort() { close(fd); }
// 数据解析函数 (根据帧类型解析不同的数据)
void parse_data(uint8_t *data, size_t length, uint8_t type)
{

  // 根据需要继续解析其他类型的数据
}
bool SerialPort::ReceiveData(VisionRecvData &visionData)
{
  uint8_t buffer[10];                   // 缓冲区
  int read_bytes = read(fd, buffer, 1); // 读取帧头
  if (read_bytes == 1 && buffer[0] == FRAME_HEAD)
  {
    uint8_t type;
    read(fd, &type, 1); // 读取数据类型
    if (type != TYPE_AHRS)
      return;
    uint8_t data_len;
    read(fd, &data_len, 1); // 读取数据长度
    if (data_len != AHRS_LEN)
      return;
    uint8_t check[4];
    read(fd, check, 4); // 读取校验码
    uint8_t data[256];
    read(fd, data, data_len); // 读取数据内容

    // 数据解析
    if (type == TYPE_AHRS && data_len == AHRS_LEN)
    {
      float ahrs_data[10];
      memcpy(ahrs_data, data, sizeof(ahrs_data));
      visionData.gimbal_roll = ahrs_data[3];
      visionData.gimbal_pitch = ahrs_data[4];
      visionData.gimbal_yaw = ahrs_data[5];
      printf("AHRS Data: Roll=%f, Pitch=%f, Heading=%f\n", ahrs_data[3], ahrs_data[4], ahrs_data[5]);
      return true;
    }
  }
}