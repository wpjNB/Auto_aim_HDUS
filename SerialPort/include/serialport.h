#ifndef SERIALPORT_H
#define SERIALPORT_H
/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <atomic>
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/netlink.h>
#include <iostream>
#include <vector>
#include "DataType.h"
#include "CRC_Check.h"
#include "general.h"
using namespace std;

#define TRUE 1
#define FALSE 0
// IMU常量定义
#define PI 3.141592653589793
#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_AHRS 0x41
#define AHRS_LEN 48
// 模式
#define CmdID0 0x00; // 关闭视觉
#define CmdID1 0x01; // 自瞄
#define CmdID3 0x03; // 小符
#define CmdID4 0x04; // 大符

// C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)

// 默认串口名
const vector<string> DEFAULT_PORT = {"ttyUSB", "ttyACM"};
// 默认串口最大编号
const int MAX_ITER = 3;

typedef struct
{
    string id;
    string alias;
    string path;
} Device;

class SerialPort
{
public:
    atomic_bool need_init = true;
    Device device;
    int fd;      // 串口号
    int last_fd; // 上一次串口号
    int speed;
    int baud;
    int mode;
    int databits, stopbits, parity;
    unsigned char rdata[255]; // raw_data
    float quat[4];            // 四元数
    float acc[3];             // 加速度
    float gyro[3];            // 角速度
    float bullet_speed;
    SerialPort(const string ID = "/dev/ttyUSB0", const int BUAD = 921600);
    bool open_port(std::string dev_name);
    bool initSerialPort(std::string dev_name);
    bool withoutSerialPort();
    std::vector<Device> listPorts();
    void TransformData(const VisionSendData &data); // 主要方案
    bool ReceiveData(VisionRecvData &visionData);
    void send(const VisionSendData &data);
    void ReadTest();
    void set_Brate();
    int set_Bit();
    void closePort();

private:
    unsigned char Tdata[30]; // transfrom data

    string serial_id;

    float exchange_data(unsigned char *data); // 将4个uchar合并成一个float
    bool getQuat(unsigned char *data);
    bool getGyro(unsigned char *data);
    bool getAcc(unsigned char *data);
    bool getSpeed(unsigned char *data);
    bool get_Mode1();
};

#endif // SERIALPORT_H
