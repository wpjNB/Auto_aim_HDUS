#ifndef DATATYPE_H
#define DATATYPE_H

#include <unistd.h>
#include <iostream>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <fmt/format.h>
#include <fmt/color.h>

using namespace std;
using namespace cv;

// const string camera_name = "KE0200110076";  // 哨兵下云台
const string camera_name = "KE0200110075"; // 步兵（4号)
// const string camera_name = "KE0200110074";  // 步兵（5号）
// const string camera_name = "KE0200110073";  // 英雄
// const string camera_name = "MV_Sentry";     // 哨兵上云台
// const string camera_name = "00J90630561";     // 5号blancer步兵

enum EnemyColor
{
    RED,
    BLUE
};

// 追踪目标装甲板个数
enum class ArmorsNum
{
    NORMAL_4 = 4,
    BALANCE_2 = 2,
    OUTPOST_3 = 3
};
// 子弹速度
enum class BulletSpeed : int
{
    bulle_speed_none = 0,
    hero10 = 10,
    hero16 = 16,
    infantry15 = 15,
    infantry18 = 18,
    infantry30 = 30 // 炮塔拥有相同的子弹速度
};

// 目标状态
enum class TargetState : int
{
    lost_target = 0,
    converging = 1,
    fire = 2,
};
// 追踪目标类型
enum class TargetType : int
{
    none = 0,
    hero = 1,
    engineer = 2,
    infantry3 = 3,
    infantry4 = 4,
    infantry5 = 5,
    outpost = 6,
    guard = 7,
    base = 8,
};

struct TaskData
{
    int mode;
    double bullet_speed;
    Mat img;
    Eigen::Quaterniond quat;
    int timestamp; // 单位：ms
};

struct TargetInfo
{
    TargetInfo() = default;
    TargetType type;              // 目标类型
    float x{}, y{}, z{};          // 目标的坐标
    float vx{}, vy{}, vz{};       // 目标的速度
    float w_yaw{}, yaw{};         // 目标的角速度和期望的偏航角
    float radius_1{}, radius_2{}; // 目标的半径
    float dz{};                   // 目标与玩家之间的距离
};
struct GridAndStride
{
    int grid0;
    int grid1;
    int stride;
};
typedef union
{
    int16_t d;
    unsigned char c[2];
} int16uchar;

// 字节数为4的结构体
typedef union
{
    float f;
    unsigned char c[4];
} float2uchar;
// 用于保存目标相关角度和距离信息及瞄准情况
typedef struct
{
    float2uchar pitch_angle; // 偏航角
    float2uchar yaw_angle;   // 俯仰角
    float2uchar dis;         // 目标距离
    TargetState state;       // 目标状态
    TargetType type;         // 目标类型
    int isFindTarget;        // 当识别的图片范围内有目标且电控发来的信号不为0x00（关闭视觉）置为1，否则置0
    int isFire;              // 是否开火
} VisionSendData;

// 接受来自主控的数据
typedef struct
{
    int mode;         // 当前模式
    EnemyColor Color; // 自己颜色

    BulletSpeed bullet_speed; // 子弹速度
    double gimbal_pitch;      // 云台当前的pitch    rad
    double gimbal_yaw;        // 云台当前的yaw      rad
    double gimbal_roll;       // 云台当前的yaw      rad

} VisionRecvData;

#endif