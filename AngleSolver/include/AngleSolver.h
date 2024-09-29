#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "armor.h"
#include "serialport.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp> //放在eigen头文件后面
#include <fmt/format.h>
#include <fmt/core.h>
#include <fmt/color.h>
using namespace std;
// armor size
#define BIG_ARMOR_LEN 217.5
#define BIG_ARMOR_WID 49.5
#define SMALL_ARMOR_LEN 137.2
#define SMALL_ARMOR_WID 54.1
class AngleSolver
{
private:
    TargetInfo t_info;                                 /**< 目标信息 */
    float fx, fy, fz, fx_center, fy_center, fz_center; /**< 附加质量 */
    float pout, yout;                                  /**< 输出变量 */

    std::vector<cv::Point3f> BigArmorPoint3D;
    std::vector<cv::Point3f> SmallArmorPoint3D;
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distortion_coefficients;
    Eigen::Vector3d xyz;
    double x_pos, y_pos, z_pos;
    float cam2GunBiasX;
    float cam2GunBiasY;
    float cam2GunBiasZ;
    float _cam2world_bias_x;
    float _cam2world_bias_y;
    float _cam2world_bias_z;
    Eigen::Vector3d cam2world_bias; // 相机到世界坐标系下平移向量bias
    Eigen::Vector3d cam2gun_bias;   // 相机到枪管坐标系下平移向量bias
    float _coeff_friction;
    float _gravity = 9.8;
    float friction_coeff_ = 0.05; /**< 摩擦系数 */
    float bullet_speed_ = 18.0;
    float bullet_speed_bias_ = -1; /**< 子弹速度偏置 */
    float tmp_fly_time;            /**< 子弹飞行时间 */

    Eigen::Vector3d theta;       // 下位机发来的欧拉角数据单位rad
    Eigen::Matrix3d R_cam2world; // 欧拉角转旋转矩阵

public:
    float pitch, yaw, dis, bias_yaw, bias_pitch, bias_dis;
    float channel_delay_; /**< 通道延迟 */
    AngleSolver(const std::string &paramPath, const std::string &config_file_path);
    void GetTransformation(rm_auto_aim::Armor &Armor, VisionRecvData &recv_data);
    void CompensatePitch();
    void PredictPose(ArmorsNum &armors_num);
    void CalcFinalAngle(TargetInfo &target_msg, VisionRecvData &recv_data, VisionSendData &send_data, ArmorsNum &num);
    static inline double normalize_angle(double angle)
    {
        const double result = fmod(angle + M_PI, 2.0 * M_PI);
        if (result <= 0.0)
            return result + M_PI;
        return result - M_PI;
    }
    static inline double shortest_angular_distance(double from, double to)
    {
        return normalize_angle(to - from);
    }
};

#endif