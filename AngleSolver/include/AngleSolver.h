#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include "armor.h"
#include "detector.h"
using namespace std;
// armor size
#define BIG_ARMOR_LEN 217.5
#define BIG_ARMOR_WID 49.5
#define SMALL_ARMOR_LEN 137.2
#define SMALL_ARMOR_WID 54.1
class AngleSolver
{
private:
    float pitch, yaw, dis;
    std::vector<cv::Point3f> BigArmorPoint3D;
    std::vector<cv::Point3f> SmallArmorPoint3D;
    cv::Mat camera_matrix = cv::Mat(3, 3, CV_32FC1);
    cv::Mat distortion_coefficients;
    float _cam_bias_z;
    float _cam_bias_y;
    float _coeff_friction;
    float _gravity;

public:
    AngleSolver();
    ~AngleSolver();

    void Init(const std::string &paramPath, float camBiasZ, float camBiasY, float gravity);
    void solve_angle(rm_auto_aim::Armor &TargetArmor);
    void GetAngle(float &pitch, float &yaw, float &distance);
};

#endif