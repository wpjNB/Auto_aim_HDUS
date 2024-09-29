#include "AngleSolver.h"

AngleSolver::AngleSolver(const std::string &paramPath, const std::string &config_file_path)
{

    // 大装甲板实际参数
    BigArmorPoint3D = {
        cv::Point3f(-BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(-BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(BIG_ARMOR_LEN / 2.0, -BIG_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(BIG_ARMOR_LEN / 2.0, BIG_ARMOR_WID / 2.0, 0.0)};

    // 小装甲板实际参数
    SmallArmorPoint3D = {
        cv::Point3f(-SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(-SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(SMALL_ARMOR_LEN / 2.0, -SMALL_ARMOR_WID / 2.0, 0.0),
        cv::Point3f(SMALL_ARMOR_LEN / 2.0, SMALL_ARMOR_WID / 2.0, 0.0)};

    cv::FileStorage fsread;
    fsread.open(paramPath, cv::FileStorage::READ);
    if (!fsread.isOpened())
    {
        cout << "Failed to open xml";
        return;
    }

    fsread["camera_matrix"] >> camera_matrix;
    fsread["distortion_coefficients"] >> distortion_coefficients;
    cout << "camera_matrix: " << camera_matrix << endl;
    cout << "distortion_coefficients: " << distortion_coefficients << endl;
    // fmt::print(fmt::fg(fmt::color::yellow), "camera_matrix: {}, distortion_coefficients: {}\n", camera_matrix, distortion_coefficients);
    fsread.release();

    cv::FileStorage config(config_file_path, cv::FileStorage::READ);
    config["compensator"]["cam2GunBiasX"] >> cam2GunBiasX;
    config["compensator"]["cam2GunBiasY"] >> cam2GunBiasY;
    config["compensator"]["cam2GunBiasZ"] >> cam2GunBiasZ;
    config["compensator"]["cam2WorldBiasX"] >> _cam2world_bias_x;
    config["compensator"]["cam2WorldBiasY"] >> _cam2world_bias_y;
    config["compensator"]["cam2WorldBiasZ"] >> _cam2world_bias_z;
    cam2world_bias << _cam2world_bias_x, _cam2world_bias_y, _cam2world_bias_z;
    config["compensator"]["bias_pitch"] >> bias_pitch;
    config["compensator"]["bias_yaw"] >> bias_yaw;
    config["compensator"]["gravity"] >> _gravity;
    config["compensator"]["friction_coeff"] >> friction_coeff_;
    config["compensator"]["bullet_speed_bias"] >> bullet_speed_bias_;
    config["compensator"]["channel_delay"] >> channel_delay_;
    // fmt::print(fmt::fg(fmt::color::yellow_green), "Bias: {}, {}, {}\n", _cam2world_bias_x, _cam2world_bias_y, _cam2world_bias_z);
    // fmt::print(fmt::fg(fmt::color::yellow_green), "Bias pitch: {}, bias yaw: {}\n", bias_pitch, bias_yaw);
    config.release();
}
// 相机坐标系到世界坐标系下的转换
void AngleSolver::GetTransformation(rm_auto_aim::Armor &Armor, VisionRecvData &recv_data)
{
    cv::Mat rVec = cv::Mat::zeros(3, 1, CV_64FC1); // 旋转向量
    cv::Mat rMat;                                  // 旋转矩阵
    cv::Mat tVec = cv::Mat::zeros(3, 1, CV_64FC1); // 平移矩阵
    vector<cv::Point2f> D2 = vector<cv::Point2f>{
        Armor.vertex[0], Armor.vertex[1], Armor.vertex[2], Armor.vertex[3]};

    switch (Armor.armor_type)
    {
    case rm_auto_aim::LARGE:
        solvePnP(
            BigArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec, false,
            cv::SOLVEPNP_IPPE);
        break;
    case rm_auto_aim::SMALL:
        solvePnP(
            SmallArmorPoint3D, D2, this->camera_matrix, this->distortion_coefficients, rVec, tVec, false, cv::SOLVEPNP_IPPE);
        break;
    default:
        break;
    }
    // 相机坐标系下
    {
        x_pos = tVec.at<double>(0, 0);
        y_pos = tVec.at<double>(1, 0);
        z_pos = tVec.at<double>(2, 0);
        // 单位m
        Armor.position_cam << x_pos / 1000 + cam2GunBiasX, y_pos / 1000 + cam2GunBiasY, z_pos / 1000 + cam2GunBiasZ;

        Eigen::AngleAxisd rot_angle(cv::norm(rVec),
                                    Eigen::Vector3d(rVec.at<double>(0, 0),
                                                    rVec.at<double>(1, 0),
                                                    rVec.at<double>(2, 0)));
        // 单位rad
        Armor.rotation_cam = rot_angle.matrix();
        dis = sqrt(Armor.position_cam[0] * Armor.position_cam[0] + Armor.position_cam[1] * Armor.position_cam[1] + Armor.position_cam[2] * Armor.position_cam[2]);
        Armor.dis = dis + bias_dis;
    }
#ifdef isIMU
    // 世界坐标系下
    {
        // 陀螺仪欧拉角(roll给定值)
        theta << recv_data.gimbal_yaw, recv_data.gimbal_pitch, recv_data.gimbal_roll;
        // 下位机陀螺仪imu欧拉角转旋转矩阵
        R_cam2world = eulerToRotationMatrix(theta);

        Armor.position_world = R_cam2world * (Armor.position_cam + cam2world_bias) / 1000; // 单位m
        Armor.rotation_world = R_cam2world * Armor.rotation_cam;                           // 弧度rad
    }
#else
    {
        double tan_pitch = Armor.position_cam[1] / sqrt(Armor.position_cam[0] * Armor.position_cam[0] + Armor.position_cam[2] * Armor.position_cam[2]);
        double tan_yaw = Armor.position_cam[0] / Armor.position_cam[2];
        pitch = -atan(tan_pitch) * 180 / CV_PI + bias_pitch;
        yaw = atan(tan_yaw) * 180 / CV_PI + bias_yaw;
        Armor.pitch = pitch;
        Armor.yaw = yaw;
    }
#endif
}
// 根据目标位置计算补偿角度和新的飞行时间 弹道高度补偿 pitch角度   根据不同距离的范围设置不同的抬头补偿📏：比如2-3米设置一个补偿值，3-4米设置一个补偿值…
void AngleSolver::CompensatePitch()
{
    double dist_horizon = sqrt(fx * fx + fy * fy); // 和目标在水平方向上的距离
    double target_height = fz;                     // 和目标在垂直方向上的距离
    // 迭代参数
    double vx, vy, fly_time, tmp_height = target_height, delta_height = target_height, tmp_pitch, real_height;
#ifdef DEBUG_COMPENSATION
    cout << "init pitch: " << atan2(target_height, dist_horizon) * 180 / M_PI << endl;
#endif // DEBUG_COMPENSATION
    // fmt::print();
    int iteration_num = 0;
    while ((iteration_num <= 50 && abs(delta_height) >= 0.005) || iteration_num <= 5)
    {
        tmp_pitch = atan((tmp_height) / dist_horizon);
        vx = bullet_speed_ * cos(tmp_pitch);
        vy = bullet_speed_ * sin(tmp_pitch);

        fly_time = (exp(friction_coeff_ * dist_horizon) - 1) / (friction_coeff_ * vx);
        real_height = vy * fly_time - 0.5 * _gravity * pow(fly_time, 2);
        delta_height = target_height - real_height;
        tmp_height += delta_height;
        iteration_num++;

#ifdef DEBUG_COMPENSATION
        cout << "iter: " << i + 1 << " " << delta_height << endl;
#endif // DEBUG_COMPENSATION
    }
#ifdef DEBUG_COMPENSATION
    cout << "res:" << tmp_pitch * 180 / CV_PI << endl;
    cout << "fly_time:" << fly_time << endl;
#endif                                               // DEBUG_COMPENSATION
    tmp_fly_time = (float)fly_time + channel_delay_; // 计算新的飞行时间
    pitch = (float)(tmp_pitch * 180 / CV_PI);        // 计算补偿角度
}

// 计算最终的角度
void AngleSolver::CalcFinalAngle(TargetInfo &target_msg, VisionRecvData &recv_data, VisionSendData &send_data, ArmorsNum &num)
{
    tmp_fly_time = channel_delay_;
    // auto bullet_speed = (float)recv_data.speed;        // 获取接收数据的子弹速度
    auto g_yaw = recv_data.gimbal_yaw;                  // 获取接收数据的偏航角
    t_info = target_msg;                                // 设置目标消息
    bullet_speed_ = bullet_speed_ + bullet_speed_bias_; // 计算子弹速度偏移后的速度

    // 迭代
    for (int i = 0; i < 3; i++)
    {
        PredictPose(num);  // 根据飞行时间预测目标位置
        CompensatePitch(); // 根据目标位置计算补偿角度和新的飞行时间
    }

    yaw = (g_yaw + (float)shortest_angular_distance(g_yaw, atan2(fx_center, fy_center))) * 180 / CV_PI;
    send_data.pitch_angle.f = pitch; // 单位角度（世界坐标下）
    send_data.yaw_angle.f = yaw;
}

// 判断两个3D点是否在二维平面上
bool cmp(const cv::Point3f &a, const cv::Point3f &b)
{
    return pow(a.x, 2) + pow(a.y, 2) < pow(b.x, 2) + pow(b.y, 2);
}
// 根据飞行时间预测目标位置
void AngleSolver::PredictPose(ArmorsNum &armors_num)
{
    if (abs(t_info.w_yaw) > 1.5 || (abs(t_info.vx) + abs(t_info.vy) + abs(t_info.vz)) > 2)
    {
        fx = t_info.x + tmp_fly_time * t_info.vx; // 根据飞行时间和速度计算预测的x坐标
        fy = t_info.y + tmp_fly_time * t_info.vy; // 根据飞行时间和速度计算预测的y坐标
        fz = t_info.z + tmp_fly_time * t_info.vz; // 根据飞行时间和速度计算预测的z坐标
        fx_center = fx;
        fy_center = fy;
        float yaw = t_info.yaw + tmp_fly_time * t_info.w_yaw; // 计算预测的偏航角
        cv::Point3f armor1, armor2, armor3, armor4;
        std::vector<cv::Point3f> armor;
        if (armors_num == ArmorsNum::OUTPOST_3)
        {
            armor1.x = fx - t_info.radius_1 * cos(yaw);                        // 计算预测的护甲点1的x坐标
            armor1.y = fy - t_info.radius_1 * sin(yaw);                        // 计算预测的护甲点1的y坐标
            armor1.z = fz;                                                     // 设置预测的护甲点1的z坐标为当前的z坐标
            armor2.x = fx - t_info.radius_1 * cos(float(yaw + 2 * CV_PI / 3)); // 计算预测的护甲点2的x坐标
            armor2.y = fy - t_info.radius_1 * sin(float(yaw + 2 * CV_PI / 3)); // 计算预测的护甲点2的y坐标
            armor2.z = fz;                                                     // 设置预测的护甲点2的z坐标为当前的z坐标
            armor3.x = fx - t_info.radius_1 * sin(float(yaw + 4 * CV_PI / 3)); // 计算预测的护甲点3的x坐标
            armor3.y = fy - t_info.radius_1 * cos(float(yaw + 4 * CV_PI / 3)); // 计算预测的护甲点3的y坐标
            armor3.z = fz;
            armor.push_back(armor1); // 将预测的护甲点1加入vector
            armor.push_back(armor2); // 将预测的护甲点2加入vector
            armor.push_back(armor3); // 将预测的护甲点3加入vector
        }
        else if (armors_num == ArmorsNum::NORMAL_4)
        {
            armor1.x = fx - t_info.radius_1 * cos(yaw); // 计算预测的护甲点1的x坐标
            armor1.y = fy - t_info.radius_1 * sin(yaw); // 计算预测的护甲点1的y坐标
            armor1.z = fz;                              // 设置预测的护甲点1的z坐标为当前的z坐标
            armor2.x = fx + t_info.radius_1 * cos(yaw); // 计算预测的护甲点2的x坐标
            armor2.y = fy + t_info.radius_1 * sin(yaw); // 计算预测的护甲点2的y坐标
            armor2.z = fz;                              // 设置预测的护甲点2的z坐标为当前的z坐标
            armor3.x = fx - t_info.radius_2 * sin(yaw); // 计算预测的护甲点3的x坐标
            armor3.y = fy + t_info.radius_2 * cos(yaw); // 计算预测的护甲点3的y坐标
            armor3.z = fz + t_info.dz;                  // 设置预测的护甲点3的z坐标为当前的z坐标加上护甲的深度
            armor4.x = fx + t_info.radius_2 * sin(yaw); // 计算预测的护甲点4的x坐标
            armor4.y = fy - t_info.radius_2 * cos(yaw); // 计算预测的护甲点4的y坐标
            armor4.z = fz + t_info.dz;                  // 设置预测的护甲点4的z坐标为当前的z坐标加上护甲的深度
            armor.push_back(armor1);                    // 将预测的护甲点1加入vector
            armor.push_back(armor2);                    // 将预测的护甲点2加入vector
            armor.push_back(armor3);                    // 将预测的护甲点3加入vector
            armor.push_back(armor4);                    // 将预测的护甲点4加入vector
        }
        else if (armors_num == ArmorsNum::BALANCE_2)
        {
            armor1.x = fx - t_info.radius_1 * cos(yaw); // 计算预测的护甲点1的x坐标
            armor1.y = fy - t_info.radius_1 * sin(yaw); // 计算预测的护甲点1的y坐标
            armor1.z = fz;                              // 设置预测的护甲点1的z坐标为当前的z坐标
            armor2.x = fx + t_info.radius_1 * cos(yaw); // 计算预测的护甲点2的x坐标
            armor2.y = fy + t_info.radius_1 * sin(yaw); // 计算预测的护甲点2的y坐标
            armor2.z = fz;                              // 设置预测的护甲点2的z坐标为当前的z坐标
            armor.push_back(armor1);                    // 将预测的护甲点1加入vector
            armor.push_back(armor2);                    // 将预测的护甲点2加入vector
        }
        sort(armor.begin(), armor.end(), cmp);       // 根据护甲点的x,y坐标排序
        double rad0 = atan2(armor[0].y, armor[0].x); // 计算排序后的第一个护甲点的航向角
        double rad1 = atan2(armor[1].y, armor[1].x);
        double rad = atan2(fy_center, fx_center); // 计算预测的航向角
        if (abs(shortest_angular_distance(rad0, rad)) < abs(shortest_angular_distance(rad1, rad)))
        {
            fx = armor[0].x; // 设置预测的x坐标为第一个护甲点的x坐标
            fy = armor[0].y; // 设置预测的y坐标为第一个护甲点的y坐标
            fz = armor[0].z; // 设置预测的z坐标为第一个护甲点的z坐标
        }
        else
        {
            fx = armor[1].x; // 设置预测的x坐标为第二个护甲点的x坐标
            fy = armor[1].y; // 设置预测的y坐标为第二个护甲点的y坐标
            fz = armor[1].z; // 设置预测的z坐标为第二个护甲点的z坐标
        }
    }
    else
    {
        fx = t_info.x - t_info.radius_1 * cos(t_info.yaw);
        fy = t_info.y - t_info.radius_1 * sin(t_info.yaw);
        fz = t_info.z;
        fx_center = fx;
        fy_center = fy;
    }
}