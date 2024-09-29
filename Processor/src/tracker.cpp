// 版权 2022 陈军

#include "tracker.h"

// 标准库
#include <cfloat>
#include <opencv2/calib3d.hpp>
#include <string>

#define CLAMP(x, a, b) (MIN(MAX(x, a), b))

// Tracker类的构造函数，从配置文件中读取参数
Tracker::Tracker(const std::string &config_file_path)
{
    // 初始化成员变量为默认值
    tracker_state = LOST;
    tracked_id = std::string("");
    target_state = Eigen::VectorXd::Zero(9);
    cv::FileStorage config(config_file_path, cv::FileStorage::READ);
    // 从配置文件读取参数
    config["processor"]["max_match_distance"] >> max_match_distance_;
    config["processor"]["max_match_yaw_diff"] >> max_match_yaw_diff_;
    config["processor"]["tracking_threshold"] >> tracking_threshold_;
    config["processor"]["lost_threshold"] >> lost_threshold_;
}

// Tracker类的构造函数，使用显式参数
Tracker::Tracker(double max_match_distance, int tracking_threshold, int lost_threshold)
    : tracker_state(LOST),
      tracked_id(std::string("")),
      target_state(Eigen::VectorXd::Zero(9)),
      max_match_distance_(max_match_distance),
      tracking_threshold_(tracking_threshold),
      lost_threshold_(lost_threshold)
{
}

// 根据Armor对象的向量初始化跟踪器
void Tracker::init(const std::vector<rm_auto_aim::Armor> &armors_msg)
{
    // 装甲板不存在
    if (armors_msg.empty())
    {
        return;
    }

    // 选择离图像中心最近的装甲板
    auto min_distance = DBL_MAX;
    tracked_armor = armors_msg[0];
    for (const auto &armor : armors_msg)
    {
        if (armor.distance_to_image_center < min_distance)
        {
            min_distance = armor.distance_to_image_center;
            tracked_armor = armor;
        }
    }

    // 使用选定的装甲板初始化扩展卡尔曼滤波器（EKF）
    initEKF(tracked_armor);
    // 记录追踪的装甲板id数字与状态
    tracked_id = tracked_armor.number;
    tracker_state = DETECTING;

    // 根据跟踪的装甲板更新装甲板个数
    updateArmorsNum(tracked_armor);
}

// 根据Armor对象的向量更新跟踪器
void Tracker::update(const std::vector<rm_auto_aim::Armor> &armors_msg)
{
    // EKF预测
    Eigen::VectorXd ekf_prediction = ekf.predict();

    bool matched = false;
    // 如果找不到匹配的装甲板，则将EKF预测作为默认目标状态
    target_state = ekf_prediction;

    if (!armors_msg.empty())
    {
        auto min_position_diff = DBL_MAX;
        auto yaw_diff = DBL_MAX;

        auto predicted_position = getArmorPositionFromState(ekf_prediction);
        for (const auto &armor : armors_msg)
        {
            // 仅考虑相同ID的装甲板
            if (armor.number == tracked_id)
            {
                auto p = armor.position_world;
                // 当前装甲板位置与跟踪装甲板的预测位置的差异
                double position_diff = (predicted_position - p).norm();
                if (position_diff < min_position_diff)
                {
                    min_position_diff = position_diff;
                    // 计算当前装甲板的姿态与EKF预测的姿态之间的差异
                    yaw_diff = abs(orientationToYaw(armor.rotation_world) - ekf_prediction(6));
                    tracked_armor = armor;
                }
            }
        }

        // 保存差异信息
        info_position_diff = min_position_diff;
        info_yaw_diff = yaw_diff;

        if (min_position_diff < max_match_distance_ && yaw_diff < max_match_yaw_diff_)
        {
            // 找到匹配的装甲板
            matched = true;
            auto p = tracked_armor.position_world;
            // 更新EKF
            double measured_yaw = orientationToYaw(tracked_armor.rotation_world);
            Eigen::Vector4d z(p(0), p(1), p(2), measured_yaw);
            target_state = ekf.update(z);
        }
        else if (yaw_diff > max_match_yaw_diff_) // 此时发生了装甲板跳变
        {
            // 处理装甲板跳变情况
            handleArmorJump(tracked_armor);
        }
        else
        {
            // 装甲板未匹配，追踪状态设置为丢失
            tracker_state = LOST;
        }
    }

    // 防止半径扩散
    target_state(8) = CLAMP(target_state(8), 0.14, 0.33);
    another_r = CLAMP(another_r, 0.14, 0.33);

    // 跟踪状态机
    if (tracker_state == DETECTING)
    {
        if (matched)
        {
            // 如果找到匹配的装甲板，增加检测计数
            detect_count_++;
            if (detect_count_ > tracking_threshold_)
            {
                // 达到跟踪阈值，切换到跟踪状态
                detect_count_ = 0;
                tracker_state = TRACKING;
            }
        }
        else
        {
            // 未找到匹配的装甲板，状态切换到丢失
            detect_count_ = 0;
            tracker_state = LOST;
        }
    }
    else if (tracker_state == TRACKING)
    {
        if (!matched)
        {
            // 如果未找到匹配的装甲板，状态切换到临时丢失
            tracker_state = TEMP_LOST;
            lost_count_++;
        }
    }
    else if (tracker_state == TEMP_LOST) // 可能偶尔遮挡
    {
        if (!matched)
        {
            // 如果未找到匹配的装甲板，增加临时丢失计数
            lost_count_++;
            if (lost_count_ > lost_threshold_)
            {
                // 临时丢失计数超过阈值，状态切换到丢失
                lost_count_ = 0;
                tracker_state = LOST;
            }
        }
        else
        {
            // 如果找到匹配的装甲板，状态切换回跟踪
            tracker_state = TRACKING;
            lost_count_ = 0;
        }
    }
}

// 根据Armor对象初始化EKF
void Tracker::initEKF(const rm_auto_aim::Armor &a)
{
    double xa = a.position_world(0);
    double ya = a.position_world(1);
    double za = a.position_world(2);
    last_yaw_ = 0;
    double yaw = orientationToYaw(a.rotation_world);

    // 将初始位置设置在目标后方0.2米处
    target_state = Eigen::VectorXd::Zero(9);
    double r = 0.20;
    double xc = xa + r * cos(yaw);
    double yc = ya + r * sin(yaw);
    dz = 0, another_r = r;
    target_state << xc, 0, yc, 0, za, 0, yaw, 0, r;

    ekf.setState(target_state);
}

// 处理装甲板跳变的方法
void Tracker::handleArmorJump(const rm_auto_aim::Armor &a)
{
    // 计算当前装甲板的偏航角
    double yaw = orientationToYaw(a.rotation_world);
    // 将偏航角设置为目标状态向量的第6个元素
    target_state(6) = yaw;

    // 更新装甲板编号
    updateArmorsNum(a);

    // 如果跟踪的装甲板数量为NORMAL_4
    if (tracked_armors_num == ArmorsNum::NORMAL_4)
    {
        // 计算高度变化
        dz = target_state(4) - a.position_world(2);
        // 更新目标状态向量的高度为当前装甲板的高度
        target_state(4) = a.position_world(2);
        // 交换目标状态向量的第8个元素和另一个变量(another_r)的值
        std::swap(target_state(8), another_r);
    }

    // 获取当前装甲板的位置
    auto p = a.position_world;

    // 从目标状态向量中推断出的装甲板位置
    Eigen::Vector3d infer_p = getArmorPositionFromState(target_state);

    // 如果当前位置与推断位置的距离大于最大匹配距离
    if ((p - infer_p).norm() > max_match_distance_)
    {
        double r = target_state(8);
        // 重置目标状态向量，将其位置设置在当前装甲板的位置后方
        target_state(0) = p(0) + r * cos(yaw); // xc
        target_state(1) = 0;                   // vxc
        target_state(2) = p(1) + r * sin(yaw); // yc
        target_state(3) = 0;                   // vyc
        target_state(4) = p(2);                // za
        target_state(5) = 0;                   // vza
        std::cout << "Reset state!" << std::endl;
    }

    // 设置扩展卡尔曼滤波器的状态为更新后的目标状态向量(重置卡尔曼滤波器的目标状态)
    ekf.setState(target_state);
}

// 获取装甲板偏航角
double Tracker::orientationToYaw(const Eigen::Matrix3d &R)
{
    double yaw = atan2(R(1, 1), R(0, 1));
    yaw = last_yaw_ + shortest_angular_distance(last_yaw_, yaw);
    last_yaw_ = yaw;
    return yaw;
}

// 从状态向量中获取装甲板位置
Eigen::Vector3d Tracker::getArmorPositionFromState(const Eigen::VectorXd &x)
{
    // 计算当前装甲板的预测位置
    double xc = x(0), yc = x(2), zc = x(4);
    double yaw = x(6), r = x(8);
    double xa = xc - r * cos(yaw);
    double ya = yc - r * sin(yaw);
    return {xa, ya, zc};
}

// 更新装甲板(数量)编号
void Tracker::updateArmorsNum(const rm_auto_aim::Armor &armor)
{
    // 4是平步的大装甲
    if (armor.armor_type == rm_auto_aim::LARGE && (tracked_id == "3" || tracked_id == "4" || tracked_id == "5"))
    {
        tracked_armors_num = ArmorsNum::BALANCE_2;
    }
    else if (tracked_id == "outpost")
    {
        tracked_armors_num = ArmorsNum::OUTPOST_3;
    }
    else
    {
        tracked_armors_num = ArmorsNum::NORMAL_4;
    }
}