// 版权所有 2022 陈军

#ifndef ARMOR_PROCESSOR_TRACKER_HPP_
#define ARMOR_PROCESSOR_TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>
#include "armor.h"
#include "extended_kalman_filter.h"
#include "DataType.h"
// STD
#include <memory>
#include <vector>

class Tracker
{
public:
    /**
     * @brief 构造函数，通过配置节点初始化跟踪器
     * @param cfg_node 配置节点
     */
    explicit Tracker(const std::string &config_file_path);

    /**
     * @brief 构造函数，通过参数初始化跟踪器
     * @param max_match_distance 最大匹配距离
     * @param tracking_threshold 跟踪阈值
     * @param lost_threshold 失踪阈值
     */
    Tracker(double max_match_distance, int tracking_threshold, int lost_threshold);

    /**
     * @brief 初始化跟踪器，设置跟踪目标
     * @param armors_msg 目标列表
     */
    void init(const std::vector<rm_auto_aim::Armor> &armors_msg);

    /**
     * @brief 更新跟踪器，更新目标状态
     * @param armors_msg 目标列表
     */
    void update(const std::vector<rm_auto_aim::Armor> &armors_msg);

    /**
     * @brief 跟踪器状态
     */
    enum State
    {
        LOST,      ///< 失踪状态
        DETECTING, ///< 检测状态
        TRACKING,  ///< 跟踪状态
        TEMP_LOST, ///< 临时失踪状态
    } tracker_state;

    /**
     * @brief 扩展卡尔曼滤波器
     */
    ExtendedKalmanFilter ekf;

    /**
     * @brief 跟踪目标的数量
     */
    ArmorsNum tracked_armors_num;

    /**
     * @brief 当前跟踪的目标
     */
    rm_auto_aim::Armor tracked_armor;

    /**
     * @brief 当前跟踪的目标ID
     */
    std::string tracked_id;

    /**
     * @brief 目标的状态向量
     */
    Eigen::VectorXd target_state;

    /**
     * @brief 位置差的信息
     */
    double info_position_diff;

    /**
     * @brief 回转差的信息
     */
    double info_yaw_diff;

    /**
     * @brief 存储另一个目标的消息
     */
    double dz{}, another_r{};

    /**
     * @brief 跳跃计数器
     */
    static int i_jump;

    /**
     * @brief 计算四元数方向到俯仰角的函数
     * @param R 四元数方向矩阵
     * @return 俯仰角
     */
    double orientationToYaw(const Eigen::Matrix3d &R);

private:
    /**
     * @brief 初始化扩展卡尔曼滤波器
     * @param a 目标消息
     */
    void initEKF(const rm_auto_aim::Armor &a);

    /**
     * @brief 处理目标跳跃
     * @param a 目标消息
     */
    void handleArmorJump(const rm_auto_aim::Armor &a);

    /**
     * @brief 获取目标位置
     * @param x 目标状态向量
     * @return 目标位置向量
     */
    static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x);

    /**
     * @brief 更新跟踪目标的数量
     * @param armor 目标消息
     */
    void updateArmorsNum(const rm_auto_aim::Armor &armor);

    /**
     * @brief 最大匹配距离
     */
    double max_match_distance_{};

    /**
     * @brief 最大匹配俯仰角差
     */
    double max_match_yaw_diff_{};

    /**
     * @brief 跟踪阈值
     */
    int tracking_threshold_{};

    /**
     * @brief 失踪阈值
     */
    int lost_threshold_{};

    /**
     * @brief 检测次数
     */
    int detect_count_{};

    /**
     * @brief 失踪次数
     */
    int lost_count_{};

    /**
     * @brief 上一次的俯仰角
     */
    double last_yaw_{};

    static inline double normalize_angle(double angle)
    {
        const double result = fmod(angle + M_PI, 2.0 * M_PI);
        if (result <= 0.0)
            return result + M_PI;
        return result - M_PI;
    }

    /*!
     * \function
     * \brief shortest_angular_distance
     *
     * Given 2 angles, this returns the shortest angular
     * difference.  The inputs and ouputs are of course radians.
     *
     * The result
     * would always be -pi <= result <= pi.  Adding the result
     * to "from" will always get you an equivelent angle to "to".
     */

    static inline double shortest_angular_distance(double from, double to)
    {
        return normalize_angle(to - from);
    }
};

#endif // ARMOR_PROCESSOR_TRACKER_HPP_
