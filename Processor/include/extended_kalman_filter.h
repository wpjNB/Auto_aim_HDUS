// 版权所有 2022 陈军

#ifndef EXTENDED_KALMAN_FILTER_HPP_
#define EXTENDED_KALMAN_FILTER_HPP_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <functional>

class ExtendedKalmanFilter
{
public:
    /**
     * @brief 构造函数
     */
    ExtendedKalmanFilter() = default;

    /**
     * @brief 非线性向量函数
     */
    using VecVecFunc = std::function<Eigen::VectorXd(const Eigen::VectorXd &)>;

    /**
     * @brief 非线性矩阵函数
     */
    using VecMatFunc = std::function<Eigen::MatrixXd(const Eigen::VectorXd &)>;

    /**
     * @brief 无参矩阵函数
     */
    using VoidMatFunc = std::function<Eigen::MatrixXd()>;

    /**
     * @brief 构造函数
     * @param f 非线性向量函数
     * @param h 非线性向量函数
     * @param j_f f()的雅可比矩阵函数
     * @param j_h h()的雅可比矩阵函数
     * @param u_q 过程噪声协方差矩阵函数
     * @param u_r 测量噪声协方差矩阵函数
     * @param P0 前置估计误差估计协方差矩阵
     */
    explicit ExtendedKalmanFilter(
        VecVecFunc f, VecVecFunc h, VecMatFunc j_f, VecMatFunc j_h,
        VoidMatFunc u_q, VecMatFunc u_r, const Eigen::MatrixXd &P0);

    /**
     * @brief 设置初始状态
     * @param x0 初始状态向量
     */
    void setState(const Eigen::VectorXd &x0);

    /**
     * @brief 计算预测状态
     * @return 预测状态矩阵
     */
    Eigen::MatrixXd predict();

    /**
     * @brief 根据测量更新估计状态
     * @param z 测量向量
     * @return 更新后的估计状态矩阵
     */
    Eigen::MatrixXd update(const Eigen::VectorXd &z);

private:
    // 非线性向量函数
    VecVecFunc f;
    // 观测非线性向量函数
    VecVecFunc h;
    // f()的雅可比矩阵函数
    VecMatFunc jacobian_f;
    Eigen::MatrixXd F;
    // h()的雅可比矩阵函数
    VecMatFunc jacobian_h;
    Eigen::MatrixXd H;
    // 过程噪声协方差矩阵函数
    VoidMatFunc update_Q;
    Eigen::MatrixXd Q;
    // 测量噪声协方差矩阵函数
    VecMatFunc update_R;
    Eigen::MatrixXd R;

    // 前置估计误差估计协方差矩阵
    Eigen::MatrixXd P_pri;
    // 后置估计误差估计协方差矩阵
    Eigen::MatrixXd P_post;

    // 卡尔曼增益
    Eigen::MatrixXd K;

    // 系统维度
    int n{};

    // N-size 身份矩阵
    Eigen::MatrixXd I;

    // 前置估计状态
    Eigen::VectorXd x_pri;
    // 后置估计状态
    Eigen::VectorXd x_post;
};

#endif // EXTENDED_KALMAN_FILTER_HPP_
