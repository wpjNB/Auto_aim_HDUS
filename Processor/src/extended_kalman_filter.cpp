// 版权 2022 陈军

#include "extended_kalman_filter.h"

#include <utility>

/**
 * @brief 扩展卡尔曼滤波器类
 *
 * @param f 系统状态转移函数
 * @param h 测量函数
 * @param j_f jacobian_f 系统状态转移函数的雅可比矩阵函数
 * @param j_h jacobian_h 测量函数的雅可比矩阵函数
 * @param u_q update_Q 更新Q矩阵的函数
 * @param u_r update_R 更新R矩阵的函数
 * @param P0 初始状态协方差矩阵
 */
ExtendedKalmanFilter::ExtendedKalmanFilter(VecVecFunc f, VecVecFunc h, VecMatFunc j_f, VecMatFunc j_h, VoidMatFunc u_q, VecMatFunc u_r, const Eigen::MatrixXd &P0)
    : f(std::move(f)),
      h(std::move(h)),
      jacobian_f(std::move(j_f)),
      jacobian_h(std::move(j_h)),
      update_Q(std::move(u_q)),
      update_R(std::move(u_r)),
      P_post(P0),
      n((int)P0.rows()),
      I(Eigen::MatrixXd::Identity(n, n)),
      x_pri(n),
      x_post(n)
{
}

/**
 * @brief 设置状态
 *
 * @param x0 状态向量
 */
void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x0) { x_post = x0; }

/**
 * @brief 预测状态
 *
 * @return 预测状态向量
 */
Eigen::MatrixXd ExtendedKalmanFilter::predict()
{
    F = jacobian_f(x_post), Q = update_Q();

    x_pri = f(x_post);
    P_pri = F * P_post * F.transpose() + Q;

    // 处理下一个预测之前没有测量的情况
    x_post = x_pri;
    P_post = P_pri;

    return x_pri;
}

/**
 * @brief 更新状态
 *
 * @param z 测量向量
 * @return 更新后的状态向量
 */
Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z)
{
    H = jacobian_h(x_pri), R = update_R(z);

    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post = x_pri + K * (z - h(x_pri));
    P_post = (I - K * H) * P_pri;

    return x_post;
}
