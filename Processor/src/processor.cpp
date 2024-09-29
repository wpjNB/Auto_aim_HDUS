#include "processor.h"

Processor::Processor(const std::string &config_file_path)

{
    dt_ = 0.01;
    cv::FileStorage config(config_file_path, cv::FileStorage::READ);
    tracker = std::make_unique<Tracker>(config_file_path);
    // EKF
    // xa = x_armor, xc = x_robot_center
    // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
    // measurement: xa, ya, za, yaw
    // f - 过程函数
    auto f = [this](const Eigen::VectorXd &x)
    {
        Eigen::VectorXd x_new = x;
        x_new(0) += x(1) * dt_;
        x_new(2) += x(3) * dt_;
        x_new(4) += x(5) * dt_;
        x_new(6) += x(7) * dt_;
        return x_new;
    };

    // J_f - 过程函数的雅可比矩阵
    auto j_f = [this](const Eigen::VectorXd &)
    {
        Eigen::MatrixXd f(9, 9);
        // clang-format off
        f <<  1,   dt_, 0,   0,   0,   0,   0,   0,   0,
              0,   1,   0,   0,   0,   0,   0,   0,   0,
              0,   0,   1,   dt_, 0,   0,   0,   0,   0,
              0,   0,   0,   1,   0,   0,   0,   0,   0,
              0,   0,   0,   0,   1,   dt_, 0,   0,   0,
              0,   0,   0,   0,   0,   1,   0,   0,   0,
              0,   0,   0,   0,   0,   0,   1,   dt_, 0,
              0,   0,   0,   0,   0,   0,   0,   1,   0,
              0,   0,   0,   0,   0,   0,   0,   0,   1;
        // clang-format on
        return f;
    };

    // h - 观测函数
    auto h = [](const Eigen::VectorXd &x)
    {
        Eigen::VectorXd z(4);
        double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
        z(0) = xc - r * cos(yaw); // xa
        z(1) = yc - r * sin(yaw); // ya
        z(2) = x(4);              // za
        z(3) = x(6);              // yaw
        return z;
    };
    // J_h - 观测函数的雅可比矩阵
    auto j_h = [](const Eigen::VectorXd &x)
    {
        Eigen::MatrixXd h(4, 9);
        double yaw = x(6), r = x(8);
        // clang-format off
        //    xc   v_xc yc   v_yc za   v_za yaw            v_yaw r
        h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,    -cos(yaw),
              0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,    -sin(yaw),
              0,   0,   0,   0,   1,   0,   0,             0,    0,
              0,   0,   0,   0,   0,   0,   1,             0,    0;
        // clang-format on
        return h;
    };

    config["processor"]["s2_q_xyz"] >> s2qxyz_;
    config["processor"]["s2_q_yaw"] >> s2qyaw_;
    config["processor"]["s2_q_r"] >> s2qr_;

    // update_Q - 过程噪声协方差矩阵
    auto u_q = [this]()
    {
        Eigen::MatrixXd q(9, 9);
        double t = dt_, x = s2qxyz_, y = s2qyaw_, r = s2qr_;
        double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
        double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
        double q_r = pow(t, 4) / 4 * r;
        // clang-format off
        //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
        q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
              q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
              0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
              0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
              0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
              0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
              0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
              0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
              0,      0,      0,      0,      0,      0,      0,      0,      q_r;
        // clang-format on
        return q;
    };

    config["processor"]["r_xyz_factor"] >> r_xyz_factor;
    config["processor"]["r_yaw"] >> r_yaw;

    // update_R - 测量噪声协方差矩阵
    auto u_r = [this](const Eigen::VectorXd &z)
    {
        Eigen::DiagonalMatrix<double, 4> r;
        double x = r_xyz_factor;
        r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
        return r;
    };

    // P - 误差估计协方差矩阵
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();

    // 初始化EKF
    tracker->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};
}

void Processor::processArmor(std::vector<rm_auto_aim::Armor> &armors_msg, TargetInfo &target_msg)
{

    auto time = std::chrono::steady_clock::now();
    if (tracker->tracker_state == Tracker::LOST)
    {
        // 目标丢失，进行初始化
        tracker->init(armors_msg);
    }
    else
    {
        // 计算当前时间和上一次记录时间之间的时间差
        dt_ = (double)std::chrono::duration_cast<Us>(time - time_elapsed_).count() / 1e6;
        // 目标跟踪
        tracker->update(armors_msg);
    }
    time_elapsed_ = time;
    const auto state = tracker->target_state;
    // 更新目标信息
    // 将状态值分配给target_msg对象
    target_msg.x = (float)state(0);
    target_msg.vx = (float)state(1);
    target_msg.y = (float)state(2);
    target_msg.vy = (float)state(3);
    target_msg.z = (float)state(4);
    target_msg.vz = (float)state(5);
    target_msg.yaw = (float)state(6);
    target_msg.w_yaw = (float)state(7);
    target_msg.radius_1 = (float)state(8);
    target_msg.radius_2 = (float)tracker->another_r;
    target_msg.dz = (float)tracker->dz;
    if (tracker->tracked_armor.number == "1")
    {
        target_msg.type = TargetType::hero;
    }
    else if (tracker->tracked_armor.number == "2")
    {
        target_msg.type = TargetType::engineer;
    }
    else if (tracker->tracked_armor.number == "3")
    {
        target_msg.type = TargetType::infantry3;
    }
    else if (tracker->tracked_armor.number == "4")
    {
        target_msg.type = TargetType::infantry4;
    }
    else if (tracker->tracked_armor.number == "5")
    {
        target_msg.type = TargetType::infantry5;
    }
    else if (tracker->tracked_armor.number == "outpost")
    {
        target_msg.type = TargetType::outpost;
    }
    else if (tracker->tracked_armor.number == "guard")
    {
        target_msg.type = TargetType::guard;
    }
    else if (tracker->tracked_armor.number == "base")
    {
        target_msg.type = TargetType::base;
    }
    else if (tracker->tracked_armor.number == "negative")
    {
        target_msg.type = TargetType::none;
    }
}
