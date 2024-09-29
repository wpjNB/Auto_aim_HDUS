#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <vector>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <armor.h>
#include "DataType.h"
#include "tracker.h"
#include "AngleSolver.h"
class Processor
{

    using Us = std::chrono::microseconds; // 微妙
    using Ms = std::chrono::milliseconds; // 毫秒
    using Ss = std::chrono::seconds;      // 秒
    using Ns = std::chrono::nanoseconds;  // 纳秒

public:
private:
    double s2qxyz_, s2qyaw_, s2qr_;

    double r_xyz_factor, r_yaw;

    std::chrono::steady_clock::time_point time_elapsed_;
    double dt_; // time interval between two frames

public:
    Processor() = default;
    Processor(const std::string &config_file_path);
    void processArmor(std::vector<rm_auto_aim::Armor> &armors_msg, TargetInfo &target_msg);
    std::unique_ptr<Tracker> tracker;
};

#endif // PROCESSOR_H
