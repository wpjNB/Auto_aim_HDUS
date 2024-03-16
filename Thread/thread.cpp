#include "thread.h"
// 生产者
bool producer(Factory<TaskData> &factory)
{
    // 相机类
    HDURM::HKcam hkcam;
    hkcam.OpenCam("2BDFA2166410");
    hkcam.SetParam();

    while (1)
    {
        // 产生数据
        TaskData src;
        auto ret = hkcam.GetFlame(src.img);

        if (!ret)
        {
            break;
        }

        factory.produce(src);
    }

    hkcam.CloseCam();
    return true;
}
// 消费者
bool consumer(Factory<TaskData> &factory)
{
    auto mode = 0, last_mode = 0;
    // 自瞄类
    rm_auto_aim::Detector detectorArmor;
    // 串口发送数据
    VisionData SendData;
    SerialPort serial("/dev/ttyUSB0", 115200);
    // 姿态解算类
    AngleSolver solver;
    solver.Init("/home/wpj/RM_Vision_code_US/auto_aim_HDUS/AngleSolver/XML/out_camera_data.xml", 0, 0, 0);

    while (1)
    {
        // 从缓存区取数据
        TaskData dst;
        factory.consume(dst);

        rm_auto_aim::Armor targetArmor;
        float pitch, yaw, dis, xyz[3];
        mode = dst.mode;

        if (mode != last_mode)
        {
            fmt::print(fmt::fg(fmt::color::pale_violet_red), "[CONSUMER] Mode switched to {}\n", mode);
            last_mode = mode;
        }

        double t1 = (double)cv::getTickCount();

        detectorArmor.run(dst.img, rm_auto_aim::BLUE, targetArmor);
        if (detectorArmor.ArmorState == rm_auto_aim::ARMOR_FOUND)
        {
            solver.solve_angle(targetArmor);
            solver.GetAngle(pitch, yaw, dis, xyz);
            if (fabs(SendData.pitch_angle.f - pitch) > 2 || fabs(SendData.yaw_angle.f - yaw) > 2)
            {
                SendData = {pitch, yaw, dis, 0, 1, 0, 0};
            }
        }
        else // 没有找到装甲板
        {
            SendData = {0, 0, 0, 0, 0, 0, 0};
        }
        serial.send(SendData);
        double t2 = (double)cv::getTickCount();
        int fps = cv::getTickFrequency() / (t2 - t1);
#ifdef UsingShowPYD
        detectorArmor.showDebuginfo(SendData.pitch_angle.f, SendData.yaw_angle.f, SendData.dis.f, fps);
        waitKey(1);
#endif
    }

    return true;
}