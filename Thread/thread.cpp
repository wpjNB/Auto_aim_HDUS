#include "thread.h"
#include <cmath> // for std::abs with float

// 生产者

void ThreadManager::InitManager(const std::string &config_file_path)
{
  hkcam = std::make_unique<HDURM::HKcam>(config_file_path);
  autoAim = std::make_unique<rm_auto_aim::Detector>(config_file_path);
  // 定义与主控板串口
  serial = std::make_unique<SerialPort>("/dev/ttyUSB1", 921600);
// 定义与IMU惯导(需要自己串口名称重定向为/dev/IMU,因为串口会跳变)
#ifdef isIMU
  imuSerial = std::make_unique<SerialPort>("/dev/IMU", 921600);
#endif
  processor = std::make_unique<Processor>(config_file_path);
  angleSolver = std::make_unique<AngleSolver>("./AngleSolver/XML/out_camera_data1.xml", config_file_path);
  fmt::print(fmt::fg(fmt::color::yellow), " ==  == == == == == == == == == == == == == == \n");
  fmt::print(fmt::fg(fmt::color::blue), "Init all modules \n");
  fmt::print(fmt::fg(fmt::color::yellow), " ==  == == == == == == == == == == == == == == \n");
}

bool ThreadManager::producer(Factory<TaskData> &factory)
{

  hkcam->OpenCam("2BDFA2166410");
  hkcam->SetParam();

  while (1)
  {
    // 产生数据
    TaskData src;
    // 开始计时
    auto start = clk::now();
    auto ret = hkcam->GetFlame(src.img);

    if (!ret)
    {
      break;
    }

    factory.produce(src);
    // 结束计时
    auto end = clk::now();
    timeMsCam = std::chrono::duration_cast<Ms>(end - start).count();
    // fmt::print(fmt::fg(fmt::color::black), "{}\n", timeMsCam);
  }

  hkcam->CloseCam();
  return true;
}
// 消费者
bool ThreadManager::consumer(Factory<TaskData> &factory, Factory<VisionSendData> &transmit_factory, Factory<VisionRecvData> &data_receive_factory)
{
  constexpr float MIN_ANGLE_THRESHOLD = 0.5f; // 最小角度阈值，小于此值时将角度设为0
  constexpr float MAX_DISTANCE_INITIAL = 100000.0f; // 初始最大距离值
  
  auto mode = 0, last_mode = 0;
  float bias_pitch = 0, bias_yaw = 0;
  // 自瞄类

  while (1)
  {
    TaskData dst;
    VisionSendData sendData;
    VisionRecvData revData;
    TargetInfo target_msg;
    // 从相机线程读图像数据
    factory.consume(dst);

    // 接受电控数据(无电控数据无法进行预测)
#ifdef isIMU
    data_receive_factory.consume(revData);
#else

#endif
    // 子弹速度写死
    revData.bullet_speed = BulletSpeed::infantry18;
    /*-----------------------------------自瞄---------------------------------------------------*/
    {
      // 开始计时
      auto start = clk::now();
#ifdef DETEDRED
      autoAim->run(dst.img, RED);
#else
      autoAim->run(dst.img, BLUE);
#endif
      // 将装甲位置从相机坐标系转换为世界坐标

      for (auto &armor : autoAim->True_armors)
      {
        // 使用转换解算器将装甲位置从图像帧转换为世界坐标
        angleSolver->GetTransformation(armor, revData);
      }
      // processor->processArmor(autoAim->True_armors, target_msg);
      // 筛选1个目标装甲板（目标中心最近的）
      rm_auto_aim::Armor *oneArmor = nullptr;
      float minDis = MAX_DISTANCE_INITIAL;
      for (auto &armor : autoAim->True_armors)
      {
        if (armor.distance_to_image_center < minDis)
        {
          minDis = armor.distance_to_image_center;
          oneArmor = &armor;
        }
      }
      if (autoAim->ArmorState == rm_auto_aim::ARMOR_FOUND && oneArmor != nullptr)
      {
        // 单独检查每个角度，如果过小则置零
        if (std::abs(oneArmor->yaw) < MIN_ANGLE_THRESHOLD)
        {
          oneArmor->yaw = 0;
        }
        if (std::abs(oneArmor->pitch) < MIN_ANGLE_THRESHOLD)
        {
          oneArmor->pitch = 0;
        }
        sendData.yaw_angle.f = oneArmor->yaw;
        sendData.pitch_angle.f = oneArmor->pitch;
        sendData.dis.f = oneArmor->dis;
        sendData.isFire = 1;
      }
      else
      {
        sendData.isFire = 0;
        sendData.pitch_angle.f = 0;
        sendData.yaw_angle.f = 0;
      }
#ifdef UsingShowImg
      // draw all Armor
      autoAim->drawResults(dst.img);
      // draw target Armor
      if (oneArmor != nullptr)
      {
        autoAim->showDebuginfo(dst.img, *oneArmor);
      }
      cv::putText(dst.img, cv::format(" aimT: %dms", timeMsMain), cv::Point(1100, 30), cv::FONT_HERSHEY_SIMPLEX, 0.67, cv::Scalar(0, 255, 0), 1);
      cv::putText(dst.img, cv::format(" camT: %dms", timeMsCam), cv::Point(1100, 60), cv::FONT_HERSHEY_SIMPLEX, 0.67, cv::Scalar(0, 255, 0), 1);
      cv::imshow("test", dst.img);
      cv::waitKey(1);
#endif

      transmit_factory.produce(sendData);
      // 结束计时
      auto end = clk::now();
      timeMsMain = std::chrono::duration_cast<Ms>(end - start).count();
      // fmt::print(fmt::fg(fmt::color::black), "{}\n", timeMsMain);
    }
    /*-----------------------------------自瞄----------------------------------------------------*/
  }
  return true;
}
// 串口发送线程
bool ThreadManager::dataTransmitter(Factory<VisionSendData> &transmit_factory)
{
  constexpr int SERIAL_RETRY_DELAY_US = 5000; // 5ms delay when serial is offline
  
  while (1)
  {
    VisionSendData data;
    transmit_factory.consume(data);
    
    // 若串口离线即初始化失败则跳过数据发送
    if (serial->need_init == true)
    {
      // Sleep to reduce CPU usage when serial is unavailable
      usleep(SERIAL_RETRY_DELAY_US);
      continue;
    }
    serial->send(data);
  }
  return true;
}
// 串口接收线程
bool ThreadManager::dataReceiver(Factory<VisionRecvData> &data_receive_factory)
{

  while (1)
  {
    VisionRecvData data;
    if (imuSerial->ReceiveData(data))
    {
      data_receive_factory.produce(data);
    }

#ifdef DEBUG_SHOW

#endif
  }
  return true;
}
