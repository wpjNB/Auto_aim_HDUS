#include "thread.h"
// 生产者

void ThreadManager::InitManager(const std::string &config_file_path)
{
  hkcam = std::make_unique<HDURM::HKcam>(config_file_path);
  autoAim = std::make_unique<rm_auto_aim::Detector>(config_file_path);
  // 定义与主控板串口
  serial = std::make_unique<SerialPort>("/dev/ttyUSB0", 115200);
// 定义与IMU惯导
#ifdef isIMU
  imuSerial = std::make_unique<SerialPort>("/dev/IMU", 115200);
#endif
  processor = std::make_unique<Processor>(config_file_path);
  angleSolver = std::make_unique<AngleSolver>("/home/wpj/RM_Vision_code_US/auto_aim_HDUS/AngleSolver/XML/out_camera_data1.xml", config_file_path);
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
    auto time_ms = std::chrono::duration_cast<Ms>(end - start).count();
    // fmt::print(fmt::fg(fmt::color::black), "{}\n", time_ms);
  }

  hkcam->CloseCam();
  return true;
}
// 消费者
bool ThreadManager::consumer(Factory<TaskData> &factory, Factory<VisionSendData> &transmit_factory, Factory<VisionRecvData> &data_receive_factory)
{
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
    revData = data_receive_factory.consume(revData);
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
      // 率选1个目标装甲板（目标中心最近的）
      auto oneArmor = new rm_auto_aim::Armor();
      float minDis = 100000;
      for (auto &armor : autoAim->True_armors)
      {
        if (armor.distance_to_image_center < minDis)
        {
          minDis = armor.distance_to_image_center;
          oneArmor = &armor;
        }
      }
      if (autoAim->ArmorState == rm_auto_aim::ARMOR_FOUND)
      {
        if (abs(oneArmor->yaw) < 0.5f || abs(oneArmor->yaw) < 0.5f)
        {
          oneArmor->yaw = 0;
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
      autoAim->showDebuginfo(oneArmor->pitch, oneArmor->yaw, oneArmor->dis, oneArmor->position_cam);
#endif
      // processor->processArmor(autoAim->True_armors, target_msg);

      // if (processor->tracker->tracker_state != Tracker::State::LOST)
      // {
      //   angleSolver->CalcFinalAngle(target_msg, revData, sendData, processor->tracker->tracked_armors_num);
      //   sendData.yaw_angle.f += bias_yaw;
      //   sendData.pitch_angle.f += bias_pitch;
      //   sendData.isFire = 1;
      // }
      // else
      // {
      //   sendData.isFire = 0;
      //   sendData.pitch_angle.f = revData.gimbal_pitch * 180 / CV_PI;
      //   sendData.yaw_angle.f = revData.gimbal_yaw * 180 / CV_PI;
      // }

      transmit_factory.produce(sendData);
      // 结束计时
      auto end = clk::now();
      auto time_ms = std::chrono::duration_cast<Ms>(end - start).count();
      // fmt::print(fmt::fg(fmt::color::black), "{}\n", time_ms);
    }
    /*-----------------------------------自瞄----------------------------------------------------*/
  }
  return true;
}
// 串口发送线程
bool ThreadManager::dataTransmitter(Factory<VisionSendData> &transmit_factory)
{

  while (1)
  {
    VisionSendData data;
    transmit_factory.consume(data);
    // 若串口离线即初始化失败则跳过数据发送
    // TODO:使用无串口的模式时会导致此线程死循环，浪费CPU性能
    if (serial->need_init == true)
    {
      usleep(5000);
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
