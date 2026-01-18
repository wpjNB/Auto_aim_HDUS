#include "thread.h"

int main()
{
  // 工厂模板类 - 使用生产者消费者模式
  Factory<TaskData> task_factory(ThreadConstants::DEFAULT_TASK_BUFFER_SIZE);
  Factory<VisionSendData> data_transmit_factory(ThreadConstants::DEFAULT_TRANSMIT_BUFFER_SIZE);
  Factory<VisionRecvData> data_receive_factory(ThreadConstants::DEFAULT_RECEIVE_BUFFER_SIZE);

  ThreadManager thread_manager;
  thread_manager.InitManager("./config.yaml");
  
  /*--------串口发送线程--------*/
  std::thread transmitter(&ThreadManager::dataTransmitter, &thread_manager, std::ref(data_transmit_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Transmitter started\n");
  
  /*--------相机更新线程--------*/
  std::thread task_producer(&ThreadManager::producer, &thread_manager, std::ref(task_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Producer started\n");
  
  /*--------自瞄线程-----------*/
  std::thread task_consumer(&ThreadManager::consumer, &thread_manager, std::ref(task_factory), std::ref(data_transmit_factory), std::ref(data_receive_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Consumer started\n");
  
  /*--------接受线程-----------*/
  std::thread receiver(&ThreadManager::dataReceiver, &thread_manager, std::ref(data_receive_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Receiver started\n");

  transmitter.join();
  task_producer.join();
  task_consumer.join();
#ifdef isIMU
  fmt::print(fmt::fg(fmt::color::blue), "IMU started\n");
  receiver.join();
#endif
  return 0;
}
