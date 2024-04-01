
#include "thread.h"
int main()
{
  // 工厂模板类
  Factory<TaskData> task_factory(3);
  Factory<VisionData> data_transmit_factory(5);
  /*--------串口发送线程----------*/
  thread transmitter(&dataTransmitter, ref(data_transmit_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Transmitter start !!!!!!!!!!\n");
  /*--------相机更新线程---------*/
  thread task_producer(&producer, ref(task_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Producer start !!!!!!!!!!\n");
  /*--------自瞄线程--------*/
  thread task_consumer(&consumer, ref(task_factory), ref(data_transmit_factory));
  fmt::print(fmt::fg(fmt::color::blue), "Consumer start !!!!!!!!!!\n");

  task_producer.join();
  task_consumer.join();
  return 0;
}