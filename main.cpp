
#include "thread.h"
int main()
{
    // 工厂模板类
    Factory<TaskData> task_factory(3);
    thread task_producer(&producer, ref(task_factory));
    fmt::print(fmt::fg(fmt::color::blue), "Producer start !!!!!!!!!!\n");
    thread task_consumer(&consumer, ref(task_factory));
    fmt::print(fmt::fg(fmt::color::blue), "Consumer start !!!!!!!!!!\n");
    task_producer.join();
    task_consumer.join();
    return 0;
}