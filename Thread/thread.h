#ifndef THREAD_H
#define THREAD_H

#include "Camera.h"
#include "detector.h"
#include "debug.h"
#include "AngleSolver.h"
#include "processor.h"
#include "serialport.h"
#include "DataType.h"
#include <iterator>
#include <thread>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <iostream>
#include <atomic>
#include <opencv2/opencv.hpp>
#include <fmt/format.h>
#include <Eigen/Core>
// 对代码段进行计时
#define TIMEIT(CODE_BLOCK)                                                                                                      \
    do                                                                                                                          \
    {                                                                                                                           \
        auto start = std::chrono::steady_clock::now();                                                                          \
        CODE_BLOCK;                                                                                                             \
        auto end = std::chrono::steady_clock::now();                                                                            \
        auto diff = end - start;                                                                                                \
        fmt::print(fmt::fg(fmt::color::yellow), "Time elapsed:{} ms", std::chrono::duration<double, std::milli>(diff).count()); \
                                                                                                                                \
    } while (0);

#define TIMEIT_ID(ID, CODE_BLOCK)                                                                                                        \
    do                                                                                                                                   \
    {                                                                                                                                    \
        auto start_##ID = std::chrono::steady_clock::now();                                                                              \
        CODE_BLOCK;                                                                                                                      \
        auto end_##ID = std::chrono::steady_clock::now();                                                                                \
        auto diff_##ID = end_##ID - start_##ID;                                                                                          \
        fmt::print(fmt::fg(fmt::color::yellow), "Time elapsed(" #ID "):{} ms", std::chrono::duration<double, std::milli>(diff).count()); \
    } while (0);

using namespace std;
using namespace cv;
template <typename T>
class Factory
{
private:
    std::deque<T> buffer;
    int buffer_size;
    mutex lock;

public:
    /**
     * @brief 工厂类初始化
     * @param size 队列长度
     **/
    Factory(int size)
    {
        buffer_size = size;
    };
    bool produce(T &product);
    bool consume(T &product);
};

template <typename T>
bool Factory<T>::produce(T &product)
{

    lock.lock();                     // 加锁
    if (buffer.size() < buffer_size) // 数据缓存区未满，放入缓存区末尾
        buffer.push_back(product);
    else
    {
        buffer.pop_front(); // 满了删除缓存区头部元素，再放在缓存区末尾
        buffer.push_back(product);
    }
    lock.unlock(); // 解锁

    return true;
}

template <typename T>
bool Factory<T>::consume(T &product)
{
    while (1)
    {
        lock.lock();
        if (!buffer.empty())
            break;
        lock.unlock();
        usleep(1e3);
    }
    product = buffer.front();
    buffer.pop_front();
    lock.unlock();

    return true;
}

class ThreadManager
{

    using Ms = std::chrono::milliseconds;           // ms
    using clk = std::chrono::high_resolution_clock; // clk
    int timeMsCam, timeMsMain;

private:
    // 相机类
    std::unique_ptr<HDURM::HKcam>
        hkcam;
    std::unique_ptr<SerialPort> serial;
    std::unique_ptr<SerialPort> imuSerial;
    std::unique_ptr<rm_auto_aim::Detector> autoAim;
    std::unique_ptr<Processor> processor;
    std::unique_ptr<AngleSolver> angleSolver;

public:
    ThreadManager() = default;

    void InitManager(const std::string &config_file_path);
    bool producer(Factory<TaskData> &factory);
    bool consumer(Factory<TaskData> &factory, Factory<VisionSendData> &transmit_factory, Factory<VisionRecvData> &data_receive_factory);
    bool dataTransmitter(Factory<VisionSendData> &transmit_factory);
    bool dataReceiver(Factory<VisionRecvData> &data_receive_factory);
};

#endif