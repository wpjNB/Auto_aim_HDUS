# 花椒步兵视觉开源方案 ReelSteel (Auto_aim_HDUS)

------

华东交通大学2024赛季3V3对抗赛视觉开源框架

## 项目改进 (Recent Refactoring)

本项目进行了全面的代码重构和优化，主要改进包括：

### 代码质量改进
- ✅ 修复了内存泄漏问题（thread.cpp中的new/delete不匹配）
- ✅ 移除了所有硬编码的绝对路径，改用相对路径
- ✅ 移除了头文件中的`using namespace`声明，避免命名空间污染
- ✅ 添加了空指针检查，提高代码健壮性
- ✅ 将魔法数字提取为命名常量，提高代码可读性
- ✅ 添加const正确性，提高代码安全性

### 构建系统改进
- ✅ 使用现代CMake实践重写CMakeLists.txt
- ✅ 添加了版本信息和构建配置摘要
- ✅ 改进了编译器警告设置
- ✅ 优化了.gitignore配置

### 错误处理改进
- ✅ 添加了配置文件打开失败的异常处理
- ✅ 改进了串口离线时的CPU使用率问题
- ✅ 添加了FileStorage资源的正确释放

### 文档改进
- ✅ 统一了代码注释风格
- ✅ 添加了配置常量的说明性注释
- ✅ 改进了函数参数的const修饰

#### 本开源参考：沈阳航空航天TUP2022，深大2019，上交2021,湖大2023开源

**新手导向**：

- RM开源汇总：https://bbs.robomaster.com/forum.php?mod=forumdisplay&fid=63&filter=typeid&typeid=167
- RMhttps://docs.qq.com/sheet/DUFlaU0FHZk1QS0l1?tab=bb08j2
- 上海交龙战队博客：https://sjtu-robomaster-team.github.io/
- 君佬rm_vision视觉开源：https://github.com/rm-vision-archive
- 君佬的完整弹道模型：https://github.com/CodeAlanqian/SolveTrajectory
- 北极熊战队：https://flowus.cn/lihanchen/share/facb28a9-5d34-42a7-9bc8-630a182c3571
- 湖南跃鹿视觉教程：https://blog.csdn.net/NeoZng/article/details/126283713?spm=1001.2014.3001.5502
- Robomaster——关于视觉组，你想要了解的都在这里:https://blog.csdn.net/weixin_42754478/article/details/108159529#comments_29363840
- 

## 1. 简介

1. ##### **硬件选型**：

- **工业相机**：海康威视MV-CS016-10UC +**搭配镜头**：8mm焦距镜头
- 普通usb摄像头

2. **功能：**

- 自瞄

3. **未完成**：

- kai huo ce lv
- 能量机关
- 吊射前哨站
- 预测，反小陀螺

4. **部署需要调节参数**：

   - 相机的曝光与增益(在曝光达不到要求时，可以尝试提高增益)

   - 相机内参矩阵与外参矩阵XML路径更改

   - 相机坐标系到枪管坐标系水平(X,Y,Z)偏移量与旋转(Pitch,Yaw)偏移量

   - 传统视觉识别参数，如二值化阈值,灯条装甲板几何约束参数

   - USB虚拟串口设备名字

   - inference model 路径更改

   - 根据比赛场地调整二值化阈值，曝光和增益。

   - 静止击打装甲板（调整bias_pitch和bias_yaw)，如果近低远高就减小摩擦系数抬高pitch

   - 取消所有调试窗口，在debug.h头文件

   - 调整开火策略，决定自瞄发射的速度。

   - 反小陀螺调参，主要调延迟时间，要是角速度非常不准就重新适当调整曝光和增益

   - usb易松动可用海绵垫
   
   - 相机可加散热片

   

     

5. **设计模式：**

​		生产者消费者模式



## 2. 环境配置

- ubuntu 22.04
- 海康相机库：https://www.hikrobotics.com/cn/machinevision/service/download?module=0（编译安装时给权限）
- gcc 11.4.0
- CMake 3.22.1
- OpenCV 4.4.0
- Fmt 编译安装(https://github.com/fmtlib/fmt)
- Glog 编译安装([ https://github.com/google/glog/releases/tag/v0.5.0](https://github.com/google/glog/releases/tag/v0.5.0))
- Ceres ([ http://ceres-solver.org/installation.html](http://ceres-solver.org/installation.html))
- Eigen (https://gitlab.com/libeigen/eigen/-/releases/3.4.0)或者直接sudo apt-get install libeigen3-dev

### 编译说明

```bash
# 1. 克隆项目
git clone https://github.com/wpjNB/Auto_aim_HDUS.git
cd Auto_aim_HDUS

# 2. 创建构建目录
mkdir build && cd build

# 3. 配置和编译
cmake ..
make -j$(nproc)

# 4. 运行（确保config.yaml和相关模型文件在正确位置）
./HDUS
```

**注意**：
- 确保所有配置文件（config.yaml, 相机参数XML等）与可执行文件在同一目录
- 模型文件应放在 `./ArmorDetector/model/` 目录下
- 相机参数文件应放在 `./HKCamera/XML/` 和 `./AngleSolver/XML/` 目录下


## 3.文件结构

```
.
├── AngleSolver          姿态解算        
│   ├── include
│   │   └── AngleSolver.h
│   ├── src
│   │   └── AngleSolver.cpp
│   └── XML
│       ├── out_camera_data1.xml
│       ├── out_camera_data_pre.xml
│       └── out_camera_data.xml
├── ArmorDetector        自瞄
│   ├── include
│   │   ├── armor.h
│   │   ├── detector.h
│   │   └── number_classifier.h
│   ├── model
│   │   ├── fc.onnx
│   │   ├── label.txt
│   │   └── mlp.onnx
│   └── src
│       ├── detector.cpp
│       └── number_classifier.cpp
├── CMakeLists.txt
├── data
│   └── README.md
├── debug.h              宏定义开关
├── docs
│   └── image.png
├── general
│   ├── general.cpp
│   └── general.h
├── HKCamera             相机
│   ├── include
│   │   ├── Camera.h
│   │   ├── CameraParams.h
│   │   ├── MvCameraControl.h
│   │   ├── MvErrorDefine.h
│   │   └── PixelType.h
│   ├── libs
│   │   └── linux
│   │       ├── libFormatConversion.so
│   │       ├── libMediaProcess.so
│   │       ├── libMvCameraControl.so
│   │       ├── libMVGigEVisionSDK.so
│   │       ├── libMVRender.so
│   │       └── libMvUsb3vTL.so
│   ├── src
│   │   └── Camera.cpp
│   └── XML
│       └── CameraParam.xml
├── main.cpp
├── README.md
├── SerialPort
│   ├── include
│   │   ├── CRC_Check.h
│   │   └── serialport.h
│   └── src
│       ├── CRC_Check.cpp
│       └── serialport.cpp
├── setup.sh
├── Thread
│   ├── thread.cpp
│   └── thread.h
├── ttyUSB.sh
└── WatchDDog.sh
```



## 4.通讯协议

| Byte0    | Byte1     | Byte2      | Byte3      | Byte4      | Byte5      | Byte6      | Byte7    | Byte8    | Byte9    |
| -------- | --------- | ---------- | ---------- | ---------- | ---------- | ---------- | -------- | -------- | -------- |
| 0xA5     | cmdID     | CRC8_Check | pitch_data | pitch_data | pitch_data | pitch_data | yaw_data | yaw_data | yaw_data |
| Byte10   | Byte11    | Byte12     | Byte13     | Byte14     | Byte15     | Byte16     | Byte17   | Byte18   | Byte19   |
| yaw_data | dist_data | dist_data  | dist_data  | dist_data  | flag1      | flag2      | flag3    | flag4    | flag5    |



- 0xA5 -帧头
- cmdID : 8 bit int - 命令模式（0 不处理，1 为红色自瞄，2 为蓝色自瞄， 3 4 5 6 7 8 为大小符）
- pitch_data : 32 bit float - 接收视觉解算出来的云台 pitch 值
- yaw_data : 32 bit float - 接收视觉解算出来的云台 yaw 值
- dist_data : 32 bit float - 接收视觉解算目标到相机的距离值
- flag1 : 8 bit int - 是否瞄准到中心（大小符用） / 哨兵模式 （stm32 -> PC）
- flag2 : 8 bit int - 是否找到目标（自瞄）/ 吊基地模式（stm32 -> PC）
- flag3 : 8 bit int - 是否识别到大小符
- flag4 : 8 bit int - 是否击打过大小符
- flag5 : 8 bit int - 装甲板是否贴脸（已弃用）

## 5.自启动

#### WatchDog.sh自启动脚本

```
#!/bin/bash 

sec=2 
cnt=0 
name=auto_aim_HDUS
program_name=HDUS
cd /home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/
#make clean && 
make -j12

/home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/HDUS
while [ 1 ] 
do 
    count=`ps -ef | grep $program_name | grep -v "grep" | wc -l`
    echo "Thread count: $count" 
    echo "Expection count: $cnt" 
    if [ $count -ge 1 ]; then 
        echo "The $name is still alive!" 
        sleep $sec 
    else  
        echo "Starting $name..." 
        gnome-terminal -- bash -c "cd /home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/;
        ./$program_name;exec bash;" 
        echo "$name has started!"   
        ((cnt=cnt+1)) 
        sleep $sec 
        if [ $cnt -gt 9 ]; then 
            echo "Reboot!" 
            #reboot 
        fi 
    fi 
done

```

#### 如何添加自启动脚本(或者将 `watchDog.sh` 添加到 ubuntu 系统的 `StartUp Applications` 中就可以实现开机自启动程序)：

1. **完善rc-local.service服务**

```shel
vi /lib/systemd/system/rc-local.service 

在结尾添加：
[Install]
WantedBy=multi-user.target  
Alias=rc-local.service

```

2. **添加/etc/rc.local文件**：

   ```shell
   sudo vim /etc/rc.local
       
   添加自己的脚本：
   sh you_dir/auto_start.sh
       
   ```

3. 给权限 

   ```
   sudo chmod +x /etc/rc.local
   ```

   

## 6.总结与未来展望
- 完善反小陀螺
- 完善预测装甲板(在敌方旋转与平移实现精确打击)
- EKF(扩展卡尔曼滤波)的预测使用
