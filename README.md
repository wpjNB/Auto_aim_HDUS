<<<<<<< HEAD
# Auto_aim_HDUS
=======
# 花椒步兵视觉开源方案 ReelSteel

------

华东交通大学2024赛季3V3对抗赛视觉开源框架

#### 本开源参考：沈阳航空航天TUP2022，深大2019，上交2021,湖大2023开源

**新手导向**：

- RM开源汇总：https://bbs.robomaster.com/forum.php?mod=forumdisplay&fid=63&filter=typeid&typeid=167
- RMhttps://docs.qq.com/sheet/DUFlaU0FHZk1QS0l1?tab=bb08j2

- 上海交龙战队博客：https://sjtu-robomaster-team.github.io/
- 君佬rm_vision视觉开源：https://github.com/rm-vision-archive
- 北极熊战队：https://flowus.cn/lihanchen/share/facb28a9-5d34-42a7-9bc8-630a182c3571
- 湖南跃鹿视觉教程：https://blog.csdn.net/NeoZng/article/details/126283713?spm=1001.2014.3001.5502
- Robomaster——关于视觉组，你想要了解的都在这里:https://blog.csdn.net/weixin_42754478/article/details/108159529#comments_29363840

## 1. 简介

1. ##### **硬件选型**：

- **工业相机**：海康威视MV-CS016-10UC +**搭配镜头**：8mm焦距镜头
- 普通usb摄像头

2. **功能：**

- 自瞄

3. **未完成**：

- 能量机关
- 预测，反小陀螺

4. **部署需要调节参数**：

   - 相机的曝光与增益,参数

   - 相机内参矩阵与外参矩阵XML路径更改
   
   - 相机坐标系到枪管坐标系水平(X,Y,Z)偏移量与旋转(Pitch,Yaw)偏移量
   
   - 传统视觉识别参数，如二值化阈值,灯条装甲板几何约束参数
   
   - USB虚拟串口设备名字
   
   - inference model 路径更改
   
     

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
- 

## 3.文件结构



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

   

## 6.总结与展望
