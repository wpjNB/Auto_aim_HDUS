#!/bin/bash
#，默认串口为名为 /dev/ttyUSB0，因为串口松动可能会变成 /dev/ttyUSB1，因此我们对两个串口都赋予权限（有一个串口会不存在，不过不妨碍程序运行）
echo 5919|sudo -S sudo chmod +777 /dev/ttyACM0
echo 5919|sudo -S sudo chmod +777 /dev/ttyACM1
echo 5919|sudo -S sudo chmod +777 /dev/ttyUSB0
echo 5919|sudo -S sudo chmod +777 /dev/ttyUSB1