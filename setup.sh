#!/usr/bin/env bash
# usage: 编译项目

project_path=$(pwd)
# git submodule update --recursive --init
cd $project_path
mkdir -p build
cd build
cmake ..
make -j12

/home/wpj/RM_Vision_code_US/auto_aim_HDUS/build/HDUS
