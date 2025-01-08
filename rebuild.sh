#!/bin/bash

# 设置构建目录
BUILD_DIR=build

# 如果构建目录存在，则删除
if [ -d "$BUILD_DIR" ]; then
  rm -rf $BUILD_DIR
fi

cmake -B ./build -G Ninja -D CMAKE_EXPORT_COMPILE_COMMANDS=1 \
  -D pybind11_DIR="/home/charleshsu/anaconda3/envs/P4Arm/lib/python3.11/site-packages/pybind11/share/cmake/pybind11"