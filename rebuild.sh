#!/bin/bash

# 设置构建目录
BUILD_DIR=build

# 如果构建目录存在，则删除
if [ -d "$BUILD_DIR" ]; then
  rm -rf $BUILD_DIR
fi

cmake -B ./build -G Ninja -D CMAKE_EXPORT_COMPILE_COMMANDS=1