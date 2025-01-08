#!/bin/bash

# 检测脚本是否通过 source 执行
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "错误：请通过 'source' 命令执行此脚本，例如："
  echo "source $0"
  return 1
fi

# 设置构建目录
BUILD_DIR=build

# 检测是否传入 --visual 参数
USE_VISUAL=false
for arg in "$@"; do
  if [ "$arg" == "--visual" ]; then
    USE_VISUAL=true
    break
  fi
done

if $USE_VISUAL; then

  # 检测 Python 版本是否为 3.11
  PYTHON_PATH=$(which python)
  if [ -z "$PYTHON_PATH" ]; then
    echo "错误：未找到 Python。请检查 Python 是否已安装并在 PATH 中。"
    return 1
  fi

  PYTHON_VERSION=$($PYTHON_PATH --version 2>&1)
  if [[ "$PYTHON_VERSION" != *"3.11"* ]]; then
    echo "错误：需要 Python 3.11，当前版本为 $PYTHON_VERSION。"
    return 1
  fi

  # 检查是否安装了必要的 Python 包：matplotlib、numpy 和 pybind11
  REQUIRED_PACKAGES=("matplotlib" "numpy" "pybind11")
  for package in "${REQUIRED_PACKAGES[@]}"; do
    if ! $PYTHON_PATH -c "import $package" 2>/dev/null; then
      echo "错误：Python 包 '$package' 未安装，请先安装。"
      return 1
    fi
  done

  PYBIND11_DIR=$($PYTHON_PATH -m pybind11 --cmakedir)
  if [ $? -ne 0 ]; then
    echo "错误：无法获取 pybind11 的 CMake 路径。"
    return 1
  fi
fi

# 如果构建目录存在，则删除
if [ -d "$BUILD_DIR" ]; then
  rm -rf $BUILD_DIR
fi

# 执行 CMake 配置
if $USE_VISUAL; then
  cmake -B ./$BUILD_DIR -G Ninja -D CMAKE_EXPORT_COMPILE_COMMANDS=1 \
    -D Python3_EXECUTABLE="$PYTHON_PATH" \
    -D Python3_ROOT_DIR="$(dirname $PYTHON_PATH)" \
    -D pybind11_DIR="$PYBIND11_DIR" \
    -D USE_VISUAL=ON
else
  cmake -B ./$BUILD_DIR -G Ninja -D CMAKE_EXPORT_COMPILE_COMMANDS=1
fi
