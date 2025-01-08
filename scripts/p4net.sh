#!/bin/bash

# 检查是否是 root 用户
if [ "$EUID" -ne 0 ]; then
    echo "错误: 请使用 root 用户或通过 sudo 执行此脚本。"
    exit 1
fi

# 获取当前脚本的目录路径
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# 定义 set 和 reset 脚本的路径
SET_SCRIPT="$SCRIPT_DIR/_p4net_set.sh"
RESET_SCRIPT="$SCRIPT_DIR/_p4net_reset.sh"
BASH_SCRIPT="$SCRIPT_DIR/_p4net_bash.sh"

# 检查参数
if [ "$#" -ne 1 ]; then
    echo "用法: $0 --set | --reset | --bash"
    exit 1
fi

# 根据参数调用相应脚本
case "$1" in
    --set)
        if [ -f "$SET_SCRIPT" ]; then
            echo "调用 $SET_SCRIPT..."
            bash "$SET_SCRIPT"
        else
            echo "错误: $SET_SCRIPT 文件不存在！"
            exit 1
        fi
        ;;
    --reset)
        if [ -f "$RESET_SCRIPT" ]; then
            echo "调用 $RESET_SCRIPT..."
            bash "$RESET_SCRIPT"
        else
            echo "错误: $RESET_SCRIPT 文件不存在！"
            exit 1
        fi
        ;;
    --bash)
        if [ -f "$BASH_SCRIPT" ]; then
            echo "调用 $BASH_SCRIPT..."
            bash "$BASH_SCRIPT"
        else
            echo "错误: $BASH_SCRIPT 文件不存在！"
            exit 1
        fi
        ;;
    *)
        echo "无效参数: $1"
        echo "用法: $0 --set | --reset"
        exit 1
        ;;
esac
