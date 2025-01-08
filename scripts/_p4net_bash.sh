#!/bin/bash

# 定义固定的网络命名空间和用户名
NETNS_NAME="p4_ns"
USERNAME="nvidia"

# 检查网络命名空间是否存在
if ! ip netns list | grep -q "^${NETNS_NAME}\b"; then
    echo "错误: 网络命名空间 '${NETNS_NAME}' 不存在。"
    exit 1
fi

# 检查用户是否存在
if ! id "$USERNAME" &>/dev/null; then
    echo "错误: 用户 '${USERNAME}' 不存在。"
    exit 1
fi

# 在网络命名空间中执行 bash，切换到 nvidia
echo "切换到网络命名空间 '${NETNS_NAME}'"
sudo ip netns exec "$NETNS_NAME" bash -c "
    su $USERNAME
"

