#!/bin/bash

# 定义命名空间和虚拟接口
NAMESPACE="p4_ns"
VETH_HOST="veth-host"
VETH_NS="veth-ns"

# 检查并迁移虚拟接口回主命名空间
if ip netns list | grep -qw "$NAMESPACE"; then
    if ip netns exec "$NAMESPACE" ip link show | grep -qw "$VETH_NS"; then
        ip netns exec "$NAMESPACE" ip link set "$VETH_NS" netns 1
        echo "虚拟接口 $VETH_NS 已迁移回主命名空间。"
    fi
    # 删除命名空间
    ip netns del "$NAMESPACE"
    echo "命名空间 $NAMESPACE 已删除。"
else
    echo "命名空间 $NAMESPACE 不存在，跳过删除。"
fi

# 清理虚拟接口
reset_interface() {
    local iface=$1
    if ip link show | grep -qw "$iface"; then
        ip link set "$iface" down
        ip addr flush dev "$iface"
        ip link delete "$iface"
        echo "虚拟接口 $iface 已删除。"
    else
        echo "虚拟接口 $iface 不存在，跳过删除。"
    fi
}

# 删除主命名空间的虚拟接口
reset_interface "$VETH_HOST"
reset_interface "$VETH_NS"

# 重置物理接口（如有必要）
reset_physical_interface() {
    local iface=$1
    if ip link show | grep -qw "$iface"; then
        ip link set "$iface" down
        ip addr flush dev "$iface"
        ip link set "$iface" up
        echo "物理接口 $iface 已重置。"
    else
        echo "物理接口 $iface 不存在，跳过重置。"
    fi
}

# 示例：如果需要重置的物理接口
reset_physical_interface "eth0"
reset_physical_interface "eth1"
reset_physical_interface "eth2"
reset_physical_interface "eth3"

echo "重置完成。"
