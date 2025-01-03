#!/bin/bash

# 创建命名空间
NAMESPACE="p4_ns"
if ! ip netns list | grep -qw "$NAMESPACE"; then
    ip netns add "$NAMESPACE"
    echo "命名空间 $NAMESPACE 创建成功。"
else
    echo "命名空间 $NAMESPACE 已经存在，跳过创建。"
fi

# 创建虚拟接口对并连接默认命名空间与命名空间 $NAMESPACE
VETH_HOST="veth-host"
VETH_NS="veth-ns"

if ip link show | grep -qw "$VETH_HOST"; then
    echo "虚拟接口对 $VETH_HOST 和 $VETH_NS 已经存在，跳过创建。"
else
    ip link add "$VETH_HOST" type veth peer name "$VETH_NS"
    ip link set "$VETH_NS" netns "$NAMESPACE"
    echo "虚拟接口对 $VETH_HOST 和 $VETH_NS 创建成功。"
fi

# 配置虚拟接口
HOST_IP="10.0.0.1"
NS_IP="10.0.0.2"
SUBNET="24"

# 配置主机侧虚拟接口
ip addr add "$HOST_IP/$SUBNET" dev "$VETH_HOST" 2>/dev/null || true
ip link set "$VETH_HOST" up
echo "虚拟接口 $VETH_HOST 配置完成，IP：$HOST_IP/$SUBNET。"

# 配置命名空间侧虚拟接口
ip netns exec "$NAMESPACE" ip addr add "$NS_IP/$SUBNET" dev "$VETH_NS" 2>/dev/null || true
ip netns exec "$NAMESPACE" ip link set "$VETH_NS" up
ip netns exec "$NAMESPACE" ip route add default via "$HOST_IP" dev "$VETH_NS" 2>/dev/null || true
echo "虚拟接口 $VETH_NS 配置完成，IP：$NS_IP/$SUBNET，默认路由通过 $HOST_IP。"

# 配置网络接口的函数
configure_interface() {
    local iface=$1
    local ip_addr=$2
    local subnet=$3
    local gateway=$4
    local metric=$5

    if ip netns exec "$NAMESPACE" ip link show | grep -qw "$iface"; then
        echo "网络接口 $iface 已经在命名空间 $NAMESPACE 中，跳过配置。"
    else
        ip link set "$iface" netns "$NAMESPACE"
        ip netns exec "$NAMESPACE" ip link set "$iface" up
        ip netns exec "$NAMESPACE" ip addr add "$ip_addr/$subnet" dev "$iface"
        ip netns exec "$NAMESPACE" ip route add default via "$gateway" dev "$iface" proto static metric "$metric"
        ip netns exec "$NAMESPACE" ip route add "${ip_addr%.*}.0/$subnet" dev "$iface" proto kernel scope link src "$ip_addr" metric "$((metric - 20000))"
        echo "网络接口 $iface 已成功配置到命名空间 $NAMESPACE。"
    fi
}

# 配置接口
configure_interface "eth0" "192.168.1.100" "24" "192.168.1.1" "20100"
configure_interface "eth1" "192.168.2.100" "24" "192.168.2.1" "20101"
configure_interface "eth2" "192.168.12.100" "24" "192.168.12.1" "20102"
configure_interface "eth3" "192.168.11.100" "24" "192.168.11.1" "20103"

# 测试连通性
echo "测试从主命名空间 Ping 命名空间内的接口 $NS_IP..."
if ping -c 3 "$NS_IP" &>/dev/null; then
    echo "主命名空间到 $NS_IP 的 Ping 测试成功！"
else
    echo "主命名空间到 $NS_IP 的 Ping 测试失败！"
fi

echo "测试从命名空间 Ping 主命名空间的接口 $HOST_IP..."
if ip netns exec "$NAMESPACE" ping -c 3 "$HOST_IP" &>/dev/null; then
    echo "命名空间到 $HOST_IP 的 Ping 测试成功！"
else
    echo "命名空间到 $HOST_IP 的 Ping 测试失败！"
fi

echo "配置和测试完成。"
