# XDS 功率计 BLE 桥接固件（nRF52840 / Zephyr）
#
# 把 XDS 功率计的测量数据转成标准 BLE Cycling Power 服务广播出去，
# 让行者 app / 行者小G+ 第三代当作普通蓝牙功率计来连接。
#
# 为什么不用 ANT+：G3+ 官方规格支持「ANT+/蓝牙双连接，功率计均可连」，
# 行者也说明第三方传感器可搜到即连。走 BLE 就完全绕开 ANT 协议栈与 Garmin 许可。

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xds_ble_bridge)

target_sources(app PRIVATE
    src/main.c
    src/xds_protocol.c
)
