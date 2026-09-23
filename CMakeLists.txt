# 功率计 BLE 桥接固件（nRF52840 / Zephyr）
#
# 把功率计的私有测量数据转成标准 BLE Cycling Power 服务（0x1818）广播出去，
# 让手机 app / 码表当作普通蓝牙功率计来连接。
#
# 为什么不用 ANT+：本板没有 SWD 编程器，而 ANT SoftDevice 会覆盖 UF2 引导器；
# 且 ANT+ 需 Garmin 授权、SoftDevice 许可也限制用途。走 BLE 完全绕开这些。

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ble_power_bridge)

target_sources(app PRIVATE
    src/main.c
    src/meter_protocol.c
)
