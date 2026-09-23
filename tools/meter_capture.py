#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功率计私有协议抓包脚本
======================

绕开桥，直接连功率计抓它的原始通知，用来核对协议字段。

注意：厂商自定义特征值（形如 a6ed0202…）里是状态/调试帧，不是功率数据，
不要抓那个。真正的功率数据在标准测量特征 0x2A63 里：

  1. 连上功率计
  2. 往 0x2A55（Cycling Power Control Point）写启动命令 02 16 AA 10
     —— 实测本型号会回 0x81 拒绝，且不发也照样有测量数据，所以这步可跳过
  3. 订阅 0x2A63（Cycling Power Measurement）拿真正的功率数据
  4. 按 meter_protocol.h 的偏移实时解析并打印

用法：
    pip install bleak

    python meter_capture.py                    # 自动扫描并连接
    python meter_capture.py C0:A1:25:04:32:08  # 指定地址（更快）
    python meter_capture.py --seconds 60       # 监听时长（默认 30 秒）

注意：
  - 功率计通常只允许一个中央设备连接。跑之前请关掉手机 App / 码表的蓝牙连接。
  - 踩一下曲柄唤醒功率计，否则它不广播。
  - 这台功率计的服务 UUID 是厂商私有的 0x1828（按标准它其实是 Mesh Proxy），
    测量特征 0x2A63，控制点 0x2A55。
"""

import argparse
import asyncio
import struct
import sys
import time

try:
    from bleak import BleakClient, BleakScanner
    HAVE_BLEAK = True
except ImportError:
    BleakClient = None
    BleakScanner = None
    HAVE_BLEAK = False

# 标准 Bluetooth SIG UUID（16 位短 UUID 展开成 128 位基准形式）
BASE = "0000{}-0000-1000-8000-00805f9b34fb"
UUID_CP_SERVICE = BASE.format("1828")   # 这台功率计实际用的服务（私有）
UUID_BRIDGE_SERVICE = BASE.format("1818")  # 我们自己的桥对外广播这个，扫描时要排除
UUID_CP_MEAS = BASE.format("2a63")      # Cycling Power Measurement  ← 功率数据
UUID_CP_CTRL = BASE.format("2a55")      # Cycling Power Control Point ← 启动命令

# 厂商私有启动命令。实测本型号返回 0x81 拒绝，跳过也不影响收数据，
# 保留它是为了换型号时还能试一下。
METER_START_COMMAND = bytes([0x02, 0x16, 0xAA, 0x10])

DEFAULT_ADDRESS = "C0:A1:25:04:32:08"


# ---------------------------------------------------------------------------
# 解析：与 src/meter_protocol.c 的偏移一致
# ---------------------------------------------------------------------------

def parse_meter_payload(payload):
    """按 src/meter_protocol.h 解析 11 字节负载。

    返回 dict；长度不足的字段返回 None（对应 C 代码里的 length>=N 判断）。
    """
    n = len(payload)
    out = {
        "len": n,
        "total_power_w": None,
        "left_power_w": None,
        "right_power_w": None,
        "cadence_rpm": None,
        "angle_deg": None,
        "error_code": None,
    }
    if n < 2:
        return out

    def le_u16(off):
        return payload[off] | (payload[off + 1] << 8)

    def le_s16(off):
        v = le_u16(off)
        return v - 65536 if v >= 32768 else v

    out["total_power_w"] = le_u16(0)
    if n >= 4:
        out["left_power_w"] = le_s16(2)
    if n >= 6:
        out["right_power_w"] = le_s16(4)
    if n >= 8:
        cad = le_s16(6)
        # C 代码：cadence > 0 才采用，否则归零（反踩时为负）
        out["cadence_rpm"] = cad if cad > 0 else 0
    if n >= 10:
        out["angle_deg"] = le_u16(8)
    if n >= 11:
        out["error_code"] = payload[10]
    return out


def parse_standard_cp(payload):
    """顺带按标准 Cycling Power Measurement 格式解一遍，用于对照。

    标准格式：flags(2) + instantaneous_power(2, s16) + [可选字段...]
    """
    if len(payload) < 4:
        return None
    flags = payload[0] | (payload[1] << 8)
    power = struct.unpack_from("<h", payload, 2)[0]
    return {"flags": "0x%04X" % flags, "power_w": power}


# ---------------------------------------------------------------------------
# 抓包
# ---------------------------------------------------------------------------

class Capture:
    def __init__(self, seconds):
        self.seconds = seconds
        self.count = 0
        self.first_at = None
        self.last_at = None

    def on_notify(self, char_uuid, data):
        self.count += 1
        now = time.time()
        if self.first_at is None:
            self.first_at = now
        self.last_at = now

        r = parse_meter_payload(data)
        std = parse_standard_cp(data)

        head = "[%3d]" % self.count
        raw = data.hex().upper()

        # 解析结果（按 src/meter_protocol.h 的偏移）
        parts = []
        if r["total_power_w"] is not None:
            parts.append("总功率=%s W" % r["total_power_w"])
        if r["left_power_w"] is not None:
            parts.append("左=%s" % r["left_power_w"])
        if r["right_power_w"] is not None:
            parts.append("右=%s" % r["right_power_w"])
        if r["cadence_rpm"] is not None:
            parts.append("踏频=%s rpm" % r["cadence_rpm"])
        if r["angle_deg"] is not None:
            parts.append("角度=%s" % r["angle_deg"])
        if r["error_code"] is not None:
            parts.append("err=%s" % r["error_code"])

        print("%s %-*s  %s" % (head, 26, raw[:26], "  ".join(parts)))
        if len(raw) > 26:
            print("       %s" % raw[26:])
        print("       长度=%d  标准CP视角: %s" % (r["len"], std))

    def summary(self):
        print()
        print("=" * 64)
        print("共收到 %d 条通知" % self.count)
        if self.count == 0:
            print("一条都没收到。可能原因：")
            print("  1. 没发启动命令，或控制点写入失败（看上面的警告）")
            print("  2. 功率计被别的设备占着（手机 App / 码表）")
            print("  3. 没踩踏，设备处于休眠")
            return
        span = (self.last_at or 0) - (self.first_at or 0)
        if span > 0:
            print("时间跨度 %.1f 秒，平均 %.1f 条/秒" % (span, self.count / span))


async def find_device(address):
    if address:
        print("按地址查找 %s …" % address)
        dev = await BleakScanner.find_device_by_address(address, timeout=15)
        if dev:
            return dev
        print("按地址没找到，改为全量扫描 …")

    print("扫描附近的 BLE 设备（10 秒）…")
    devices = await BleakScanner.discover(timeout=10, return_adv=True)
    cands = []
    for dev, adv in devices.values():
        name = (adv.local_name or dev.name or "")
        uuids = [u.lower() for u in (adv.service_uuids or [])]
        low = name.lower()
        # 认 0x1828 服务；有些型号广播里不带服务 UUID，退回按名字认。
        # 排除我们自己的桥（广播 0x1818、名字含 bridge）。
        hit = (UUID_CP_SERVICE in uuids) or ("xds-" in low)
        if (UUID_BRIDGE_SERVICE in uuids) or ("bridge" in low):
            hit = False
        if hit:
            cands.append((dev, name, uuids))
    if not cands:
        print("没找到带 0x1828 服务的设备。附近设备列表：")
        for dev, adv in devices.values():
            print("   %-20s %s" % (adv.local_name or dev.name or "(无名)", dev.address))
        return None
    print("找到候选设备 %d 个：" % len(cands))
    for dev, name, uuids in cands:
        print("   %-24s %s" % (name or "(无名)", dev.address))
    return cands[0][0]


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("address", nargs="?", default=None,
                    help="功率计 BLE 地址；不填则自动扫描")
    ap.add_argument("--seconds", type=int, default=30, help="监听时长，默认 30 秒")
    ap.add_argument("--no-start", action="store_true",
                    help="不发启动命令（用于对比测试）")
    args = ap.parse_args()

    addr = args.address or DEFAULT_ADDRESS
    dev = await find_device(addr)
    if dev is None:
        print()
        print("没找到功率计。踩一下曲柄唤醒它再试，或用 --help 看用法。")
        return

    print("连接 %s …" % dev.address)
    cap = Capture(args.seconds)

    async with BleakClient(dev, timeout=20.0) as client:
        print("已连接")

        # 1) 先发启动命令（实测本型号会拒绝，但不影响后续订阅）
        if not args.no_start:
            try:
                await client.write_gatt_char(UUID_CP_CTRL, METER_START_COMMAND,
                                             response=True)
                print("已发送启动命令 %s 到 0x2A55" % METER_START_COMMAND.hex().upper())
            except Exception as exc:
                print("!! 写 0x2A55 失败：%s" % exc)
                print("   （有些功率计不需要这条命令，继续尝试订阅）")
        else:
            print("按参数要求跳过启动命令")

        # 2) 订阅标准功率测量
        def cb(sender, data):
            cap.on_notify(str(sender), bytes(data))

        print("订阅 0x2A63（Cycling Power Measurement）…")
        try:
            await client.start_notify(UUID_CP_MEAS, cb)
            print("订阅成功，开始监听 %d 秒。现在请踩踏曲柄。" % args.seconds)
        except Exception as exc:
            print("!! 订阅 0x2A63 失败：%s" % exc)
            print("   列出该设备实际提供的服务/特征，供排查：")
            for svc in client.services:
                print("   服务 %s" % svc.uuid)
                for ch in svc.characteristics:
                    print("      特征 %s  %s" % (ch.uuid, ",".join(ch.properties)))
            return

        t0 = time.time()
        while time.time() - t0 < args.seconds:
            await asyncio.sleep(0.25)
        try:
            await client.stop_notify(UUID_CP_MEAS)
        except Exception:
            pass

    cap.summary()


if __name__ == "__main__":
    if not HAVE_BLEAK:
        print("缺少 bleak。请先运行：  pip install bleak")
        sys.exit(1)
    if BleakClient is None:
        print("bleak 未就绪")
        sys.exit(1)
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n手动中断")
