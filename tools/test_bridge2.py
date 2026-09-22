"""更稳的连接测试：多轮扫描 + 更长超时 + 详细报错。"""
import asyncio
import struct
import sys
import time

from bleak import BleakClient, BleakScanner

BASE = "0000{}-0000-1000-8000-00805f9b34fb"
UUID_CPM = BASE.format("2a63")
TARGET = "XDS Power Bridge"


def parse(payload):
    if len(payload) < 4:
        return None
    flags = payload[0] | (payload[1] << 8)
    power = struct.unpack_from("<h", payload, 2)[0]
    out = {"flags": flags, "power": power, "len": len(payload)}
    off = 4
    if flags & 0x0020 and len(payload) >= off + 4:
        out["revs"] = payload[off] | (payload[off + 1] << 8)
        out["evt"] = payload[off + 2] | (payload[off + 3] << 8)
    return out


async def find():
    for attempt in range(4):
        print("  扫描第 %d 轮…" % (attempt + 1))
        devices = await BleakScanner.discover(timeout=8, return_adv=True)
        best = None
        for d, adv in devices.values():
            nm = adv.local_name or d.name or ""
            if TARGET.lower() in nm.lower():
                best = (d, adv)
                break
        if best:
            return best
    return None


async def main():
    print("=== 1. 找设备 ===")
    hit = await find()
    if not hit:
        print("没找到 %s" % TARGET)
        return 1
    dev, adv = hit
    print("找到: %s  地址 %s  信号 %s dBm" % (adv.local_name or dev.name,
                                              dev.address, getattr(adv, "rssi", "?")))
    print("广播服务: %s" % (adv.service_uuids or []))

    print()
    print("=== 2. 连接（超时 45 秒）===")
    for attempt in range(3):
        try:
            print("  第 %d 次尝试…" % (attempt + 1))
            client = BleakClient(dev, timeout=45)
            await client.connect()
            print("  连接成功，is_connected =", client.is_connected)
            break
        except Exception as exc:
            print("  失败: %s: %s" % (type(exc).__name__, str(exc)[:150]))
            await asyncio.sleep(2)
    else:
        print()
        print("三次都连不上。")
        print("可能原因：")
        print("  1. 固件的 BLE 栈卡住了（例如主循环里反复启动扫描失败）")
        print("  2. 广播在但不可连接（BT_LE_ADV_OPT_CONN 没生效）")
        print("  3. Windows 蓝牙栈问题（试试拔插 USB 换个口）")
        return 1

    try:
        print()
        print("=== 3. 读服务 ===")
        for svc in client.services:
            print("  服务 %s" % svc.uuid)
            for ch in svc.characteristics:
                print("     特征 %s  %s" % (ch.uuid, ",".join(ch.properties)))

        print()
        print("=== 4. 订阅并收数据（20 秒，请转动曲柄）===")
        count = 0

        def cb(_s, data):
            nonlocal count
            count += 1
            r = parse(bytes(data))
            if r:
                extra = ""
                if "revs" in r:
                    extra = " 曲柄圈数=%d 事件时间=%d" % (r["revs"], r["evt"])
                print("[%3d] %s 功率=%d W flags=0x%04X%s"
                      % (count, bytes(data).hex().upper(), r["power"], r["flags"], extra))
            else:
                print("[%3d] %s (太短)" % (count, bytes(data).hex().upper()))

        await client.start_notify(UUID_CPM, cb)
        print("  订阅成功")
        t0 = time.time()
        while time.time() - t0 < 20:
            await asyncio.sleep(0.3)
        await client.stop_notify(UUID_CPM)
        print()
        print("收到 %d 条通知" % count)
    finally:
        await client.disconnect()
        print("已断开")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
