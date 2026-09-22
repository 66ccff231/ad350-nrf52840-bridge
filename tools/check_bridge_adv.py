"""扫描并打印本机板子广播的完整内容，确认服务 UUID 是否已改成 0x1818。"""
import asyncio
import sys

from bleak import BleakScanner

TARGET = "XDS Power Bridge"

KNOWN = {
    "00001800-0000-1000-8000-00805f9b34fb": "Generic Access",
    "00001801-0000-1000-8000-00805f9b34fb": "Generic Attribute",
    "0000180a-0000-1000-8000-00805f9b34fb": "Device Information",
    "0000180f-0000-1000-8000-00805f9b34fb": "Battery Service",
    "00001818-0000-1000-8000-00805f9b34fb": "★ Cycling Power（标准功率服务）",
    "00001816-0000-1000-8000-00805f9b34fb": "Cycling Speed and Cadence",
    "00001814-0000-1000-8000-00805f9b34fb": "Running Speed and Cadence",
    "0000180d-0000-1000-8000-00805f9b34fb": "Heart Rate",
    "00001826-0000-1000-8000-00805f9b34fb": "Fitness Machine",
    "00001828-0000-1000-8000-00805f9b34fb": "⚠ Mesh Proxy（不是功率服务！）",
}


async def main():
    print("扫描 12 秒…")
    found = await BleakScanner.discover(timeout=12, return_adv=True)

    hit = None
    for dev, adv in found.values():
        nm = adv.local_name or dev.name or ""
        if TARGET.lower() in nm.lower():
            hit = (dev, adv)
            break

    if not hit:
        print("没找到 %s" % TARGET)
        print("附近设备：")
        for dev, adv in found.values():
            print("   %-26s %s" % (adv.local_name or dev.name or "(无名)", dev.address))
        return 1

    dev, adv = hit
    print()
    print("=" * 62)
    print("找到板子: %s" % (adv.local_name or dev.name))
    print("地址: %s   信号: %s dBm" % (dev.address, getattr(adv, "rssi", "?")))
    print()
    print("广播的服务 UUID：")
    uuids = [u.lower() for u in (adv.service_uuids or [])]
    if not uuids:
        print("   （广播里没有服务 UUID！）")
    for u in uuids:
        print("   %s   %s" % (u, KNOWN.get(u, "(未知)")))
    print()
    print("制造商数据:", adv.manufacturer_data)
    print()
    print("=" * 62)
    has_cp = "00001818-0000-1000-8000-00805f9b34fb" in uuids
    has_mesh = "00001828-0000-1000-8000-00805f9b34fb" in uuids
    if has_cp and not has_mesh:
        print("✅ 广播的是标准功率服务 0x1818 —— 新固件已生效")
        print("   如果行者 app 还是搜不到，问题就在 app 侧的过滤条件或配对要求")
    elif has_mesh:
        print("❌ 还是 0x1828（Mesh Proxy）—— 新固件没烧上，请重新烧录")
    else:
        print("⚠ 既没有 0x1818 也没有 0x1828，广播内容异常")
    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
