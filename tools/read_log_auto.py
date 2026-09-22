"""
自动找出板子的串口并显示日志
============================

不用手动猜 COM 几。这个脚本会把所有非蓝牙的串口都试一遍，
哪个有数据就用哪个，然后持续显示。

用法：
    python read_log_auto.py

看到日志后按 Ctrl+C 停止。把输出全部贴给我即可。
"""

import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("缺少 pyserial。请运行：")
    print("  pip install pyserial -i https://pypi.org/simple")
    sys.exit(1)


SKIP_DESC = ("蓝牙", "bluetooth", "bthenum")


def candidates():
    """所有可能是板子的串口（排除电脑蓝牙自带的口）。"""
    out = []
    for p in list_ports.comports():
        d = (p.description or "").lower()
        if any(k in d for k in SKIP_DESC):
            continue
        out.append((p.device, p.description or ""))
    return out


def probe(device, seconds=4):
    """试读一个串口，返回读到的文本（可能为空）。"""
    try:
        ser = serial.Serial(device, 115200, timeout=0.5)
    except Exception as exc:
        return None, "打不开: %s" % exc
    lines = []
    t0 = time.time()
    try:
        while time.time() - t0 < seconds:
            raw = ser.readline()
            if raw:
                lines.append(raw.decode("utf-8", "replace").rstrip("\r\n"))
    except Exception as exc:
        return None, "读取出错: %s" % exc
    finally:
        ser.close()
    return lines, None


def main():
    ports = candidates()
    print("=" * 62)
    print("检查候选串口（已排除电脑蓝牙自带的口）：")
    for dev, desc in ports:
        print("   %-8s %s" % (dev, desc))
    if not ports:
        print()
        print("没有候选串口。检查板子有没有插好、有没有在跑固件。")
        return 1
    print("=" * 62)
    print()

    winner = None
    for dev, desc in ports:
        print("--- 试 %s (%s) 4 秒…" % (dev, desc))
        lines, err = probe(dev, 4)
        if err:
            print("    %s" % err)
            continue
        if lines:
            print("    >>> 有输出！这就是板子 <<<")
            for l in lines:
                print("    " + l)
            winner = dev
            break
        print("    没有输出")

    if winner is None:
        print()
        print("=" * 62)
        print("所有串口都没有输出。可能原因：")
        print("  1. 新固件还没烧上（还在引导器里，或还是旧固件）")
        print("  2. 板子没插好 / 线是充电线不能传数据")
        print("  3. 固件跑起来但崩了（日志还没来得及输出）")
        print()
        print("建议：重新双击 RST 进引导器，把 zephyr.uf2 拖进去，")
        print("      然后拔插一次 USB，再运行本脚本。")
        return 1

    print()
    print("=" * 62)
    print("持续显示 %s 的输出（Ctrl+C 停止）" % winner)
    print("现在请踩几脚曲柄唤醒功率计，观察 20 秒…")
    print("-" * 62)
    try:
        ser = serial.Serial(winner, 115200, timeout=1)
        while True:
            raw = ser.readline()
            if raw:
                text = raw.decode("utf-8", "replace").rstrip("\r\n")
                if text.strip():
                    print(text)
    except KeyboardInterrupt:
        print()
        print("-" * 62)
        print("停止")
    except Exception as exc:
        print("出错: %s" % exc)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
