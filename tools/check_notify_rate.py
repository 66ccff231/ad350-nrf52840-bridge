#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
功率计上报频率测量 / 曲柄角度法可行性判定
==========================================

背景
----
标准 CPS 服务里没有「踏频」字段，码表是用「累计曲柄圈数 + 事件时间」反推的。
固件目前是**按踏频积分**估算圈数。理论上还有个更准的做法：
功率计在偏移 8-9 上报**曲柄角度**（0~359°），如果能从角度回绕**数出真实圈数**，
踏频就变成真实测量值，快速变速也不滞后。

可行性完全取决于**上报频率**：两次上报之间曲柄必须转过 **不到一整圈**，
否则角度回绕分不清是 1 圈还是 2 圈（混叠），圈数必然偏少。
即要求 **上报频率 > 踏频/60**，换算成安全上限：**最大不混叠踏频 = 60 × 频率(Hz)**。

实测结论（手转 + 捏刹车两种工况）
----------------------------------
    上报频率        1.00 Hz（中位间隔 1020 ms，与踏频无关；93 rpm 时仍是 1.00 Hz）
    每帧转过圈数    中位 1.19 圈，最大 1.70 圈
    超过一整圈的帧  44 / 56 = 79%
    角度法结果      数出 27.8 rpm，真实 70.2 rpm —— **低 60%**
    -> 角度法**不可行**，1 Hz 是硬件天花板，必须继续用积分法。

因此本脚本的长期用途是：**换功率计 / 怀疑上报频率时复测一次**，
直接看 `ALIASING CEILING` 那一行 —— 低于你实际骑行上限（约 120 rpm）就别走角度法。

用法
----
    python check_notify_rate.py --scan-only          # 只扫描，确认能认出功率计
    python check_notify_rate.py --seconds 90         # 抓 90 秒（边踩边抓）
    python check_notify_rate.py --seconds 90 --address C0:A1:25:04:32:08

注意事项
--------
  - 跑之前**断开桥接板子**：功率计同时只允许一个连接，板子占着就连不上。
  - 本脚本按 `0x1828` 服务认功率计；桥自己广播 `0x1818` 且名字叫
    「XDS Power Bridge」，所以名字里含 xds 但带 bridge 的一律排除，免得抓到自己人。
  - 输出 CSV：t, dt_ms, total_w, left_w, right_w, cadence, angle, err, len, raw_hex

需要：pip install bleak
"""

import argparse
import asyncio
import csv
import os
import statistics
import sys
import time

from bleak import BleakClient, BleakScanner

BASE = "0000{}-0000-1000-8000-00805f9b34fb"
UUID_XDS_SERVICE = BASE.format("1828")   # the METER advertises this
UUID_CP_SERVICE = BASE.format("1818")    # our own bridge advertises this
UUID_CP_MEAS = BASE.format("2a63")
UUID_CP_CTRL = BASE.format("2a55")
XDS_START_COMMAND = bytes([0x02, 0x16, 0xAA, 0x10])

DEFAULT_ADDRESS = "C0:A1:25:04:32:08"


def le_u16(b, off):
    return b[off] | (b[off + 1] << 8)


def le_s16(b, off):
    v = le_u16(b, off)
    return v - 65536 if v >= 32768 else v


def parse(payload):
    n = len(payload)
    r = {"len": n, "total": None, "left": None, "right": None,
         "cadence": None, "angle": None, "err": None}
    if n >= 2:
        r["total"] = le_u16(payload, 0)
    if n >= 4:
        r["left"] = le_s16(payload, 2)
    if n >= 6:
        r["right"] = le_s16(payload, 4)
    if n >= 8:
        c = le_s16(payload, 6)
        r["cadence"] = c if c > 0 else 0
    if n >= 10:
        r["angle"] = le_u16(payload, 8)
    if n >= 11:
        r["err"] = payload[10]
    return r


async def scan_devices(verbose=True):
    print("scanning 12s ...")
    found = await BleakScanner.discover(timeout=12.0, return_adv=True)
    items = []
    if isinstance(found, dict):
        items = list(found.values())
    else:
        items = [(d, a) for d, a in found]
    cands = []
    for dev, adv in items:
        name = (getattr(adv, "local_name", None) or dev.name or "")
        uuids = [str(u).lower() for u in (getattr(adv, "service_uuids", None) or [])]
        rssi = getattr(adv, "rssi", None)
        if verbose:
            print("  %-34s %-24s rssi=%s uuids=%s"
                  % (dev.address, name or "(none)", rssi, ",".join(uuids) or "-"))
        # The meter is the device advertising 0x1828. Our own bridge advertises
        # 0x1818 and is literally named "XDS Power Bridge", so a name match on
        # "xds" alone would pick up the bridge instead of the meter.
        hit = UUID_XDS_SERVICE in uuids
        if not hit and "xds" in name.lower() and "bridge" not in name.lower():
            hit = True
        if hit:
            cands.append(dev)
    return cands


class Cap:
    def __init__(self):
        self.rows = []
        self.t0 = None

    def on_notify(self, _sender, data):
        now = time.perf_counter()
        if self.t0 is None:
            self.t0 = now
        b = bytes(data)
        r = parse(b)
        self.rows.append({
            "t": now - self.t0,
            "total": r["total"], "left": r["left"], "right": r["right"],
            "cadence": r["cadence"], "angle": r["angle"], "err": r["err"],
            "len": r["len"], "raw": b.hex().upper(),
        })
        n = len(self.rows)
        if n <= 3 or n % 10 == 0:
            print("  [%4d] t=%7.3fs power=%s cad=%s angle=%s"
                  % (n, self.rows[-1]["t"], r["total"], r["cadence"], r["angle"]))


def analyze(cap, csv_path):
    rows = cap.rows
    print()
    print("=" * 70)
    print("notifications: %d" % len(rows))
    if len(rows) < 3:
        print("too few samples.")
        return

    ts = [r["t"] for r in rows]
    ang = [r["angle"] for r in rows]
    cad = [r["cadence"] for r in rows]
    span = ts[-1] - ts[0]
    dts = [(ts[i + 1] - ts[i]) * 1000.0 for i in range(len(ts) - 1)]
    print("span: %.2f s   overall rate: %.2f Hz" % (span, (len(rows) - 1) / span))
    print("interval ms (all): min=%.1f median=%.1f max=%.1f"
          % (min(dts), statistics.median(dts), max(dts)))

    # ---- rate while the crank is actually turning (what aliasing depends on) ----
    act_dt, act_n = [], 0
    for i in range(len(rows) - 1):
        if cad[i] > 0 and cad[i + 1] > 0:
            act_dt.append(ts[i + 1] - ts[i])
            act_n += 1
    if act_n >= 5:
        rate = act_n / sum(act_dt)
        print("ACTIVE rate (cadence>0): %.2f Hz over %d frames" % (rate, act_n))
        print("interval ms (active): min=%.1f median=%.1f max=%.1f"
              % (min(act_dt) * 1000, statistics.median(act_dt) * 1000,
                 max(act_dt) * 1000))
    else:
        rate = (len(rows) - 1) / span if span > 0 else 0
        print("ACTIVE rate: not enough rotating frames (act_n=%d), using overall %.2f Hz"
              % (act_n, rate))

    # ---- THE decisive question: does the meter stream faster under load? ----
    pw = [(r["total"] or 0) for r in rows]
    print()
    print("max power seen: %d W   frames with power>0: %d / %d"
          % (max(pw) if pw else 0, sum(1 for v in pw if v > 0), len(pw)))
    loaded_rate = None
    for label, sel in (("power>0 (LOADED)", lambda i: pw[i] > 0),
                       ("power=0 (no load)", lambda i: pw[i] == 0)):
        d2 = [ts[i + 1] - ts[i] for i in range(len(rows) - 1)
              if sel(i) and sel(i + 1)]
        if len(d2) >= 3:
            r2 = len(d2) / sum(d2)
            print("  rate %-20s = %.2f Hz   (n=%3d, median interval %4.0f ms)"
                  % (label, r2, len(d2), statistics.median(d2) * 1000))
            if "LOADED" in label:
                loaded_rate = r2
        else:
            print("  rate %-20s = n/a (only %d frames)" % (label, len(d2)))

    if loaded_rate:
        print()
        print("  -> under load the rate is %.2fx the 1 Hz baseline" % loaded_rate)
        rate = max(rate, loaded_rate)

    ceiling = 60.0 * rate
    print()
    print(">>> ALIASING CEILING: %.0f rpm  (60 x %.2f Hz)" % (ceiling, rate))
    print(">>> angle counting is safe for cadence below this value")

    # ---- is the angle field genuine? compare measured vs expected step ----
    print()
    print("consistency check: measured d_angle  vs  cadence-implied d_angle")
    print("  (only pairs whose EXPECTED step is < 360 deg are unambiguous)")
    ok = bad = 0
    worst = 0.0
    checked = 0
    for i in range(len(rows) - 1):
        if cad[i] <= 0:
            continue
        dt = ts[i + 1] - ts[i]
        exp = cad[i] / 60.0 * 360.0 * dt
        if exp >= 355:
            continue
        meas = (ang[i + 1] - ang[i]) % 360
        checked += 1
        err = abs(meas - exp)
        if err > 45:
            bad += 1
            if err > worst:
                worst = err
        else:
            ok += 1
    if checked:
        print("  checked %d pairs -> consistent %d, off-by-much %d (worst err %.0f deg)"
              % (checked, ok, bad, worst))
        print("  ratio ok/total = %.1f%%" % (100.0 * ok / checked))
    else:
        print("  no unambiguous pairs (crank never turned slowly enough)")

    # ---- what would angle-counting yield? ----
    wraps = sum(1 for i in range(len(ang) - 1) if ang[i + 1] < ang[i])
    metered = [c for c in cad if c]
    print()
    if span > 0:
        print("wraps observed: %d  -> %.1f rpm if each wrap = 1 revolution"
              % (wraps, wraps * 60.0 / span))
    if metered:
        print("meter-reported cadence: mean=%.1f max=%d (nonzero frames=%d)"
              % (statistics.mean(metered), max(metered), len(metered)))

    # ---- verdict ----
    print()
    print("=" * 70)
    if ceiling >= 150:
        print("VERDICT: VIABLE - safe up to %.0f rpm, well above real cycling." % ceiling)
    elif ceiling >= 100:
        print("VERDICT: MARGINAL - safe only up to %.0f rpm." % ceiling)
    else:
        print("VERDICT: NOT VIABLE - only safe below %.0f rpm; must keep integrating."
              % ceiling)

    print()
    print("first 20 frames: t, dt_ms, cadence, angle, d_angle, expected")
    for i, r in enumerate(rows[:20]):
        if i == 0:
            print("  %7.3f      -      %5s %5s      -        -"
                  % (r["t"], r["cadence"], r["angle"]))
        else:
            d = (rows[i]["angle"] - rows[i - 1]["angle"]) % 360
            dt = r["t"] - rows[i - 1]["t"]
            e = rows[i - 1]["cadence"] / 60.0 * 360.0 * dt
            print("  %7.3f %7.1f   %5s %5s   %4d   %7.1f"
                  % (r["t"], dt * 1000, r["cadence"], r["angle"], d, e))

    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["t", "dt_ms", "total_w", "left_w", "right_w",
                    "cadence", "angle", "err", "len", "raw_hex"])
        prev = None
        for r in rows:
            dt = "" if prev is None else "%.2f" % ((r["t"] - prev) * 1000)
            prev = r["t"]
            w.writerow(["%.4f" % r["t"], dt, r["total"], r["left"], r["right"],
                        r["cadence"], r["angle"], r["err"], r["len"], r["raw"]])
    print()
    print("CSV written: %s" % csv_path)


async def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=90)
    ap.add_argument("--address", default=None)
    ap.add_argument("--scan-only", action="store_true")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    cands = await scan_devices()

    if args.scan_only:
        print()
        print("candidates: %d" % len(cands))
        return
    if not cands:
        print()
        print("no candidate found (service 0x1828 / name XDS).")
        print("wake the meter (pedal once) and make sure no phone/computer is holding it.")
        return

    dev = cands[0]
    if args.address:
        for d in cands:
            if d.address.lower() == args.address.lower():
                dev = d
                break
    print()
    print("connecting %s ..." % dev.address)

    cap = Cap()
    async with BleakClient(dev) as client:
        print("connected")
        try:
            await client.write_gatt_char(UUID_CP_CTRL, XDS_START_COMMAND, response=True)
            print("start command sent to 0x2A55")
        except Exception as exc:
            print("write 0x2A55 failed: %s (continuing)" % exc)

        await client.start_notify(UUID_CP_MEAS, cap.on_notify)
        print("subscribed 0x2A63. capturing %d s -- PEDAL NOW" % args.seconds)
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            await asyncio.sleep(0.2)
        try:
            await client.stop_notify(UUID_CP_MEAS)
        except Exception:
            pass

    out = args.out or os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                   "notify_rate.csv")
    analyze(cap, out)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("interrupted")
