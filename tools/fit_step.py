#!/usr/bin/env python3
"""
拟合 M4c 电压阶跃响应：i(t) = I_f − (I_f − I_0)·exp(−(t − t0)/τ)，L = τ·R
用法：tools/fit_step.py data/m4c_step_xxx.csv [R_ohm=3.212]
CSV 列：k, t_us（以写入阶跃电压的那次中断为 0）, id_A
"""
import csv, math, sys

path = sys.argv[1]
R = float(sys.argv[2]) if len(sys.argv) > 2 else 3.212
t, i = [], []
with open(path) as f:
    for r in csv.DictReader(f):
        t.append(int(r["t_us"]) * 1e-6); i.append(float(r["id_A"]))

base = sum(v for tt, v in zip(t, i) if tt < 0) / sum(1 for tt in t if tt < 0)
tail = i[-60:]; fin0 = sum(tail) / len(tail)

best = None
for dfin in range(-40, 41, 2):                       # 在稳态均值附近 ±4 mA 搜终值
    fin = fin0 + dfin * 1e-4
    for t0_us in range(0, 51):                        # 搜纯延迟 t0（0~50 µs）
        t0 = t0_us * 1e-6
        xs, ys = [], []
        for tt, v in zip(t, i):                       # 对数线性化：ln f = −(t − t0)/τ
            if tt <= t0: continue
            fr = (fin - v) / (fin - base)
            if 0.05 < fr < 0.95: xs.append(tt - t0); ys.append(math.log(fr))
        if len(xs) < 4: continue
        tau = -sum(x * x for x in xs) / sum(x * y for x, y in zip(xs, ys))
        err = sum((v - (base if tt < t0 else fin - (fin - base) * math.exp(-(tt - t0) / tau))) ** 2
                  for tt, v in zip(t, i))
        if best is None or err < best[0]: best = (err, fin, t0_us, tau)

err, fin, t0_us, tau = best
print(f"I_0 = {base:.4f} A, I_f = {fin:.4f} A, dI = {fin - base:.4f} A")
print(f"pure delay t0 = {t0_us} us, tau = {tau * 1e6:.1f} us, fit rms = {math.sqrt(err / len(t)) * 1e3:.2f} mA")
print(f"L = tau * R = {tau * R * 1e3:.3f} mH   (R = {R} ohm)")
