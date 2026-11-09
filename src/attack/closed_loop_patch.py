"""Closed-loop distance-pulling driven by the MEASURED EOT-patch shrink r(box_h).

A tracking drone keeps standoff d0 by the perceived box height. With the trained
scale patch on the target, every frame the box is shrunk by the measured r (held
across the approach by the EOT patch), so perceived distance = d_true / r -> the
drone reads the target as farther and closes in -> collision. An independent depth
anchor flags the box/anchor mismatch before impact. Grounded in real per-distance
patch measurements (outputs/eot_rows.json), not an assumed parameter.
"""
from __future__ import annotations
import numpy as np


def r_of(box_h, rows):
    """Interpolate measured shrink ratio by clean box height (rows: [scale,clean_h,patched_h,conf,ratio])."""
    xs = np.array([r[1] for r in rows]); ys = np.array([r[4] for r in rows])
    o = np.argsort(xs)
    return float(np.interp(box_h, xs[o], ys[o]))


def run(rows, f_px=600.0, S=4.0, d0=8.0, d_start=10.0, kp=0.5, dt=0.1, t_max=60.0,
        t_attack=4.0, tau=0.15, anchor_noise=0.1, defense=True, collision=0.5, seed=0):
    rng = np.random.default_rng(seed)
    d = d_start; t = 0.0; log = []; alarm_t = None; collided = False
    while t < t_max:
        true_h = f_px * S / max(d, 1e-3)
        attacking = t >= t_attack
        r = r_of(true_h, rows) if attacking else 1.0
        perceived_h = true_h * r
        d_box = f_px * S / max(perceived_h, 1e-3)        # deceived distance
        v = kp * (d_box - d0)                            # >0 -> move toward target
        # independent depth anchor + consistency defense
        d_anchor = d + rng.normal(0, anchor_noise)
        mism = abs(d_box - d_anchor) / max(d_anchor, 1e-3)
        if defense and attacking and mism > tau and alarm_t is None:
            alarm_t = t
        d = max(d - v * dt, 0.0)
        log.append((t, d, d_box, true_h, perceived_h, mism, r))
        if d <= collision:
            collided = True; break
        t += dt
    return dict(log=np.array(log), alarm_t=alarm_t, collided=collided, t_attack=t_attack)
