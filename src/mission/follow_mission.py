"""Fully-autonomous follow mission + controller — the SAME logic that will run on the
Jetson for the mocap demo. No human piloting; the only human action is an emergency
kill (not modelled). Phases: TAKEOFF -> SEARCH -> FOLLOW(keep d0) -> SAFETY_HOLD.

Perception is monocular: distance is inferred from the target's apparent size (YOLO box
height). The adversarial patch shrinks the box -> perceived distance inflates -> the
FOLLOW law closes in -> the AUTONOMOUS geofence (true distance from mocap) halts it at
d_safe BEFORE contact. A consistency check (perceived vs mocap) also flags the deceit.

On the real drone, replace `box_h_from_truth` with the live YOLO box height and feed the
velocity command to the FC over MAVLink; everything else is identical.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

TAKEOFF, SEARCH, FOLLOW, SAFETY_HOLD = "TAKEOFF", "SEARCH", "FOLLOW", "SAFETY_HOLD"


@dataclass
class Params:
    f_px: float = 600.0      # camera focal (px)
    S: float = 1.0           # real target height (m)
    d0: float = 3.0          # desired standoff (m)
    d_safe: float = 1.5      # autonomous geofence: never let TRUE gap go below this
    hover_z: float = 1.5     # takeoff altitude (m)
    kp: float = 0.6
    vmax: float = 0.6        # slow, for safety (m/s)
    climb: float = 0.5       # m/s takeoff
    dt: float = 0.05
    tau: float = 0.2         # deceit flag threshold (perceived vs mocap mismatch)
    patch_r: float = 0.46    # measured box-shrink ratio when patch is visible


def box_h_from_truth(d_true, p: Params, patch_on: bool):
    """Apparent box height; on the real drone this comes from YOLO instead."""
    r = p.patch_r if patch_on else 1.0
    return p.f_px * p.S / max(d_true, 1e-3) * r


@dataclass
class Mission:
    p: Params = field(default_factory=Params)
    state: str = TAKEOFF
    z: float = 0.0
    x: float = 0.0           # drone forward position
    deceived_flag_t: float | None = None

    def step(self, t, target_x, patch_on, mocap_gap):
        """One autonomous control tick. Returns (vx, vz, perceived_d, info)."""
        p = self.p
        vx = vz = 0.0
        perceived_d = None
        # --- TAKEOFF ---
        if self.state == TAKEOFF:
            vz = p.climb
            if self.z >= p.hover_z - 0.02:
                self.z = p.hover_z; self.state = SEARCH
        # --- SEARCH: hold until a target box is seen (here: as soon as in range) ---
        elif self.state == SEARCH:
            if mocap_gap < 8.0:
                self.state = FOLLOW
        # --- FOLLOW: keep standoff using the PERCEIVED distance (box height) ---
        elif self.state == FOLLOW:
            box = box_h_from_truth(mocap_gap, p, patch_on)
            perceived_d = p.f_px * p.S / max(box, 1e-3)
            vx = float(np.clip(p.kp * (perceived_d - p.d0), -p.vmax, p.vmax))
            # consistency check (would use an independent depth sensor on the real drone;
            # here mocap stands in): flag deceit when perceived disagrees with truth
            mism = abs(perceived_d - mocap_gap) / max(mocap_gap, 1e-3)
            if mism > p.tau and self.deceived_flag_t is None:
                self.deceived_flag_t = t
            # AUTONOMOUS geofence: the program itself stops before contact
            if mocap_gap <= p.d_safe:
                self.state = SAFETY_HOLD
        # --- SAFETY_HOLD: frozen, no contact ---
        elif self.state == SAFETY_HOLD:
            vx = 0.0
        # integrate drone motion
        self.z = min(self.z + vz * p.dt, p.hover_z)
        self.x = self.x + vx * p.dt
        return vx, vz, perceived_d, dict(state=self.state, deceived=self.deceived_flag_t is not None)
