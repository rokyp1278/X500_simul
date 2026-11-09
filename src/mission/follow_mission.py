"""FULLY AUTONOMOUS follow mission + controller — the SAME logic that will run on the
Jetson for the mocap demo. No human piloting; the only human action is an emergency
kill (not modelled). Phases:

  TAKEOFF -> SEARCH(yaw-sweep until the object enters the camera FOV & detect range)
          -> ACQUIRE -> FOLLOW(yaw-center the box + hold standoff d0) -> SAFETY_HOLD

This is a 2-D kinematic model (x, y, yaw) so SEARCH and yaw-centering are real: the
target sits at a world point the drone does NOT know in advance; it must rotate to find
it, turn to face it, then close to d0. Perception is monocular — distance from the
target's apparent size (YOLO box height), bearing from the box's horizontal offset.

The adversarial patch shrinks the box -> perceived distance inflates -> FOLLOW closes in
-> the AUTONOMOUS geofence (true gap from mocap) halts it at d_safe before contact; a
consistency check (perceived vs independent depth) also flags the deceit.

On the real drone: replace `_perceive` with the live YOLO box (height -> distance,
horizontal centre -> bearing error) and send (vx, yaw_rate, vz) to the FC over MAVLink.
Everything else — the state machine and the control laws — is identical.
"""
from __future__ import annotations
from dataclasses import dataclass, field
import numpy as np

TAKEOFF, SEARCH, FOLLOW, SAFETY_HOLD = "TAKEOFF", "SEARCH", "FOLLOW", "SAFETY_HOLD"


def angdiff(a, b):
    """Smallest signed angle a-b in (-pi, pi]."""
    return (a - b + np.pi) % (2 * np.pi) - np.pi


@dataclass
class Params:
    f_px: float = 600.0        # camera focal (px)
    S: float = 0.6             # real target height (m)  -- a ~0.6 m board in the cube
    fov: float = 1.15          # camera horizontal FOV (rad, ~66 deg)
    d0: float = 1.5            # desired standoff (m)
    d_detect: float = 3.0      # max range YOLO reliably detects the board (m)
    d_safe: float = 0.7        # autonomous geofence: never let TRUE gap go below this
    hover_z: float = 1.0       # takeoff altitude (m)
    kp: float = 0.6            # forward (standoff) gain
    yaw_kp: float = 1.5        # yaw-centering gain (rad/s per rad error)
    yaw_search: float = 0.6    # SEARCH sweep rate (rad/s)
    center_tol: float = 0.12   # start approaching only when |bearing err| < this (rad)
    vmax: float = 0.5          # slow, for safety (m/s)
    yaw_rate_max: float = 1.0  # rad/s
    climb: float = 0.5         # m/s takeoff
    dt: float = 0.05
    tau: float = 0.2           # deceit flag threshold (perceived vs independent depth)
    patch_r: float = 0.46      # measured box-shrink ratio when the patch is visible


@dataclass
class Mission:
    p: Params = field(default_factory=Params)
    state: str = TAKEOFF
    z: float = 0.0
    x: float = 0.0             # drone world position
    y: float = 0.0
    yaw: float = 0.0           # heading (rad); camera looks along +heading
    deceived_flag_t: float | None = None
    acquire_t: float | None = None

    def _perceive(self, target_xy, patch_on):
        """Monocular perception from the drone pose. On the real drone this is YOLO:
        returns (seen, box_h, bearing_err, true_gap). box_h -> distance, the box's
        horizontal offset -> bearing_err."""
        p = self.p
        dx, dy = target_xy[0] - self.x, target_xy[1] - self.y
        true_gap = float(np.hypot(dx, dy))
        bearing = np.arctan2(dy, dx)
        berr = angdiff(bearing, self.yaw)             # +ve: target is to the left
        in_fov = abs(berr) <= p.fov / 2
        seen = in_fov and true_gap <= p.d_detect
        r = p.patch_r if patch_on else 1.0
        box_h = p.f_px * p.S / max(true_gap, 1e-3) * r if seen else 0.0
        return seen, box_h, berr, true_gap

    def step(self, t, target_xy, patch_on, depth_anchor=None):
        """One autonomous control tick. target_xy = world (x,y) of the object (UNKNOWN to
        the controller except through perception). depth_anchor = independent range
        reading (mocap/stereo) used only for the safety geofence + deceit check.
        Returns (vx, yaw_rate, vz, info)."""
        p = self.p
        vx = yaw_rate = vz = 0.0
        seen, box_h, berr, true_gap = self._perceive(target_xy, patch_on)
        if depth_anchor is None:
            depth_anchor = true_gap
        perceived_d = (p.f_px * p.S / max(box_h, 1e-3)) if box_h > 0 else None

        if self.state == TAKEOFF:
            vz = p.climb
            if self.z >= p.hover_z - 0.02:
                self.z = p.hover_z; self.state = SEARCH

        elif self.state == SEARCH:
            yaw_rate = p.yaw_search                     # sweep in place to find the object
            if seen:
                self.state = FOLLOW; self.acquire_t = t

        elif self.state == FOLLOW:
            if not seen:                                # lost it -> go back to searching
                self.state = SEARCH
            else:
                yaw_rate = float(np.clip(p.yaw_kp * berr, -p.yaw_rate_max, p.yaw_rate_max))
                # only drive forward once the object is roughly centred
                if abs(berr) < p.center_tol:
                    vx = float(np.clip(p.kp * (perceived_d - p.d0), -p.vmax, p.vmax))
                # deceit check: perceived vs independent depth anchor
                mism = abs(perceived_d - depth_anchor) / max(depth_anchor, 1e-3)
                if mism > p.tau and self.deceived_flag_t is None:
                    self.deceived_flag_t = t
                # AUTONOMOUS geofence: the program itself stops before contact
                if true_gap <= p.d_safe:
                    self.state = SAFETY_HOLD

        elif self.state == SAFETY_HOLD:
            vx = yaw_rate = 0.0

        # integrate (forward speed along heading)
        self.yaw = (self.yaw + yaw_rate * p.dt + np.pi) % (2 * np.pi) - np.pi
        self.z = min(self.z + vz * p.dt, p.hover_z)
        self.x += vx * np.cos(self.yaw) * p.dt
        self.y += vx * np.sin(self.yaw) * p.dt
        return vx, yaw_rate, vz, dict(state=self.state, seen=seen, box_h=box_h,
                                      perceived_d=perceived_d, true_gap=true_gap,
                                      bearing_err=berr, deceived=self.deceived_flag_t is not None)


def run(target_xy=(2.2, 1.3), patch_on=True, t_max=60.0, p: Params = None):
    """Headless rollout for analysis/plots. Returns the trajectory log."""
    p = p or Params()
    m = Mission(p=p)
    log = []
    t = 0.0
    while t < t_max:
        vx, yr, vz, info = m.step(t, target_xy, patch_on)
        log.append((t, m.x, m.y, m.z, m.yaw, info["true_gap"],
                    info["perceived_d"] if info["perceived_d"] else np.nan,
                    info["state"], int(info["deceived"])))
        if info["state"] == SAFETY_HOLD and t > 5:
            # run a little past the halt, then stop
            if all(r[7] == SAFETY_HOLD for r in log[-20:]):
                break
        t += p.dt
    return log


if __name__ == "__main__":
    for patch in (False, True):
        lg = run(patch_on=patch)
        final_gap = lg[-1][5]
        states = [r[7] for r in lg]
        print("patch=%-5s  final_true_gap=%.2fm  reached=%s  deceived=%s"
              % (patch, final_gap, states[-1], any(r[8] for r in lg)))
