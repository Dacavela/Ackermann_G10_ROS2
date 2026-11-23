#!/usr/bin/env python3
"""Reproduce V4 Gazebo figure-eight plots from the CSV log.

Usage:
    python3 scripts/plot_ackermann_v4.py \
      results/resultado_world_precision_2cm.csv --output assets

Dependencies: python3-numpy python3-pandas python3-matplotlib
"""
import argparse
from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

parser = argparse.ArgumentParser()
parser.add_argument("csv", type=Path)
parser.add_argument("--output", type=Path, default=Path("assets"))
args = parser.parse_args()
args.output.mkdir(parents=True, exist_ok=True)

d = pd.read_csv(args.csv)
fields = ["tiempo_s", "x_real", "y_real", "error_seguimiento_m"]
assert set(fields).issubset(d.columns), "Unexpected CSV column names"
t = d.tiempo_s.to_numpy()
x = d.x_real.to_numpy()
y = d.y_real.to_numpy()
error = d.error_seguimiento_m.to_numpy()
distance_cm = float(np.hypot(x[-1], y[-1]) * 100)
mean_cm = float(100*np.mean(error))
max_cm = float(100*np.max(error))
rmse_cm = float(100*np.sqrt(np.mean(error**2)))
radius = 2.6  # configured trajectory radius parameter, meters
theta = np.linspace(0, 2*np.pi, 1500)
xr = radius*np.sin(theta)
yr = radius*np.sin(theta)*np.cos(theta)

fig, ax = plt.subplots(figsize=(12.6, 8.3))
ax.plot(xr, yr, ls="--", lw=2.6, label="Desired figure-eight trajectory (mathematical reference)")
ax.plot(x, y, lw=2.2, alpha=0.94, label="Gazebo ground-truth trajectory (V4, 0.30 m/s)")
ax.plot(x[0], y[0], marker="o", ms=9, ls="None", label="Start: (0, 0) m", zorder=5)
ax.plot(x[-1], y[-1], marker="X", ms=11, ls="None",
        label=f"Final logged position: ({x[-1]:+.3f}, {y[-1]:+.3f}) m", zorder=6)
ax.set_title("Ackermann Robot | Final Figure-Eight Trajectory", fontsize=18,
             weight="bold", pad=19)
ax.set(xlabel="X position (m)", ylabel="Y position (m)")
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-2.83, 2.83)
ax.set_ylim(-1.53, 1.53)
ax.grid(True, alpha=0.23)
ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.14), ncol=2,
          frameon=False, fontsize=10)
fig.subplots_adjust(top=0.86, bottom=0.27, left=0.10, right=0.98)
fig.text(.5, .875,
         f"ROS 2 Jazzy · Gazebo Harmonic · Pure Pursuit · /ground_truth/odom · "
         f"{t[-1]:.2f} s · {len(d):,} samples",
         ha="center", va="center", fontsize=11)
fig.text(.5, .055,
         f"Mean tracking error: {mean_cm:.2f} cm    |    "
         f"Max tracking error: {max_cm:.2f} cm    |    "
         f"RMSE: {rmse_cm:.2f} cm    |    "
         f"Final logged distance to origin: {distance_cm:.2f} cm",
         ha="center", fontsize=11, weight="bold")
fig.text(.5, .021,
         "Simulation results, not measurements from a physical robot. Final distance is computed from the CSV.",
         ha="center", fontsize=9)
fig.savefig(args.output/"ackermann_v4_trajectory.png", dpi=220, bbox_inches="tight")
fig.savefig(args.output/"ackermann_v4_trajectory.svg", bbox_inches="tight")
plt.close(fig)

fig, ax = plt.subplots(figsize=(12, 5.2))
ax.plot(t, 100*error, lw=1.9, label="Cross-track tracking error")
ax.axhline(mean_cm, ls="--", lw=1.5, label=f"Mean error: {mean_cm:.2f} cm")
p = int(np.argmax(error))
ax.plot(t[p], 100*error[p], marker="o", ms=7, ls="None",
        label=f"Maximum: {max_cm:.2f} cm at {t[p]:.2f} s")
ax.set(title="Ackermann Robot | Tracking Error During Final V4 Run",
       xlabel="Simulation time (s)", ylabel="Tracking error (cm)")
ax.set_xlim(0, t[-1])
ax.set_ylim(bottom=0)
ax.grid(True, alpha=0.25)
ax.legend(loc="upper right", frameon=False)
fig.tight_layout()
fig.savefig(args.output/"ackermann_v4_tracking_error.png", dpi=220, bbox_inches="tight")
plt.close(fig)

fig, ax = plt.subplots(figsize=(8.3, 7.4))
ax.plot(x[-35:], y[-35:], lw=2.0, label="Final approach")
ax.plot(0, 0, marker="o", ls="None", ms=8, label="Target: (0, 0) m")
ax.plot(x[-1], y[-1], marker="X", ls="None", ms=10,
        label=f"Logged endpoint: {distance_cm:.2f} cm from target")
ax.add_patch(Circle((0, 0), 0.02, fill=False, ls="--", lw=2,
                    label="Stopping tolerance: 2 cm"))
ax.set_aspect("equal", adjustable="box")
ax.set_xlim(-0.145, 0.11)
ax.set_ylim(-0.145, 0.11)
ax.set(xlabel="X position (m)", ylabel="Y position (m)")
ax.set_title("Final Position | 2 cm Stopping Tolerance", fontsize=15, pad=14)
ax.grid(True, alpha=0.25)
ax.legend(loc="upper left", fontsize=9.2, frameon=True)
fig.tight_layout()
fig.savefig(args.output/"ackermann_v4_final_position_zoom.png",
            dpi=220, bbox_inches="tight")
plt.close(fig)
print(f"Samples: {len(d)}, duration: {t[-1]:.2f} s")
print(f"Mean: {mean_cm:.2f} cm, Max: {max_cm:.2f} cm, RMSE: {rmse_cm:.2f} cm")
print(f"Logged final distance: {distance_cm:.2f} cm")
