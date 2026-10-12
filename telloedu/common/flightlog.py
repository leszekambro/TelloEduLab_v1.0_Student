"""Backend-neutral telemetry logging utilities."""
from __future__ import annotations
import csv
import time
from pathlib import Path
import numpy as np

FIELDS = ["time_s","x_m","y_m","z_m","roll_deg","pitch_deg","yaw_deg",
          "vx_mps","vy_mps","vz_mps","height_cm","tof_cm","battery_pct",
          "cmd_lr","cmd_fb","cmd_ud","cmd_yaw"]

class FlightLogger:
    def __init__(self, name="drone"):
        self.name = str(name); self.clear()
    def clear(self): self.rows=[]; self.t0=time.perf_counter()
    def append(self, **values):
        row={k: np.nan for k in FIELDS}; row.update(values)
        row["time_s"] = values.get("time_s", time.perf_counter()-self.t0)
        self.rows.append(row); return row
    def arrays(self):
        return {k: np.asarray([r[k] for r in self.rows], dtype=float) for k in FIELDS}
    def save_csv(self, filename):
        path=Path(filename); path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as f:
            w=csv.DictWriter(f, fieldnames=FIELDS); w.writeheader(); w.writerows(self.rows)
        return str(path)
    def plot(self, show=True):
        import matplotlib.pyplot as plt
        if not self.rows: return []
        a=self.arrays(); figs=[]
        if any(np.isfinite(a[k]).any() for k in ("x_m","y_m","z_m")):
            fig,ax=plt.subplots(3,1,sharex=True,figsize=(9,7)); figs.append(fig)
            for i,k in enumerate(("x_m","y_m","z_m")):
                ax[i].plot(a["time_s"],a[k]); ax[i].set_ylabel(k); ax[i].grid(True)
            ax[-1].set_xlabel("Time [s]"); fig.suptitle(f"{self.name}: simulator ground truth / external position"); fig.tight_layout()
        # attitude and velocity are supported on both backends
        if False:
            ax[i].plot(a["time_s"],a[k]); ax[i].set_ylabel(k); ax[i].grid(True)
        ax[-1].set_xlabel("Time [s]"); fig.suptitle(f"{self.name}: position / ground truth"); fig.tight_layout()
        fig,ax=plt.subplots(3,1,sharex=True,figsize=(9,7)); figs.append(fig)
        for i,k in enumerate(("roll_deg","pitch_deg","yaw_deg")):
            ax[i].plot(a["time_s"],a[k]); ax[i].set_ylabel(k); ax[i].grid(True)
        ax[-1].set_xlabel("Time [s]"); fig.suptitle(f"{self.name}: attitude"); fig.tight_layout()
        fig,ax=plt.subplots(3,1,sharex=True,figsize=(9,7)); figs.append(fig)
        for i,k in enumerate(("vx_mps","vy_mps","vz_mps")):
            ax[i].plot(a["time_s"],a[k]); ax[i].set_ylabel(k); ax[i].grid(True)
        ax[-1].set_xlabel("Time [s]"); fig.suptitle(f"{self.name}: velocity"); fig.tight_layout()
        if show: plt.show()
        return figs
