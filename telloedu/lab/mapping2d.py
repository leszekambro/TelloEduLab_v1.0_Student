"""Log-odds 2D occupancy grid for simulated range measurements."""
import numpy as np


class OccupancyGrid2D:
    def __init__(self, xlim, ylim, resolution=0.08):
        self.xmin, self.xmax = map(float, xlim)
        self.ymin, self.ymax = map(float, ylim)
        self.resolution = float(resolution)
        self.nx = int(np.ceil((self.xmax - self.xmin) / self.resolution))
        self.ny = int(np.ceil((self.ymax - self.ymin) / self.resolution))
        self.log_odds = np.zeros((self.ny, self.nx), dtype=np.float32)
        self.seen = np.zeros((self.ny, self.nx), dtype=bool)

    def cell(self, x, y):
        ix = int(np.floor((x - self.xmin) / self.resolution))
        iy = int(np.floor((y - self.ymin) / self.resolution))
        return (iy, ix) if 0 <= ix < self.nx and 0 <= iy < self.ny else None

    def integrate_scan(self, xy, yaw, angles, ranges, max_range):
        for angle, measured in zip(angles, ranges):
            distance = float(np.clip(measured, 0, max_range))
            # Free-space tracing stops one cell before an obstacle hit.
            heading = float(yaw + angle)
            c, s = np.cos(heading), np.sin(heading)
            hit = distance < max_range - 0.02
            free_limit = max(0.0, distance - (self.resolution if hit else 0.0))
            visited = set()
            for radius in np.arange(0.0, free_limit + 1e-8, self.resolution * 0.65):
                ij = self.cell(xy[0] + radius * c, xy[1] + radius * s)
                if ij is not None and ij not in visited:
                    self.log_odds[ij] = np.clip(self.log_odds[ij] - 0.24, -4, 4)
                    self.seen[ij] = True
                    visited.add(ij)
            if hit:
                ij = self.cell(xy[0] + distance * c, xy[1] + distance * s)
                if ij is not None:
                    self.log_odds[ij] = np.clip(self.log_odds[ij] + 0.9, -4, 4)
                    self.seen[ij] = True

    def occupied_count(self):
        return int(np.count_nonzero(self.seen & (self.log_odds > 0.5)))

    def explored_count(self):
        return int(np.count_nonzero(self.seen))


def plot_occupancy(grid, path, waypoints, filename, display=None):
    import matplotlib.pyplot as plt
    if display is None:
        fig, ax = plt.subplots(figsize=(10, 6))
    else:
        plt_live, fig, ax = display
        ax.clear()
    prob = 1.0 / (1.0 + np.exp(-grid.log_odds))
    img = np.where(grid.seen, prob, 0.5)
    ax.imshow(img, origin='lower', extent=[grid.xmin, grid.xmax, grid.ymin, grid.ymax],
              cmap='gray_r', vmin=0, vmax=1, interpolation='nearest')
    if path:
        xy = np.asarray(path)
        ax.plot(xy[:, 0], xy[:, 1], '-', lw=1.8, label='Flight path (SIM GT)')
        ax.plot(xy[-1, 0], xy[-1, 1], 'o', markersize=6, label='Current position')
    if waypoints:
        wp = np.asarray(waypoints)
        ax.plot(wp[:, 0], wp[:, 1], 'x--', alpha=0.5, label='Waypoints')
    ax.set(xlabel='World X [m]', ylabel='World Y [m]',
           title=f'Pose-assisted occupancy mapping — explored {grid.explored_count()} cells')
    ax.set_aspect('equal')
    ax.grid(alpha=0.2)
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout()
    fig.savefig(filename, dpi=140)
    if display is not None:
        try:
            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            plt_live.pause(0.03)
        except Exception:
            pass
    else:
        plt.close(fig)
