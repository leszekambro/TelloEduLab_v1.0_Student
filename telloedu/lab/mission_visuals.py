"""Mission plotting with an explicit, independent, non-interactive PNG renderer."""
from pathlib import Path
import numpy as np


def _paint(ax, path, mission, target=None, stage='', camera=None):
    waypoints = np.asarray([p for _, p in mission], dtype=float)
    ax.plot(waypoints[:, 0], waypoints[:, 1], 'o--', lw=1.4, label='Plan')
    if len(path):
        pts = np.asarray(path, dtype=float).reshape(-1, 3)
        ax.plot(pts[:, 0], pts[:, 1], lw=2, label='Flight (SIM GT)')
        ax.scatter(pts[-1, 0], pts[-1, 1], s=65, marker='^', label='Drone')
    else:
        ax.scatter(0, 0, marker='^', s=65, label='Start')
    if target is not None:
        ax.scatter(target[0], target[1], marker='*', s=140, label='Target')
    for label, xyz in mission:
        ax.annotate(label, xyz[:2], xytext=(4, 5), textcoords='offset points', fontsize=7)
    ax.set(xlabel='X [m]', ylabel='Y [m]', title=f'Mission XY | {stage}')
    ax.set_xlim(-0.8, 4.0)
    ax.set_ylim(-2.0, 1.8)
    ax.set_aspect('equal', adjustable='box')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, loc='upper left')


def _camera(ax, camera):
    if camera is None:
        ax.text(.5, .5, 'Waiting for live camera frame', ha='center', va='center', transform=ax.transAxes)
    else:
        frame = np.asarray(camera)
        if frame.ndim == 3 and frame.shape[-1] in (3, 4):
            ax.imshow(frame[..., :3])
        else:
            ax.text(.5, .5, 'Unexpected camera format', ha='center', va='center', transform=ax.transAxes)
    ax.set_title('SIM onboard camera')
    ax.axis('off')


def save_mission_view(filename, path, mission, target=None, stage='', camera=None):
    # Figure + Agg canvas prevents pyplot backend switching or an empty GUI
    # figure from affecting the saved file.
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    fig = Figure(figsize=(13, 5.5), dpi=130)
    FigureCanvasAgg(fig)
    left, right = fig.subplots(1, 2)
    _paint(left, path, mission, target, stage)
    _camera(right, camera)
    fig.tight_layout()
    filename = Path(filename)
    filename.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(filename, dpi=130)


def update_live_mission(display, path, mission, target=None, stage='', camera=None):
    plt, fig, axes = display
    axes[0].clear()
    axes[1].clear()
    _paint(axes[0], path, mission, target, stage)
    _camera(axes[1], camera)
    fig.canvas.draw_idle()
    fig.canvas.flush_events()
    plt.pause(0.03)
