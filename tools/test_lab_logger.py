"""Headless logger regression test; no PyBullet or physical drone required."""
from pathlib import Path
import tempfile
import json
from labs_solutions.lab_common import LabRun

with tempfile.TemporaryDirectory() as td:
    run = LabRun('logger_test', 'sim', root=td)
    for i in range(10):
        run.add(t_s=i*0.1, fb_cmd=18 if 2 <= i <= 5 else 0,
                height_m=1.0, x_gt=i*0.01, y_gt=0.0,
                vx_mps=0.1, roll_deg=0.0, battery_pct=100)
    folder = run.save()
    summary = json.loads((folder/'summary.json').read_text())
    assert summary['samples'] == 10
    assert {'t_wall_s','t_sim_s','t_s'} <= set(summary['columns'])
    assert (folder/'commands.png').is_file()
    assert (folder/'altitude.png').is_file()
    assert (folder/'trajectory_xy.png').is_file()
    real = LabRun('real_guard', 'real', root=td)
    try:
        real.add(t_s=0.0, x_gt=1.0)
    except ValueError:
        pass
    else:
        raise AssertionError('REAL GT must be forbidden')
print('PASS: logger time, grouped plots, SIM GT, REAL GT guard')
