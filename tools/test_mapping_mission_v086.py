"""Offline regression tests, no PyBullet needed."""
import ast
import sys
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from telloedu.lab.mapping2d import OccupancyGrid2D
ROOT = Path(__file__).resolve().parents[1]
for lab in ('13_slam', '14_autonomous_mission'):
    a = ast.parse((ROOT / 'labs' / lab / 'starter.py').read_text())
    b = ast.parse((ROOT / 'labs_solutions' / lab / 'solution.py').read_text())
    def signature(tree):
        return [(n.name, [x.arg for x in n.args.args]) for n in tree.body if isinstance(n, ast.FunctionDef)]
    assert signature(a) == signature(b), lab
    assert 'labs_solutions' not in (ROOT / 'labs' / lab / 'starter.py').read_text()
    print('Aligned:', lab)
g = OccupancyGrid2D((-2, 2), (-2, 2), 0.1)
g.integrate_scan(np.array([0.,0.]), 0., np.array([0., np.pi/2]), np.array([1., 2.]), 2.)
assert g.explored_count() > 10
assert g.occupied_count() > 0
print('PASS mapping2d ray integration')
