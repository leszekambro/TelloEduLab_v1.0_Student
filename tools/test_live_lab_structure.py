"""Offline checks; does not start PyBullet or fly a drone."""
import ast
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
files=sorted((ROOT/'labs_solutions').glob('*/solution.py'))
assert len(files)==14, f'Expected 14 solutions, found {len(files)}'
for f in files:
    source=f.read_text(encoding='utf-8');tree=ast.parse(source)
    assert 'from telloedu import create_tello' in source, f
    assert 'drone = create_tello(' in source, f
    assert 'drone.connect()' in source, f
    assert 'def main():' in source, f
    assert 'finally:' in source, f
    assert 'run.add(' in source, f
    assert 'np.random' not in source, f
    assert 'from lab_common import *' not in source, f
    assert 'MODE = "sim"' in source, f
    assert 'HOST = "192.168.10.1"' in source, f
    print('PASS',f.parent.name)
print(f'STATIC PASS: {len(files)} labs. No flight tests performed.')
