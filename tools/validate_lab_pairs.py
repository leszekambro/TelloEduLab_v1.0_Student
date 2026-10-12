"""Verify 14 laboratory pairs share identical scaffolding and differ only in experiment()."""
from pathlib import Path
import ast
import sys
ROOT=Path(__file__).resolve().parents[1]
errors=[]
for starter in sorted((ROOT/'labs').glob('*/starter.py')):
    solution=ROOT/'labs_solutions'/starter.parent.name/'solution.py'
    if not solution.exists():
        errors.append(f'Missing solution: {starter.parent.name}'); continue
    a=ast.parse(starter.read_text()); b=ast.parse(solution.read_text())
    def strip_exp(t):
        for node in t.body:
            if isinstance(node,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='EXPERIMENT_READY' for t in node.targets):
                node.value=ast.Constant(value=True)
            if isinstance(node,ast.FunctionDef) and node.name=='experiment':
                node.body=[ast.Pass()]
        return ast.dump(t,include_attributes=False)
    if strip_exp(a)!=strip_exp(b):
        errors.append(f'Non-algorithm scaffolding differs: {starter.parent.name}')
    if 'from labs_solutions' in starter.read_text():
        errors.append(f'Student starter imports instructor code: {starter.parent.name}')
    print(('FAIL' if errors and errors[-1].endswith(starter.parent.name) else 'CHECK'),starter.parent.name)
print(f'Pairs checked: {len(list((ROOT/"labs").glob("*/starter.py")))}; errors: {len(errors)}')
if errors:
    print('\n'.join(errors)); sys.exit(1)
print('PASS: All student/instructor scaffolds match.')
