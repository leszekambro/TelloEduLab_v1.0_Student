from pathlib import Path
import ast,csv
r=Path(__file__).resolve().parents[1]
rows=list(csv.DictReader((r/'docs'/'COURSE_ORDER.csv').open()))
errors=[]
for row in rows:
 for k in ('original','launcher'):
  p=r/row[k]
  try:ast.parse(p.read_text(encoding='utf-8'),filename=str(p))
  except Exception as e:errors.append((str(p),str(e)))
print(f'Checked {len(rows)} course items / {len(rows)*2} Python files; errors: {len(errors)}')
for e in errors:print(*e)
raise SystemExit(bool(errors))
