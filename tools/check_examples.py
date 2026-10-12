"""Static syntax and inventory validation; no drone flight or simulator launch."""
from pathlib import Path
import ast,csv,sys
root=Path(__file__).resolve().parents[1]
rows=list(csv.DictReader((root/'docs/EXAMPLE_CATALOG.csv').open()))
errors=[]
for row in rows:
    p=root/'examples'/row['example']
    try:ast.parse(p.read_text(),filename=str(p))
    except (SyntaxError,UnicodeError) as e:errors.append((str(p),str(e)))
print(f"Examples: {len(rows)} | syntax failures: {len(errors)}")
for p,e in errors:print(p,e)
sys.exit(bool(errors))
