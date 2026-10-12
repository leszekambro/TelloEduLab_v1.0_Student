"""Build a clean student release and reject instructor-only material."""
from pathlib import Path
import shutil, sys

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / "dist" / "TelloEduLab_student"
FORBIDDEN_NAMES = {"solutions", "instructor_notes", "expected_results", "grading", "research_private"}
FORBIDDEN_TOKENS = ("PRIVATE instructor", "INSTRUCTOR_ONLY", "SOLUTION_BEGIN")

def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    ignore = shutil.ignore_patterns(".git", "__pycache__", "*.pyc", "runs", "dist", *FORBIDDEN_NAMES)
    shutil.copytree(ROOT, OUT, ignore=ignore)
    bad=[]
    for p in OUT.rglob("*"):
        if p.is_dir() and p.name in FORBIDDEN_NAMES:
            bad.append(str(p))
        elif p.is_file() and p.name != "build_student_release.py" and p.suffix.lower() in {".py",".md",".txt",".json",".yaml",".yml"}:
            text=p.read_text(encoding="utf-8", errors="ignore")
            if any(t in text for t in FORBIDDEN_TOKENS):
                bad.append(str(p))
    if bad:
        shutil.rmtree(OUT)
        raise SystemExit("Release rejected; instructor-only content detected:\n" + "\n".join(bad))
    print(f"Student release created: {OUT}")

if __name__ == "__main__":
    main()
