#!/usr/bin/env python3
"""SIM-only course batch runner. Never initiates REAL flights.

Exit status 0 only if every executed example passes and none fail/timeout.
SKIP means not executed, not verified. PASS means subprocess exit code 0,
not physical correctness of the trajectory.
"""
from __future__ import annotations
import argparse
import ast
import csv
from datetime import datetime
import html
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / 'docs' / 'COURSE_ORDER.json'


def preflight():
    return {module: importlib.util.find_spec(module) is not None for module in ('pybullet', 'numpy', 'matplotlib', 'cv2')}


def write_report(folder, results, environment, arguments):
    folder.mkdir(parents=True, exist_ok=True)
    summary = {key: sum(r['status'] == key for r in results) for key in ('PASS', 'FAIL', 'TIMEOUT', 'SKIP')}
    payload = {'summary': summary, 'preflight': environment, 'arguments': vars(arguments), 'results': results,
               'meaning': {'PASS': 'Process returned exit code 0; flight quality not validated',
                           'SKIP': 'Not executed; no evidence of correctness'}}
    (folder / 'report.json').write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding='utf-8')
    with (folder / 'report.csv').open('w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=['index', 'stage', 'name', 'category', 'status', 'seconds', 'reason', 'run_dir'])
        writer.writeheader(); writer.writerows(results)
    trs = []
    for r in results:
        cells = ''.join(f'<td>{html.escape(str(r.get(key, "")))}</td>' for key in ('index','stage','name','category','status','seconds','reason'))
        link = f'<a href="{html.escape(os.path.relpath(r["run_dir"], folder))}">logs</a>' if r['run_dir'] else ''
        trs.append(f'<tr class="{r["status"].lower()}">{cells}<td>{link}</td></tr>')
    page = '''<!doctype html><html lang="en"><meta charset="utf-8"><title>TelloEduLab test report</title>
<style>body{font:15px system-ui;margin:3rem;max-width:1400px}table{border-collapse:collapse;width:100%}th,td{padding:.55rem;border:1px solid #ddd;text-align:left}th{background:#eee}.pass{background:#e5f6e7}.fail,.timeout{background:#ffe0e0}.skip{background:#fff4dc}code{background:#eee;padding:.2rem}</style>
<h1>TelloEduLab — SIM test report</h1><p>PASS = process exited successfully, <strong>not</strong> proof of flight accuracy. SKIP = not executed.</p>'''
    page += '<p>' + ' · '.join(f'{key}: <strong>{value}</strong>' for key,value in summary.items()) + '</p>'
    page += '<p>Dependencies: ' + html.escape(str(environment)) + '</p>'
    page += '<table><thead><tr>' + ''.join(f'<th>{x}</th>' for x in ('#','Stage','Example','Category','Status','Time (s)','Details','Artifacts')) + '</tr></thead><tbody>' + ''.join(trs) + '</tbody></table></html>'
    (folder / 'report.html').write_text(page, encoding='utf-8')
    return summary


def main():
    p = argparse.ArgumentParser(description='Run course examples in isolated SIM subprocesses and generate HTML/CSV/JSON reports.')
    p.add_argument('--mode', choices=['sim'], default='sim', help='REAL batch flights intentionally unsupported')
    p.add_argument('--timeout', type=int, default=75, help='Seconds allowed for each example')
    p.add_argument('--start', type=int, default=1, help='First course index')
    p.add_argument('--end', type=int, default=49, help='Last course index')
    p.add_argument('--stage', type=int, default=None, help='Run only one course stage')
    p.add_argument('--dry-run', action='store_true', help='List examples without executing flights')
    p.add_argument('--require-pybullet', action='store_true', help='Fail before testing when PyBullet is missing')
    args = p.parse_args()
    if args.timeout < 1 or args.start < 1 or args.end < args.start:
        p.error('Invalid timeout or index range')
    entries = json.loads(MANIFEST.read_text(encoding='utf-8'))
    chosen = [r for r in entries if args.start <= r['index'] <= args.end and (args.stage is None or r['stage'] == args.stage)]
    env = preflight()
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    folder = ROOT / 'runs' / f'batch_{timestamp}_{os.getpid()}'
    results = []
    if args.require_pybullet and not env['pybullet']:
        print('ERROR: PyBullet is missing. Install it in the selected PyCharm interpreter: python -m pip install pybullet', file=sys.stderr)
        return 2
    print('TELLOEDULAB AUTOMATED SIM TEST REPORT')
    print('Preflight:', env)
    for item in chosen:
        original = ROOT / item['original']
        status, reason, run_dir, elapsed = 'SKIP', '', '', 0.0
        if not original.is_file():
            status, reason = 'FAIL', 'Source file missing'
        else:
            try:
                source = original.read_text(encoding='utf-8')
                ast.parse(source, filename=str(original))
            except (OSError, SyntaxError) as exc:
                status, reason = 'FAIL', f'Syntax/source error: {exc}'
            else:
                import re
                if not re.search(r'(?m)^MODE\s*=\s*[\'\"](?:sim|real)[\'\"]', source):
                    reason = 'No canonical MODE variable: migration needed'
                elif args.dry_run:
                    reason = 'Dry run: not executed'
                elif not env['pybullet']:
                    reason = 'PyBullet not installed; SIM cannot run'
                else:
                    launcher = ROOT / item['launcher']
                    started = time.monotonic()
                    try:
                        proc = subprocess.run([sys.executable, str(launcher), '--mode', 'sim', '--timeout', str(args.timeout)],
                                              cwd=ROOT, capture_output=True, text=True, timeout=args.timeout + 15,
                                              env={**os.environ, 'PYTHONUNBUFFERED': '1'})
                        elapsed = round(time.monotonic() - started, 2)
                        status = 'PASS' if proc.returncode == 0 else 'FAIL'
                        reason = f'Exit code {proc.returncode}'
                        for line in reversed(proc.stdout.splitlines()):
                            if '"output":' in line:
                                try: run_dir = json.loads(line.split('"output":', 1)[1].strip().rstrip(',').strip())
                                except (ValueError, IndexError): pass
                                break
                        if not run_dir:
                            # Save runner-level logs when the launcher fails before creating its own directory.
                            log_dir = folder / f'item_{item["index"]:02d}'
                            log_dir.mkdir(parents=True, exist_ok=True)
                            (log_dir / 'stdout.log').write_text(proc.stdout, encoding='utf-8')
                            (log_dir / 'stderr.log').write_text(proc.stderr, encoding='utf-8')
                            run_dir = str(log_dir)
                    except subprocess.TimeoutExpired:
                        elapsed = round(time.monotonic() - started, 2)
                        status, reason = 'TIMEOUT', f'Exceeded {args.timeout + 15}s (including launcher overhead)'
        result = {'index': item['index'], 'stage': item['stage'], 'name': Path(item['original']).stem,
                  'category': item['category'], 'status': status, 'seconds': elapsed, 'reason': reason, 'run_dir': run_dir}
        results.append(result)
        print(f'{item["index"]:02d} {result["name"][:35]:35s} {status:7s} {reason}')
        write_report(folder, results, env, args)  # Incremental report survives interruptions.
    summary = write_report(folder, results, env, args)
    print('\nSummary:', summary)
    print('HTML report:', folder / 'report.html')
    if summary['FAIL'] or summary['TIMEOUT']:
        return 1
    if not args.dry_run and summary['PASS'] == 0:
        print('WARNING: No example was executed successfully; this is not a passing integration test.')
        return 2
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
