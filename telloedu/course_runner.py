"""Safe, reproducible per-example execution with plot capture and logs."""
import argparse, datetime, json, os, pathlib, subprocess, sys, time
ROOT=pathlib.Path(__file__).resolve().parents[1]
def run_example(relative_path):
    ap=argparse.ArgumentParser()
    ap.add_argument('--mode',choices=('sim','real'),default='sim')
    ap.add_argument('--allow-real',action='store_true')
    ap.add_argument('--timeout',type=int,default=180)
    a=ap.parse_args()
    if a.mode=='real' and not a.allow_real:
        ap.error('REAL is disabled by default; inspect the source and add --allow-real explicitly.')
    source=ROOT/relative_path
    code=source.read_text(encoding='utf-8')
    import re
    # Restrict substitution to the canonical top-level MODE assignment.
    code,n=re.subn(r'(?m)^(MODE\s*=\s*)[\"\'](?:sim|real)[\"\']',lambda m:m.group(1)+repr(a.mode),code,count=1)
    if n!=1:
        print('SKIP: no canonical MODE variable; manual migration required:',relative_path)
        return
    if a.mode=='real' and ('get_position(' in code or 'get_front_distance(' in code or 'World(' in code):
        ap.error('REAL blocked: source contains SIM-only ground truth, raycasting or world setup. Use an explicit estimator/sensor adapter.')
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    out=ROOT/'runs'/f'{stamp}_{source.stem}_{a.mode}';out.mkdir(parents=True,exist_ok=True)
    (out/'configuration.json').write_text(json.dumps({'source':relative_path,'mode':a.mode,'timeout_s':a.timeout},indent=2))
    # Separate subprocess protects the runner from GUI errors and hangs.
    # Matplotlib show() is patched to save all figures automatically.
    bootstrap="import os,sys,runpy,matplotlib\nmatplotlib.use('Agg')\nimport matplotlib.pyplot as plt\nfrom pathlib import Path\noutput=Path(os.environ['TELLOEDU_RUN_DIR'])\ndef save_figures(*args,**kwargs):\n    for num in plt.get_fignums():\n        plt.figure(num).savefig(output/f'figure_{num:02d}.png',dpi=160,bbox_inches='tight')\n    plt.close('all')\nplt.show=save_figures\nrunpy.run_path(os.environ['TELLOEDU_SCRIPT'],run_name='__main__')\nsave_figures()\n"
    script=out/'_executed_example.py';script.write_text(code)
    env=os.environ.copy();env['TELLOEDU_RUN_DIR']=str(out);env['TELLOEDU_SCRIPT']=str(script);env['PYTHONPATH']=str(ROOT)+os.pathsep+env.get('PYTHONPATH','')
    start=time.monotonic();status='PASS'
    try:
        p=subprocess.run([sys.executable,'-c',bootstrap],cwd=str(ROOT),env=env,capture_output=True,text=True,timeout=a.timeout)
        (out/'stdout.log').write_text(p.stdout);(out/'stderr.log').write_text(p.stderr)
        if p.returncode:status='FAIL'
    except subprocess.TimeoutExpired as e:
        status='TIMEOUT';(out/'stdout.log').write_bytes(e.stdout or b'');(out/'stderr.log').write_bytes(e.stderr or b'')
    result={'status':status,'elapsed_s':round(time.monotonic()-start,2),'source':relative_path,'mode':a.mode,'output':str(out)}
    (out/'results.json').write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
    if status!='PASS':sys.exit(1)
