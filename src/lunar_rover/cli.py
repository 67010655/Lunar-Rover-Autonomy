"""CLI with auditable inputs, explicit trial outcomes and output checksums."""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

from . import __version__
from .planning import NoPathError, plan_p0
from .scenario import load_scenario
from .simulation import run_trial


def _write_json(path: Path, payload: dict) -> None:
    path.write_text(json.dumps(payload,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _source_state() -> dict:
    root = Path(__file__).resolve().parents[2]
    sources = sorted((root/'src').rglob('*.py'))
    digest = hashlib.sha256()
    for source in sources:
        digest.update(source.relative_to(root).as_posix().encode())
        digest.update(source.read_bytes())
    state = {'version': __version__, 'python_source_sha256': digest.hexdigest(), 'git_revision': None, 'git_dirty': None}
    try:
        opts = dict(cwd=root,capture_output=True,text=True,check=True,timeout=10)
        state['git_revision'] = subprocess.run(['git','rev-parse','HEAD'],**opts).stdout.strip()
        state['git_dirty'] = bool(subprocess.run(['git','status','--porcelain'],**opts).stdout.strip())
    except (OSError,subprocess.SubprocessError):
        pass
    return state


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description='Synthetic P0 closed-loop navigation baseline (not SLAM)')
    parser.add_argument('--scenario',type=Path,required=True)
    parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--localization',choices=('ideal','odometry','fixes'),default='ideal')
    parser.add_argument('--output',type=Path,required=True,help='New or empty output directory')
    parser.add_argument('--animate',action='store_true',help='Save time-compressed GIF preview')
    args = parser.parse_args(argv)
    try:
        scenario = load_scenario(args.scenario)
        if args.output.exists() and any(args.output.iterdir()):
            raise ValueError('Output directory is non-empty; choose a new path to preserve previous runs')
        args.output.mkdir(parents=True,exist_ok=True)
        _write_json(args.output/'scenario.json',scenario.raw)
        manifest = {'schema_version': 1,'created_utc': datetime.now(timezone.utc).isoformat(),
                    'scenario_id': scenario.scenario_id,'data_type': 'synthetic',
                    'coordinate_frame': 'map_local_m','cell_convention': 'row increases +y; cell centres; rectangles inclusive',
                    'units': {'distance':'m','time':'s','angle':'rad'},
                    'planner':'P0','localization_model': args.localization,'seed':args.seed,
                    'scenario_sha256': _sha256(args.output/'scenario.json'),'source':_source_state(),
                    'python':platform.python_version(),'platform':platform.platform(),
                    'dependencies':{name:importlib.metadata.version(name) for name in ('matplotlib','numpy','Pillow')},
                    'arguments': {'scenario':str(args.scenario),'seed':args.seed,'localization':args.localization,'animate':args.animate},
                    'limitations':['Synthetic development map; no lunar data or measured radio link',
                                   'Planar body abstraction; not 6WD/4WS kinematics or rocker-bogie physics',
                                   'Command-based odometry error model; no SLAM, IMU fusion, or full GNSS model',
                                   'Known initial pose and prior obstacle map; no global replanning',
                                   'Position fixes, if enabled, use an ideal scheduled sensor with Gaussian noise only']}
        start_time = time.perf_counter()
        try:
            plan = plan_p0(scenario.grid,scenario.start,scenario.goal,
                           scenario.robot.body_radius_m+scenario.robot.planning_margin_m)
        except NoPathError as exc:
            _write_json(args.output/'metrics.json',{'status':'no_path','success':False,'reason':str(exc),
                                                 'planning_time_s':time.perf_counter()-start_time})
            manifest['status'] = 'no_path'
            manifest['output_sha256'] = {p.name:_sha256(p) for p in args.output.iterdir() if p.is_file()}
            _write_json(args.output/'manifest.json',manifest)
            print(f'No route: {exc}',file=sys.stderr)
            return 2
        planning_time = time.perf_counter()-start_time
        trial = run_trial(scenario,plan,args.seed,args.localization)
        trial.metrics.update(planning_time_s=planning_time,expanded_nodes=plan.expanded_nodes)
        _write_json(args.output/'metrics.json',trial.metrics)
        with (args.output/'trajectory.csv').open('w',newline='',encoding='utf-8') as stream:
            writer = csv.DictWriter(stream,fieldnames=list(trial.samples[0]))
            writer.writeheader()
            writer.writerows(trial.samples)
        with (args.output/'planned_route.csv').open('w',newline='',encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(('row','column','x_m','y_m'))
            for cell in plan.cells:
                writer.writerow((*cell,*scenario.grid.to_world(cell)))
        # Save raw data before rendering so an interrupted plot does not erase a run.
        manifest['status'] = trial.metrics['status']
        manifest['output_sha256'] = {p.name:_sha256(p) for p in args.output.iterdir() if p.is_file()}
        _write_json(args.output/'manifest.json',manifest)
        from .visualization import render_outputs
        render_outputs(scenario,plan,trial,args.output,args.animate)
        manifest['output_sha256'] = {p.name:_sha256(p) for p in args.output.iterdir() if p.is_file() and p.name != 'manifest.json'}
        _write_json(args.output/'manifest.json',manifest)
        print(json.dumps(trial.metrics,indent=2,allow_nan=False))
        print(f'Artifacts: {args.output.resolve()}')
        return 0 if trial.metrics['success'] else 2
    except (ValueError,OSError,json.JSONDecodeError,importlib.metadata.PackageNotFoundError) as exc:
        print(f'Error: {exc}',file=sys.stderr)
        return 1
