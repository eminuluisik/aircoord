"""Run the nine documented MPC conditions with explicit output isolation."""
import argparse
import subprocess
import sys
from pathlib import Path

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--out', type=Path, default=Path('results/grid'))
parser.add_argument('--dry-run', action='store_true', help='Print commands without executing simulations')
args = parser.parse_args()
root = Path(__file__).resolve().parents[1]
for delay in (0.5, 1.0, 2.0):
    for mode in ('current', 'stale', 'compensated'):
        dest = args.out / f'delay_{delay:g}_{mode}'
        if dest.exists() and any(dest.iterdir()):
            raise SystemExit(f'Output is not empty: {dest}. Choose a new --out root.')
        command = [sys.executable, str(root / 'aircoord.py'), '--controller', 'mpc',
                   '--agents', '4', '--seed', '7', '--episodes', '10',
                   '--duration', '60', '--delay', str(delay), '--accel-noise-std', '0.3',
                   '--horizon', '5', '--blocks', '5', '--margin', '3', '--maxiter', '80',
                   '--initialization', 'warm', '--state-source', mode, '--out', str(dest)]
        print('Running:', ' '.join(command), flush=True)
        if not args.dry_run:
            subprocess.run(command, check=True)
