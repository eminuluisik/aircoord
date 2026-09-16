"""Print a Markdown table directly from the supplied experiment records."""
import json
from pathlib import Path

root = Path(__file__).resolve().parents[1]
groups = [json.loads(p.read_text(encoding='utf-8')) for p in (root / 'evidence').glob('*.json')]
order = ['current', 'stale', 'compensated']
groups.sort(key=lambda g: (g['config']['delay_s'], order.index(g['mpc_settings']['state_source'])))
print('| Delay setting (s) | State source | Episodes with violation | Arrival rate | Worst separation (m) | Failed solver calls |')
print('|---:|---|---:|---:|---:|---:|')
for group in groups:
    episodes = group['episodes']
    assert [e['seed'] for e in episodes] == list(range(7, 17))
    violations = sum(e['episode_violation'] for e in episodes)
    rate = sum(e['arrival_rate'] for e in episodes) / len(episodes)
    worst = min(e['minimum_separation_m'] for e in episodes)
    failures = sum(e['solver_failures'] for e in episodes)
    print(f"| {group['config']['delay_s']:.1f} | {group['mpc_settings']['state_source']} | "
          f'{violations}/{len(episodes)} | {rate:.1%} | {worst:.3f} | {failures} |')
