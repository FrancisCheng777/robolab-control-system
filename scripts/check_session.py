"""Audit saved episodes after collection stops; never opens hardware or exports."""
import argparse
import json
from pathlib import Path
import sys
import h5py
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from pi05_demo.episode import inspect_episode
from pi05_demo.timing import audit_timing

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--input', type=Path, required=True)
parser.add_argument('--report', type=Path, required=True)
args = parser.parse_args()
results = []
for path in sorted(args.input.glob('*.h5')):
    item = {'path': str(path), 'passed': False}
    try:
        item.update(inspect_episode(path, allow_mock=False))
        with h5py.File(path, 'r') as episode:
            item['timing'] = audit_timing(episode, 15, allow_resampling=episode.attrs.get('schema') == 'pi05_plug_demo/2')
        item['passed'] = True
    except Exception as exc:
        item['error'] = str(exc)
    results.append(item)
args.report.parent.mkdir(parents=True, exist_ok=True)
args.report.write_text(json.dumps({'episodes': results}, indent=2) + '\n')
print(f'本次保存 {len(results)} 条，完整性与时序检查通过 {sum(x["passed"] for x in results)} 条。')
print('原始数据：' + str(args.input))
print('检查报告：' + str(args.report))
print('训练数据另行离线导出；D405 安装状态沿用本次配置记录。')
