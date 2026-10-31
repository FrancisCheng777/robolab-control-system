"""Inspect one real trial after control has stopped; never starts hardware."""
import argparse
import json
from pathlib import Path
import sys
import subprocess

import h5py
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pi05_demo.episode import inspect_episode
from pi05_demo.timing import audit_timing

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--input", required=True, type=Path)
parser.add_argument("--report", required=True, type=Path)
args = parser.parse_args()
args.report.mkdir(parents=True, exist_ok=True)
files = sorted(args.input.glob("*.h5"))
if len(files) != 1:
    raise SystemExit(f"未得到一条完整 HDF5，实际 {len(files)} 条；请查看本轮 collection.log。")
result = inspect_episode(files[0], allow_mock=False)
result["training_demonstration"] = False
result["d405_mounted"] = False
result["timing_passed"] = False
with h5py.File(files[0], "r") as episode:
    try:
        result["timing"] = audit_timing(episode, 15,allow_resampling=episode.attrs.get("schema")=="pi05_plug_demo/2")
        result["timing_passed"] = True
    except (ValueError, RuntimeError) as exc:
        result["timing_error"] = str(exc)
    result["cameras"] = {}
    if result["frames"]:
        for name in ("main", "wrist"):
            image = episode["images/" + name][result["frames"] // 2]
            Image.fromarray(image).save(args.report / (name + ".png"))
            ids = episode["camera_frame_id/" + name][:]
            result["cameras"][name] = {"shape": list(image.shape),
                "unique_device_frames": int(len(np.unique(ids)))}
(args.report / "episode_check.json").write_text(json.dumps(result, indent=2) + "\n")
print(f"已保存 {result['frames']} 帧，结果 {result['outcome']}。")
print("时序检查：" + ("通过" if result["timing_passed"] else "未通过，请查看报告"))
print("数据：" + str(files[0]))
print("检查报告：" + str(args.report / "episode_check.json"))
print("D405 尚未固定在末端：本条用于联调，暂不作为正式插接训练示范。")

if result["outcome"]=="success" and result["timing_passed"]:
    exported=args.report / "lerobot_checkout"
    attempt=subprocess.run([str(ROOT/"scripts/export.sh"),"--input",str(args.input),
        "--output",str(exported),"--checkout"],cwd=ROOT,stdout=(args.report/"export.log").open("w"),stderr=subprocess.STDOUT)
    result["export_passed"]=attempt.returncode==0
    result["export_path"]=str(exported)
    print("LeRobot 导出与官方读取："+("通过" if attempt.returncode==0 else "未通过，查看 export.log"))
    (args.report/"episode_check.json").write_text(json.dumps(result,indent=2)+"\n")
else:
    print("本条不是成功且时序合格的试动，保留原始文件，不导出训练动作。")
