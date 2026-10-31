#!/usr/bin/env bash
set -euo pipefail
TRIAL_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$TRIAL_ROOT"
TRIAL_CONTINUOUS=0
case "${1:-}" in
  --continuous) TRIAL_CONTINUOUS=1; shift ;;
  "") ;;
  *) echo "用法：$0 [--continuous]"; exit 2 ;;
esac
[[ $# == 0 ]] || { echo '不支持额外参数'; exit 2; }
TRIAL_CONFIG_DIR="${PI05_TRIAL_CONFIG_DIR:-$TRIAL_ROOT/reports/robolab_restore_20261030}"
TRIAL_SESSION="trial_$(TZ=Asia/Taipei date +%Y%m%d_%H%M%S)"
TRIAL_RUN="${PI05_TRIAL_RUN_ROOT:-$TRIAL_CONFIG_DIR}/$TRIAL_SESSION"
TRIAL_DATA="${PI05_TRIAL_DATA_ROOT:-$TRIAL_ROOT/data/trial_raw_20261030}/$TRIAL_SESSION"
if [[ "$TRIAL_CONTINUOUS" == 1 ]]; then
  TRIAL_SESSION="collection_$(TZ=Asia/Taipei date +%Y%m%d_%H%M%S)"
  TRIAL_RUN="$TRIAL_ROOT/reports/collection/$TRIAL_SESSION"
  TRIAL_DATA="$TRIAL_ROOT/data/raw/$TRIAL_SESSION"
fi
TRIAL_SOCKET="${PI05_TRIAL_SOCKET:-/tmp/pi05-robolab-trial-20261030.sock}"
exec 9>/tmp/pi05-robolab-trial-20261030.lock
flock -n 9 || { echo '已有采集在运行，请使用原窗口。'; exit 1; }
mkdir -p "$TRIAL_RUN" "$TRIAL_DATA"
printf '%s\n' "$TRIAL_RUN" > "${PI05_TRIAL_RUN_ROOT:-$TRIAL_CONFIG_DIR}/latest_run.txt"
export PYTHONUNBUFFERED=1
exec > >(tee -a "$TRIAL_RUN/collection.log") 2>&1
if [[ "$TRIAL_CONTINUOUS" == 1 ]]; then
  printf '%s\n' 'RoboLab 连续采集：不设录制时长和条数上限，由按键结束。' \
    'Enter 启动设备 → Enter 初次回位 → Enter 开始跟随并录制。' \
    'Enter 停止本条 → s 成功 / f 失败 / a 放弃；保存后待机，Enter 开始下一条。' \
    'q 或 Ctrl-C 退出整个采集；未标记的当前条保存为 aborted。' \
    '每条之间不自动回初始位。初始位、GELLO 映射及故障保护保持不变。' \
    '现在尚未连接 Franka 控制。按 Enter 继续，输入 q 后 Enter 退出。'
else
printf '%s\n' 'RoboLab 单条试采：GELLO 大致与机器人同向即可，不要求七轴逐一对齐。' \
  '准备好后 Enter 启动设备；看到准备完成再 Enter 回初始位，再 Enter 开始跟随并录制。' \
  '启动跟随后机器人会逐渐接近 GELLO。先做 10–15 秒小幅空中动作和夹爪开合。' \
  'Enter 停止，s=试动正常 / f=异常 / a=放弃；保存一条后自动检查和导出。' \
  '现在尚未连接 Franka 控制。按 Enter 继续，输入 q 后 Enter 退出。'
fi
read -r TRIAL_START
[[ "$TRIAL_START" != q ]] || exit 0
# Check scheduling before opening any device. This machine previously inherited nice=19.
.venv/bin/python - <<'PYRT'
import os
if os.getpriority(os.PRIO_PROCESS, 0) != 0:
    raise SystemExit('当前终端优先级异常（需要 nice=0）。系统修复后请关闭旧终端，使用新登录会话；尚未修复时运行 sudo python3 scripts/fix_realtime_priority.py --apply。')
if not {12, 13}.issubset(os.sched_getaffinity(0)):
    raise SystemExit('当前 CPU 亲和性不包含本机控制核 12；请在普通终端启动。')
import sys
sys.path.insert(0, 'scripts')
from fix_fci_network_priority import discover
network = discover()
if network['policy'] != os.SCHED_FIFO or network['priority'] != 50:
    raise SystemExit('机器人网卡实时接收调度未应用（重启后会失效）。请先执行 sudo python3 scripts/fix_fci_network_priority.py --apply，再启动采集。')
PYRT
.venv/bin/python - <<'PY'
from pathlib import Path
import json, os
from pi05_demo.gripper_teleop import unowned
root = Path.cwd()
config_root = Path(os.environ.get('PI05_TRIAL_CONFIG_DIR', str(root/'reports/robolab_restore_20261030')))
c = json.loads((config_root/'collection.trial.json').read_text())
b = json.loads((config_root/'bridge.trial.json').read_text())
if not c['hardware_verified'] or not b['hardware_verified']:
    raise SystemExit('本轮试采配置尚未就绪。')
for port in (c['gello_port'], b['gripper_device']):
    unowned(port)
if Path(c['socket']).exists():
    raise SystemExit('试采 socket 已存在；请先检查原进程，程序不会自动清除。')
PY
.venv/bin/python scripts/check_desk_tool.py --config "$TRIAL_CONFIG_DIR/bridge.trial.json" --output "$TRIAL_RUN/desk-before.json" > "$TRIAL_RUN/desk-before.log" 2>&1
.venv/bin/python - "$TRIAL_RUN/desk-before.json" <<'PY'
import json, sys
d = json.load(open(sys.argv[1]))
state = d['state']
if not d['matches_expected_tool']:
    raise SystemExit('Desk 工具配置不匹配：' + ', '.join(d['mismatched_fields']))
if state['robot_mode'] != 'Idle' or state['current_errors']:
    print('Franka 状态：' + str(state['robot_mode']), flush=True)
    print('当前故障：' + str(state['current_errors']), flush=True)
    print('上次运动故障：' + str(state.get('last_motion_errors', [])), flush=True)
    print('实际关节位置（rad）：' + str(state.get('q')), flush=True)
    errors = state['current_errors'] + state.get('last_motion_errors', [])
    if 'joint_position_limits_violation' in errors:
        print('检测到关节限位故障记录。GELLO 摆正不会自动解除 Franka 的 Reflex。', flush=True)
        print('请在 Desk 按关节限位恢复提示操作，恢复为无故障 Idle 后重新运行。', flush=True)
    else:
        print('请先在 Desk 检查并恢复机器人状态；本脚本不会自动清错或启动运动。', flush=True)
    raise SystemExit('试采尚未启动。状态报告：' + sys.argv[1])
PY
TRIAL_BRIDGE_PID=
trial_cleanup() {
  if [[ -n "$TRIAL_BRIDGE_PID" ]]; then
    kill -TERM "$TRIAL_BRIDGE_PID" 2>/dev/null || true
    wait "$TRIAL_BRIDGE_PID" || true
    TRIAL_BRIDGE_PID=
  fi
}
trap trial_cleanup EXIT
# ROS shell libraries can shadow the pinned Ruckig build. Use the binary RUNPATH.
env -u LD_LIBRARY_PATH PI05_CONTROL_CPU=12 taskset -c 0-11,13-19 "$TRIAL_ROOT/build/pi05_franka_bridge" --config "$TRIAL_CONFIG_DIR/bridge.trial.json" \
  --robot 172.16.0.40 --socket "$TRIAL_SOCKET" --fault-log "$TRIAL_RUN/franka_fault.csv" > "$TRIAL_RUN/bridge.log" 2>&1 &
TRIAL_BRIDGE_PID=$!
printf '%s\n' "$TRIAL_BRIDGE_PID" > "$TRIAL_RUN/bridge.pid"
if ! .venv/bin/python - "$TRIAL_SOCKET" <<'PY'
import sys, time
from pi05_demo.transport import Bridge
deadline = time.monotonic() + 10
error = None
while time.monotonic() < deadline:
    try:
        bridge = Bridge(sys.argv[1], mock=False)
        print('机器人通信就绪，正在启动双相机和 GELLO。', flush=True)
        break
    except (OSError, RuntimeError) as exc:
        error = exc
        time.sleep(.05)
else:
    raise SystemExit(f'机器人通信未就绪：{error}；请查看本轮 bridge.log。')
PY
then
  echo "桥接程序启动失败，具体原因如下（$TRIAL_RUN/bridge.log）："
  tail -n 20 "$TRIAL_RUN/bridge.log"
  exit 1
fi
TRIAL_EPISODE_ARGS=(--one-episode)
TRIAL_COLLECTION_CONFIG="$TRIAL_CONFIG_DIR/collection.trial.json"
if [[ "$TRIAL_CONTINUOUS" == 1 ]]; then
  TRIAL_EPISODE_ARGS=()
  TRIAL_COLLECTION_CONFIG="$TRIAL_RUN/collection.json"
  .venv/bin/python - "$TRIAL_COLLECTION_CONFIG" <<'PYCONFIG'
import json, sys
from pathlib import Path
config = json.loads(Path('reports/robolab_restore_20261030/collection.trial.json').read_text())
config['task'] = json.loads(Path('configs/collection.robolab.json').read_text())['task']
Path(sys.argv[1]).write_text(json.dumps(config, indent=2) + '\n')
PYCONFIG
fi
set +e
taskset -c 0-11,13-19 ./scripts/collect.sh --execute --config "$TRIAL_COLLECTION_CONFIG" \
  --session "$TRIAL_SESSION" --output "$TRIAL_DATA" --guide "${TRIAL_EPISODE_ARGS[@]}" \
  --event-log "$TRIAL_RUN/events.jsonl"
TRIAL_RESULT=$?
set -e
trial_cleanup
if [[ "$TRIAL_RESULT" != 0 ]]; then
  echo "本轮已停止，退出码 $TRIAL_RESULT。日志：$TRIAL_RUN"
  if [[ -s "$TRIAL_RUN/bridge.log" ]]; then
    echo "控制桥接末尾日志（用于区分 FCI 原始故障与反馈超时）："
    tail -n 12 "$TRIAL_RUN/bridge.log"
  fi
fi
if compgen -G "$TRIAL_DATA/*.h5" >/dev/null; then
  if [[ "$TRIAL_CONTINUOUS" == 1 ]]; then
    .venv/bin/python scripts/check_session.py --input "$TRIAL_DATA" --report "$TRIAL_RUN/session_check.json"
  else
    if [[ "${PI05_LATENCY_TRIAL:-0}" == 1 ]]; then
      .venv/bin/python scripts/check_session.py --input "$TRIAL_DATA" --report "$TRIAL_RUN/session_check.json"
    else
      .venv/bin/python scripts/check_trial.py --input "$TRIAL_DATA" --report "$TRIAL_RUN"
    fi
  fi
else
  echo '本轮尚未产生完整 episode。'
fi
exit "$TRIAL_RESULT"
