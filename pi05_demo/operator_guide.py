"""Terminal guidance only; takeover decisions remain in CollectionSession."""
import json
import math
from pathlib import Path


def format_alignment(data):
    if data.get("robolab"):
        ready=bool(data.get("ready"))
        errors=data.get("joint_error_degrees")
        text=("回位完成，当前待机：尚未跟随、尚未录制。按 Enter 才开始。" if ready else
              "暂不能开始："+data.get("detail", "等待有效读数与停止确认"))
        text+="\n无需七轴精确对齐；保持 GELLO 大致与机械臂同向，启动后 Franka 会向 GELLO 靠近。"
        if errors is not None:
            text+="\n关节差值（仅供参考，度）："+", ".join(f"J{i+1} {x:+.1f}" for i,x in enumerate(errors))
        text+="\n差值很大时请先粗调 GELLO，避免大幅追赶。第七轴已自动映射。"
        text+="\n按 Enter 开始跟随后夹爪才响应 GELLO；当前保持原开度。"
        text+="\nEnter 停止 → s 正常 / f 失败 / a 放弃；q 退出。"
        return text
    errors = data.get("joint_error_degrees")
    width_error = data.get("gripper_error_mm")
    if errors is None or width_error is None:
        return "等待有效 GELLO 读数；尚未对齐。\n" + data.get("detail", "")
    ready = bool(data.get("ready"))
    reference = bool(data.get("reference_only"))
    if reference:
        heading = ("预摆位完成：按 Enter 启动试采准备，之后还需回位与实测对齐。" if ready else
                   "预摆位：按表格调整 GELLO；此时尚未启用 Franka 控制。")
    else:
        heading = (
            "可以开始：按 Enter 开始跟随并录制。"
            if ready else
            "尚未对齐：现在只调整 GELLO，Franka 保持不动。"
        )
    lines = [heading,
             "J1 从 GELLO 底座数起，J7 在末端。角度统一按 Franka 坐标显示。",
             "关节   GELLO目标°  Franka参考°   GELLO还需调°   状态" if reference else
             "关节   GELLO目标°  Franka当前°   GELLO还需调°   状态"]
    leader = data.get("leader_joint_degrees")
    robot = data.get("robot_joint_degrees")
    for i, error in enumerate(errors):
        actual = "    --" if leader is None else f"{leader[i]:8.2f}"
        target = "    --" if robot is None else f"{robot[i]:8.2f}"
        status = "OK" if abs(error) <= math.degrees(.03) else "调整"
        lines.append(f"J{i+1}    {actual}     {target}       {-error:+8.2f}     {status}")
    status = "OK" if abs(width_error) <= 5 else "调整"
    lines.append(f"夹爪：还需调整 {-width_error:+.1f} mm（正数张开，负数闭合）  {status}")
    if data.get("leader_width_mm") is not None:
        lines.append(f"当前 GELLO 开度 {data['leader_width_mm']:.1f} mm；Franka 开度 {data['robot_width_mm']:.1f} mm")
    if not ready:
        worst = max(range(7), key=lambda i: abs(errors[i]))
        if abs(errors[worst]) > math.degrees(.03):
            lines.append(f"先调 J{worst+1}：轻动该轴，观察“还需调”的绝对值是否变小。")
        lines.append("预摆位需同时满足：七轴各在 ±1.72° 内、夹爪在 ±5 mm 内。" if reference else
                     "需同时满足：七轴各在 ±1.72° 内、夹爪在 ±5 mm 内，并且机械臂与夹爪已停止。")
        if data.get("detail"):
            lines.append("未就绪原因：" + data["detail"])
    lines.append("本屏是默认姿态的预摆位参考，不表示已经接管机械臂；q 退出。" if reference else
                 "录制后：Enter 停止 → s 成功 / f 失败 / a 仅试动；q 退出。")
    return "\n".join(lines)


class OperatorGuide:
    def __init__(self, event_log=None):
        self.log = None
        if event_log:
            path = Path(event_log)
            path.parent.mkdir(parents=True, exist_ok=True)
            self.log = path.open("a", encoding="utf-8")

    def __call__(self, event, **data):
        if self.log:
            self.log.write(json.dumps({"event": event, **data}, ensure_ascii=False) + "\n")
            self.log.flush()
        if event == 'recording' and data.get('teleop_only'):
            print('\n正在遥操作：相机关闭、不录数据。Enter 停止并结束本轮；q 退出。', flush=True)
            return
        if event == 'teleop_finished':
            print(f"\n本轮遥操作结束，共 {data['updates']} 次控制更新；没有生成训练数据。", flush=True)
            return
        if event == "home_progress":
            print(f"回位检查：最大角度误差 {math.degrees(data['max_error_rad']):.2f}° / "
                  f"允许 {math.degrees(data['tolerance_rad']):.2f}°；"
                  f"最大关节速度 {data['max_speed_rad_s']:.4f} rad/s（需 ≤0.01）",flush=True)
            return
        if event == "alignment":
            large_difference = data.get("robolab") and any(abs(x)>45 for x in (data.get("joint_error_degrees") or []))
            color = "\033[32m" if data.get("ready") and not large_difference else "\033[33m"
            print("\033[2J\033[H" + color + format_alignment(data) + "\033[0m", flush=True)
            return
        messages = {
            "init": ("设备准备完成，等待停止确认后自动回位并开爪；无需再按 Enter，尚未录制。q 可退出。"
                     if data.get("auto_home") else "准备完成。按 Enter：Franka 回初始位；此时尚未录制。"),
            "homing": ("正在回初始位；到位后自动张开夹爪，再进入待机。此时尚未录制；q 可退出。"
                       if data.get("open_gripper") else "正在回初始位；夹爪保持当前开度。q 可退出。"),
            "opening_gripper": "机械臂已到位，正在自动张开夹爪；请等待完成。q 退出，a 取消。",
            "settling": "正在减速并保持，请等待。",
            "standby": "待机：检查 GELLO 方向与周围空间；显示“可以开始”后按 Enter。",
            "recording": "正在跟随并录制！按 Enter 停止；或直接按 s/f/a 停止并标记。",
            "label": "已停止跟随。请按 s=任务成功 / f=任务失败 / a=仅试动或放弃。",
            "waiting_for_hold": "还在停止，请稍后再按 Enter。",
        }
        if event == "saved":
            message = f"已保存 {data['frames']} 帧，标记 {data['outcome']}：\n{data['episode']}"
        else:
            message = messages.get(event, f"{event}: {data}")
        print("\n" + message, flush=True)

    def close(self):
        if self.log:
            self.log.close()
