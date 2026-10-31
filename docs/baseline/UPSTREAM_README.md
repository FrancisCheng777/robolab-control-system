# π0.5 插排 Demo：独立采集与 Stage-1 全量微调

## 当前末端工具（2026-10-31）

实机已安装原厂 Robotiq 2F-85 指尖、D405 与 R2 PETG 支架，无黄色 TPU 指尖。当前配置为 `configs/tools/robotiq_2f85_d405/endeffector-config.json`：质量 1.0098550544536928 kg，法兰系重心 [0, -0.005963192692566201, 0.060715106723257514] m，完整惯量见该 JSON，TCP 为 [0, 0, 0.174] m。Desk 已应用并经 FCI 只读验证。这是工程估算模型，不是负载辨识或手眼标定。

常规入口使用 `configs/bridge.real.json`，RoboLab 模板使用 `configs/bridge.robolab.json`，当前单条/连续采集入口 `scripts/try_one.sh` / `scripts/collect_session.sh` 使用 `reports/robolab_restore_20261030/bridge.trial.json`；三者的 `expected_desk_tool` 均为同一新模型。试采预检使用实际启动的 trial 配置进行读回核对。空爪 `load_mass/load_center/load_inertia` 仍为零，它们只表示额外抓持物，不能再次计入已在 Desk 中配置的工具质量。

旧无相机目录、历史记录及备份保留用于追溯；下文 2026-10-29/30 的无相机描述仅代表当时状态。应用记录见 `reports/d405_desk_apply_20261031_1523/`，控制链路核查与本次修改备份见 `reports/d405_control_chain_20261031/`。



## 连续采集入口（无录制时长、条数上限）

在 4090 终端执行 `cd ~/pi05_plug_demo && ./scripts/collect_session.sh`。
Enter 启动设备 → Enter 首次回位（到位停稳后自动张开夹爪）→ Enter 开始跟随并录制。
Enter 停止本条，再按 s=成功 / f=失败 / a=放弃；也可直接 s/f/a 停止并标记。
保存后保持待机，Enter 开始下一条，不会每条自动回位。q 或 Ctrl-C 退出整个采集；未标记的当前条保留为 aborted。
录制没有人为时长或条数上限，通信看门狗、设备故障与回位超时保护继续生效。
回位参考峰值速度现为 0.20 rad/s（最初 0.05），桥接回位限速 0.20 rad/s；参考轨迹加速度 0.10 rad/s²、jerk 0.5 rad/s³ 保持不变。长距离回位的参考时长约为最初的 1/4，短距离及最终稳定时间不保证同比缩短；遥操作速度和碰撞阈值未变。

原始数据在 4090 的 `data/raw/collection_<时间>/`，日志与退出后的逐条检查报告在 `reports/collection/collection_<时间>/`。退出后不自动导出训练集，按下文的离线导出流程处理。
沿用已验证的试采硬件配置、控制速度、初始位与绝对映射，任务文本取自 `configs/collection.robolab.json`。
D405 尚未固定在末端，因此保留原来的相机安装/联调标记；连续采集功能不代表相机安装已经验收。
原 `./scripts/try_one.sh` 仍为保存一条就退出的检查入口。

## 当前推荐入口：RoboLab 遥操作（2026-10-30）

在 **4090 本机终端**（ToDesk 中）运行：

```bash
cd ~/pi05_plug_demo
./scripts/try_one.sh
```

1. 大致把 GELLO 摆成与 Franka 同向、相近的姿态；不要求七个轴逐一满足 ±1.72°。先保持不动，按 Enter 启动设备。
2. 看到“准备完成”后按 Enter，Franka 低速回到已确认的初始姿态 `[0,-36,0,-144,0,108,0]°`。 回位完成采用原 RoboLab 的逐轴 0.05rad（约 2.86°）容差，并要求关节速度不超过 0.01rad/s；这不是 GELLO 接管对齐条件。回位途中夹爪保持原开度，到位停稳后自动张开；开度反馈到位后进入待机。录制时开始响应 GELLO。
3. 等待“回位完成，当前待机”。终端的角度差值仅供参考；确认追赶路径空闲，按 Enter 开始渐进跟随并录制。GELLO 与 Franka 差得越远，启动后的追赶幅度越大。不要把“不要求精确对齐”理解为任意放置都不会产生动作。
4. 做 10–15 秒小幅空中动作与夹爪开合。按 Enter 停止，随后 s=试动正常、f=失败、a=放弃。保存一条后自动关闭本轮控制，并检查 HDF5、时间戳与 LeRobot 导出/官方读取。q 或 Ctrl-C 退出；不会自动恢复故障。

数据保存在 4090 的 `data/trial_raw_20261030/<session>/`，日志、相机截图、检查结果和 LeRobot 检查导出在 `reports/robolab_restore_20261030/<session>/`。`latest_run.txt` 指向最新运行。D405 目前在旁边，Desk 继续使用仅含 2F-85 的工具配置；这批为联调数据，不能充当正式腕部相机示范。试采导出带 `checkout_only=true`，SFT adapter 会拒绝将它作为训练集。

机械臂使用移植自 RoboLab 的绝对关节目标、可选 IIR、每周期限幅及力矩 PD（Kp `[120,120,120,120,120,150,60]`，Kd 全部 20）。原版每周期上限 0.0005 rad，本次试采进一步按现有速度配置限制到 **0.0002 rad/周期**（理想 1kHz 下 0.2rad/s）；回位上限更低。不采用动态相对偏移。J7 保持固定减 π/4 的已确认映射。停止时取消追赶并保持参考位置。此路径不再使用 Ruckig 的加速度/jerk 限制，保留 libfranka 力矩限幅、关节位置检查、碰撞保护、250ms 命令看门狗和实测状态新鲜度检查。按原 RoboLab 力矩回调逻辑，每次收到反馈只前进一步，不重放漏掉的周期；RoboLab 力矩路径不套用位置控制路径的 0.05rad 内部参考跟踪误差中断；通信异常由 libfranka 的原生约束处理，不再额外套用位置控制器的 10ms 中断条件。网络抖动仍在记录，本次并未修复物理网络。

GELLO 越过机器人关节范围时，沿用原 RoboLab 发布器的关节目标边界限制；同时保存边界限制前的 `gello_unclipped_target` 与实际发送的 `q_target`，不掩盖原始输入。起始姿态、D405/Kinect 序列号、2F-85 开度端点均沿用已确认配置。

### 记录与训练动作

新版 HDF5 schema 为 `pi05_plug_demo/2`，完整保存每帧实际 q、GELLO 原始/发送目标、实测/命令开度、两路图像与原有时间戳。录制不再依赖 `|GELLO目标-q|<=0.03`。

离线导出参考 RoboLab `scripts/lerobot_tools/core.py`：先按真实时间戳建立 15Hz 网格，插值实测关节/开度、对命令开度做前值保持，并选取时间最近的相机帧；再用相邻网格实测位置生成训练动作：前七维为 `(q[t+1]-q[t])/0.03`，夹爪维为该网格时刻命令开度的原有归一化。这样训练的是机器人实际完成的运动，而不是尚未追上的远端目标。它不是原始 GELLO 目标动作的无损等价替换；使用该数据训练的模型应按既有 joint-delta adapter 解码，并在同类控制器下进行后续 rollout 验证。最后一个网格状态没有下一步，不生成动作行；所有原始帧保留在 HDF5 中；不填零、不重复末尾动作。采样抖动在报告中如实显示；超过 200ms 的观测缺口或超出模型范围的网格动作会使导出失败，不截断动作、不跨长缺口插值。

正式 SFT 仍需成功示范、按不同 session 划分 train/validation，再执行 `./scripts/export.sh --input ... --output ... --validation-session ...`；原 schema1 数据仍可读取，但不与新版配置混合导出。动作的维度、比例、夹爪开度和 J7 坐标约定保持不变。

相机参考原版的独立采集方式，使用独立线程与短缓存；采样时选取有效的近时间帧对并保存原始到达时间。这是适配现有 HDF5 schema 的实现，没有声称复用了 RoboLab 的整个 Zenoh recorder。相机曝光硬同步仍未验证。

新配置在 `configs/collection.robolab.json`、`configs/bridge.robolab.json`。标准配置保留 `hardware_verified=false`；单条入口使用本次明确授权的独立试采配置。下文旧入口与严格对齐说明仅供历史版本参考，不适用于这里的 `try_one.sh`。


目录：heinrich:/home/heinrich/pi05_plug_demo。
代码、配置、原始数据、导出数据均在这里；未修改原 roboLab、OpenPI、桥接目录或 Mac 主项目。

## 已实现

GELLO → Panda/Robotiq bridge → Kinect + D405 → HDF5 → 浏览器检查 → LeRobot v3。
一次会话可连续采集多条 episode；F2 开始/结束跟随与录制，人工标记成功、失败或放弃。关键阶段和干预标记可选。原始数据保留实测关节、目标关节、夹爪宽度、双路 RGB、分开的时间信息、命令 ACK 和标记。
复制的 pi05-rlt-rtc 在 runtime/，动作和相机接口已对齐。

当前目标是同一操作者采集约 100 条 Franka 示范，之后在 GPU 上进行离线 SFT。Stage 1 使用固定示范，同时训练 π0.5 和为后续 RLT 准备的 RL-token 表征；它不在线收集 rollout 或使用 actor–critic。阶段细分、力信息、深度、高分辨率归档、reward classifier、多操作者元数据均不是本次基础采集的必需项。

尚未完成整套真机采集验收、实际 GPU SFT、真机异步 RLT 或 RTC 性能验证。双相机实测进度见下文。runtime/ 原 Stage-2 入口仍只支持 ManiSkill；本次交付是采集和训练数据准备链路。

## 固定动作约定

action_contract_id: panda_delta_q0.03_robotiq_width0.085_v1

- action[:7] = (目标 q - 该步实测 q) / 0.03，范围 [-1,1]，单位由 rad 转成控制量。
- action[7] = 2 × 夹爪命令宽度 / 0.085 - 1；-1 闭合，+1 张开。
- state[9] = q[7] + 两个 width/2；后两维派生自同一夹爪开口，不是两路独立传感器。
- GELLO 的闭合比例先转换成米；越界拒绝，不静默裁剪标签。
- 15 Hz 采集/控制，bridge 独立执行 1 kHz 回调。
- 图像 RGB 224×224；main 按配置旋转/裁方；wrist 保持方向；两路 letterbox。
- 主机事件使用 monotonic 时间；相机设备帧时间另存并标注时钟域。两者没有自动完成时钟/曝光标定。
- bridge 的 gripper_speed/force 是 Robotiq 1–255 整数寄存器值，不是 m/s 或 N。

不能再套 DROID 速度转换或重复 DeltaActions。chunk 每行相对该执行步的实测 q。
ACK 证明目标被接收，不代表机器人已到达目标。原始数据不是含 pending queue 的完整 RLT replay。

## 时间记录与原始图像

新采集文件记录 `timing_version=2`：
- `camera_time/{main,wrist}`：SDK 返回帧后的主机接收时间，早于 Python 的颜色转换、裁剪和缩放；不是曝光时间。
- `camera_ready_time/*`：224×224 预处理完成时间。
- `camera_source_time/*`：设备帧时间，单位统一为秒；`camera_source_clock/*` 属性注明设备时钟域。Kinect 原生计数单位为 0.125 ms，RealSense API 原生单位为 ms。缺失值通过 `camera_source_time_valid/*=false` 明确表示，不冒充曝光时间。
- `camera_frame_id/*`：设备帧编号，无法读取时为 -1。
- `timing/state_bridge_receive_s`、`state_client_receive_s`：bridge 收到机器人反馈、采集端收到该反馈的主机时间；`gripper_bridge_read_s` 是夹爪读数完成时间。
- `timing/gello_read_s`、`command_send_s`、`command_enqueue_s`、`command_ack_s`：读取遥操作输入、发送命令、bridge 入队、客户端收到 ACK 的时间。ACK 不表示物理执行完成。

导出 manifest 每条源记录附带 `timing_quality`，汇总周期分布、相机接收年龄/时间差、重复帧及 ACK 往返时间。相邻采样间隔偏离标称周期超过 50%，或相对固定 FPS 累积漂移超过一帧时拒绝导出。这个阈值是初期数据质量检查，不是硬件同步精度保证；设备时钟映射及真实曝光/执行延迟仍待实机测量。旧记录继续可读，其缺失的时间信息不自动补造。

本次仍保存 RGB224；这可用于基础 SFT，前提是缩放后能看清插头与插孔。源相机的高分辨率画面在缩放/裁剪后未归档，所以以后无法恢复被丢失的细节。高分辨率/深度归档可按实际需求另行增加。

## 无硬件检查

在项目根目录运行：
```bash
.venv/bin/python -m unittest discover -s tests -q
./build/bridge_core_test
.venv/bin/python scripts/smoke.py
```

smoke 自动使用独立 socket 上的 mock bridge，结果见 reports/latest_smoke.json。
合成数据在 data/smoke_*/，导出默认拒绝 mock。显式 --allow-mock 也始终保留 mock 标记。

## 硬件到位后

configs/collection.json：
已填写并实测 Kinect `019337540647`、D405 `262622272972`；正式采集前核验相机视野、方向、GELLO 路径和 vendor/gello 中的关节标定。

configs/bridge.real.json：
填写 Robotiq 串口路径、实测 gPO 全开/全闭端点、额外抓取物的质量/重心/惯量（`load_*`，空爪为零）；核验限速、阻抗、碰撞阈值。
工具本体应先在 Franka Desk 配置；`expected_desk_tool` 与实时 `m_ee/F_x_Cee/I_ee/F_T_NE/F_T_EE` 必须一致，否则 bridge 在控制和激活夹爪之前拒绝启动。`setLoad` 只用于额外负载，不能重复填写夹爪重量。
完成硬件验收后才把两个配置的 hardware_verified 改为 true；今晚的初始模型不等于已完成整套实机验收。
相同相机参数同步到 configs/runtime_franka.yaml；不同裁剪/标定版本不要混用。
首轮使用断电排插，先明确插入深度/保持条件，再人工判定成功。

只读预检：
```bash
LD_LIBRARY_PATH=$HOME/freenect2_prefix/lib .venv/bin/python -m pi05_demo.preflight --hardware-config
```
未填硬件参数时非零退出是预期行为。

## 双相机接入验证（2026-10-30）

两台相机均直接连接 4090，采集与 runtime 配置中的序列号一致。D405 SDK 报告 USB 3.2，采集 640×480、30 FPS 彩色流；Kinect 保留原有顺时针旋转和居中裁方。两路仍输出 RGB 224×224。修正 Kinect SDK 串号需传 bytes 的兼容问题，并将旋转、裁剪和通道转换合并为一次最终 RGB 拷贝，保留原有像素结果与接收/设备时间信息，没有放宽新鲜度或时间差门槛。

真实 Kinect + D405 经正式相机 provider、记录器写入两条各 60 帧的 HDF5，采样 15 Hz，每路每条 60 个不同设备帧号，主机接收时间差最大约 44.14 ms（门槛 80 ms）。LeRobot 导出及官方 loader 对 train/validation 两个 session 的读取、视频首尾帧解码均通过。机械臂、GELLO、夹爪均为 mock，文件保留 `mock=true` 和未安装 D405 的说明，只用于链路验证。56 项 Python 测试通过。

证据及测试数据位于 `reports/camera_integration_20261030/`。测试不会连接真实 Panda 或夹爪，可在两台相机空闲时复现：

```bash
LD_LIBRARY_PATH=$HOME/freenect2_prefix/lib .venv/bin/python reports/camera_integration_20261030/probe_live_cameras.py
```

D405 目前放在旁边，尚未安装到末端，因此这不是腕部视角的训练示范。Desk 继续使用实际已安装的 2F-85 单夹爪工具模型；支架与 D405 装好后再更新组合质量、重心、惯量和工具配置，并检查腕部视野。两份 `hardware_verified` 仍为 false；夹爪接线器更换与整套真机验收仍需完成。

## 已安装的 2F-85：GELLO 夹爪试用

2026-10-29 已确认 RS-485 转接器 `DAAL8Y3V`（固定 by-id 路径，当前 ttyUSB1），并写入 `configs/bridge.real.json`。这是 USB 转接器编号，不是相机或夹爪机身出厂序列号。实测 gPO 端点为全开 3、全闭 227，来源是同日五次完整开合、gOBJ=3 且无故障的记录，保存在 `reports/gripper_integration_20261029/robotiq_4090_cycle30s_20261029.jsonl`。85 mm 仍是标称最大开度，尚未做尺量的宽度精度验证。

完整 Panda 数采仍需 D405 安装和工具负载验收；本入口独立检查 GELLO → 2F-85，复用正式 bridge 的 C++/Robotiq 官方 SDK 夹爪驱动，不连接 Panda，不生成训练 episode。

```bash
cd ~/pi05_plug_demo
./scripts/test_gripper.sh --seconds 3                  # 只读检查
./scripts/test_gripper.sh --execute --seconds 120 --force 16  # Enter 开始；Ctrl-C 停止
```

GELLO 必须接通电机电源。试用前让夹爪行程内保持空置。启动即以设定电机速度跟随 GELLO 当前旋钮开度；捏紧/放松分别饱和到全闭/全开，正常越过旧角度端点不会中断。默认 15 Hz，与数采频率一致；`--hz 30` 仅供对比，`--speed` / `--force` 是 1–255 的试用寄存器覆盖值，不会改写正式配置。程序不自动激活、复位或移动机械臂。若夹爪未激活，明确报错停止。

每次使用新读到的 GELLO 旋钮值；小于约 1 mm 的变化被死区过滤，更新目标时不等待上一个开合完成。到时、Ctrl-C 或异常会请求停止，C++ 子进程在运动期间 250 ms 未收到请求也会请求停止；串口物理断开时不能保证停机命令送达。两个串口都只允许单一控制进程使用。

`reports/gripper_teleop_*.jsonl` 记录旋钮读取、命令发送/确认和实际 gPO 反馈。命令确认耗时不代表机械到位耗时；这项测试也不包含双相机和 Panda 链路的额外延迟。

## 采集会话：F2 与 GELLO 对齐

沿用 RoboLab 的交互顺序：`init → gohome → standby ⇄ recording`。第一次 F2 执行启动回位，随后每次 F2 只切换跟随/录制，不再自动回位。`b` 没有绑定任何操作。

这里额外加入了每次接管前的对齐检查。原 RoboLab 的 GELLO publisher 只发送绝对关节目标，并不会驱动 GELLO 电机自动归位；原版重新使能后 Panda 会逐渐追向 GELLO。新版本要求手动对齐并明确按键接管，保持固定映射，不通过临时重设 offset 隐藏错位。

### 已确认的新默认姿态

采用用户在 2026-10-29 现场确认朝向正常的 [DROID reset_joints](https://github.com/droid-dataset/droid/blob/33ae6a67274f36d2e29525b86f23a56616ef43a7/droid/robot_env.py#L27)：

```text
Panda: [0, -π/5, 0, -4π/5, 0, 3π/5, 0] rad
       [0°, -36°, 0°, -144°, 0°, 108°, 0°]
GELLO: [0, -π/5, 0, -4π/5, 0, 3π/5, π/4] rad
Panda 目标 J7 = 原标定 GELLO 输出 J7 - π/4
```

两组参考写入 `configs/collection.json`，随每条 episode 保存。GELLO 的前六轴也需摆到这组新参考；J7 仍采用既有的固定 −π/4 映射，方向与增益不变。不要再修改 vendor 编码器 offset，也不要在 bridge、训练或推理端重复补偿。记录的实测状态、目标和 action 始终使用 Panda 实际关节坐标。

`configs/poses/droid_home_no_camera.json` 保留来源、运动学验证和现场验收边界。此前实机回位完成后曾读到 `cartesian_reflex`；该历史记录保持原样，不自动恢复故障。默认姿态已确定，不代表含相机、工具、工作区的整套验收已完成。

### 操作顺序

硬件验收完成后，先关闭其他占用 Franka FCI 的控制程序，并确认启动回位的运动路径清空。终端 A：

```bash
cd ~/pi05_plug_demo
./scripts/start_bridge.sh --execute
```

终端 B：

```bash
cd ~/pi05_plug_demo
./scripts/collect.sh --execute --session plug_day1_train
```

1. 在采集终端按 **F2**（或 **Enter**）：低速回到默认姿态，夹爪保持，不录制。只允许在启动阶段执行一次；回位中再次 F2 忽略。
2. 进入 **standby** 后，手动摆好 GELLO 七轴和夹爪旋钮。终端显示各轴角度误差与夹爪开度误差。七轴各自误差须不超过 **0.03 rad（约 1.72°）**，夹爪须不超过 **5 mm**，机械臂和夹爪还必须已完成停止。
3. 再按 **F2 / Enter**：通过本次对齐检查后开始跟随并记录新的一条。没有通过时继续待机；摆正后需重新按键，不会自行启动。第一条实际发送的动作还会重新检查。
4. 按 **F2 / Enter** 结束本条：先减速保持、停止夹爪跟随，再等待 **s=成功 / f=失败 / a=放弃**。也可以录制时直接按 s/f/a，同时结束并标记。标记完成后保存并回到 standby。
5. 人工整理场景，再次对齐 GELLO、按 F2，开始下一条。每次都重新检查对齐；不需要重启 bridge。待机时 Panda 是受控保持，不是自由拖动模式。若用 GELLO 操控做准备动作，该段结束后标记 a，可保留诊断但不会进入 SFT。
6. **q / Ctrl-C** 退出整个会话并停止；未完成标记的当前条保存为 aborted。启动回位时 a 也可退出。

c 切换关键阶段、i 切换干预；它们仍是可选标记。F2/Enter 作用于采集终端；Mac 功能键被系统占用时可直接用 Enter。没有引入原版全局键盘插件。原版 F1 checkpoint 不参与这一版流程。

只有 recording 状态产生示范帧。回位、减速停止、待机对齐和等待标记均不写入 episode；SFT 默认只选成功示范。末条未标记、异常与退出均不会自动成为成功。异常时先请求停止，再进行文件收尾；`.inprogress` 文件不可导出。

底层新增可恢复的 hold，与永久 stop/故障分开。hold 用受速度/加速度/jerk 约束的减速轨迹，之后保持位置，并由串口 worker 确认夹爪停止；重新跟随必须等停止完成。回位参考上限为 0.05 rad/s，bridge 回位模式独立限制不超过 0.06 rad/s、0.15 rad/s²、1 rad/s³，并且不超过原标定限制。正常动作增量、250 ms 看门狗、状态新鲜度、实时内核要求、碰撞保护都继续生效。终端卡住、设备失联或故障会停止，不能通过 F2 自动恢复。

### 验证边界与 mock

2026-10-29 操作者确认夹爪接线器故障、待更换；2026-10-30 双相机已接入并完成上述链路测试，但 D405 尚未安装到末端。两份 `hardware_verified` 保持 false，不代表已完成真机验收。更换夹爪转接设备后应重新核对 by-id 路径。

两个终端可以无硬件演练同一按键流程：

```bash
./scripts/start_bridge.sh --mock
./scripts/collect.sh --mock --interactive --session mock_keyboard
```

mock 的 leader 自动贴近合成机器人状态，不能用于验证真实 GELLO 校准或夹爪时延。自动 smoke 入口仍支持 `--mock --frames 12`。

建议先采 10 条验证链路，再采约 100 条。同一人同一天可按整条 episode 分成 `plug_train`、`plug_val` 两批，例如 90 条训练、10 条验证，避免相邻帧混入不同集合。

## 浏览器检查

```bash
.venv/bin/python -m pi05_demo.review --input data/raw --port 8766
```

Mac 转发：
```bash
ssh -L 8766:127.0.0.1:8766 heinrich
```

打开 http://127.0.0.1:8766。支持双相机播放/逐帧、目标关节、夹爪和阶段标记；入口只读，不裁剪/修改原始数据，也不启动机器人。

## 导出 SFT 数据

```bash
./scripts/export.sh --input data/raw --output data/datasets/plug_v1 \
  --validation-session plug_day2_val
```

输出 train/、validation/、norm_stats.json、manifest.json。
按采集 session 划分，统计只用 train；manifest 保留源文件 SHA256 和官方 loader/视频验证结果。
已有输出不覆盖，请使用新版本名。不同采集/标定配置不能静默合并。
采样缺口和明显时间抖动会拒绝导出；失败输出留在 .inprogress-*，不会发布成训练集。

configs/training_contract.json 定义数据接口。
pi05_demo.training_data.ChunkDataset 返回 state[9]、actions[H,8]、OpenPI 图像和掩码、prompt；第三路是 mask=false 的 padding。
两个 SFT loader（`pi05_demo.training_data.ChunkDataset` 与 `stage1.data.PlugDataset`）只取 episode 内真实存在的完整动作块，不填充末尾。例如 12 帧、H=10 时得到 3 个样本，末尾动作仍包含在最后一个完整窗口中。少于 H 帧的 episode 不产生训练窗口；整个 split 没有完整窗口时明确报错。原始数据和导出的帧数保持完整。
已接入独立 RLinf Stage-1 SFT 入口，支持 2×A800 80GB FULL_SHARD 和梯度检查点；CPU 单元及双进程测试见 reports/stage1_*。完整模型的 GPU 显存和吞吐仍待 AutoDL 验证，不能直接套 DROID 速度配置。
Stage 1 必须同时训练 RL-token 表征，普通 pi05_droid 权重不等价。

末尾窗口处理属于离线训练取样；inference-time RTC 则是在机器人执行当前动作块时，利用已承诺动作约束下一动作块的生成。遥操作采集不运行 RTC，也不需要补造 RTC 队列；训练和执行保持同一动作契约、频率、相机处理和归一化即可。

## RLT/RTC 模板

```bash
./scripts/runtime.sh runtime-validate --runtime-config configs/runtime_franka.yaml
```

checkpoint、norm_stats、rlinf_root 需填真实产物。H=10/C=5/D=2 是待 profile 初值。
训练与推理使用同一个动作契约和 camera factory。
GPU 训练、真机 reward/复位、异步 learner、队列 replay 是下一阶段，不会被采集程序启动。

未来真机 Stage 2 可以由操作者人工判定成功/失败并提供稀疏奖励，无须先补齐阶段标注或训练 reward classifier。接入时需正确写入 transition 的 reward、终止/超时标记和最后观察。RLinf 的 `single_stage` 键盘奖励与 `rlt_policy_switch` 的策略切换是不同功能；后者的切换按键不能直接当成成功标签。当前 runtime 真机奖励接口尚未完成。

参考：[RLinf RLT 两阶段流程](https://rlinf.readthedocs.io/en/latest/rst_source/examples/embodied/rlt.html)、[Franka 人工键盘奖励](https://rlinf.readthedocs.io/en/latest/rst_source/examples/embodied/franka.html#label-rewards-from-a-keyboard-optional)、[推理时 RTC](https://www.pi.website/research/real_time_chunking)。

## 环境和构建

.venv 新建并只读继承 openpi 的 site-packages；h5py/dynamixel-sdk 安装在新环境内。
scripts/export.sh 只读复用 roboLab/.venv-lerobot 的 CPU LeRobot 安装；没有修改它。本版本不是脱离原机的容器。
vendor/ 保留 GELLO、camera helper、C++ 依赖副本及 GELLO LICENSE。
原始文件指纹见 reports/source_manifest.json；交付复核见 reports/source_integrity.json。

```bash
cmake -S cpp/bridge -B build -DPI05_WITH_FRANKA=ON -DPI05_FETCH_DEPS=ON \
  -DFETCHCONTENT_SOURCE_DIR_RUCKIG="$PWD/vendor/cppdeps/ruckig-0.15.3" \
  -DFETCHCONTENT_SOURCE_DIR_JSON="$PWD/vendor/cppdeps/json-3.11.3" \
  -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
(cd build && ctest --output-on-failure)
```

真实 bridge 使用 kEnforce。只有一个线程访问 Robotiq Modbus，避免并发串口事务；通讯失败不会伪装成有效夹爪反馈。

## 2×A800 80GB 全量微调

准备、smoke、训练、恢复和验证边界见 [Stage-1 操作说明](docs/STAGE1_2XA800.md)。

入口：scripts/setup_stage1.sh、scripts/stage1.sh inspect|preflight|smoke|train。新 RLinf 副本只在 vendor/rlinf_stage1/，原项目未改动。

## 官方 Robotiq SDK 与历史无相机工具配置（2026-10-29）

夹爪通信已接入 [Robotiq 官方 C++ SDK 1.2.0](https://github.com/robotiq/grippers/tree/068bc7b5192ea54f7c58b6577f499ce65e0b7644)，固定在 `vendor/robotiq_grippers`，保留原厂 BSD-3-Clause LICENSE。使用 SDK 的 `GripperModbusClient`：bridge 的单独 Modbus 工作线程安排事务，诊断 `status` 只发读请求；没有第二个后台写指令循环，也不会让串口事务进入 Panda 1 kHz 回调。SDK 传输六个有定义的字节（三个寄存器），其 16 字节内存块的保留尾部不走串口。

重新编译：`cmake --build build -j4`。保留已测 gPO 3/227、命令 ACK 语义、串口独占及 250 ms 诊断看门狗。已激活且健康的夹爪不会在 bridge 启动时被复位/打开；空闲后的 0x09 通信标志仅通过恢复读取清除，其他故障不会自动复位。`build/gripper_activation_probe` 是伪终端测试夹具，不是实机启动入口。

`configs/tools/robotiq_2f85_no_camera/` 包含 Desk 导入包、JSON、说明、来源与离线生成脚本。质量 0.921 kg、法兰系重心 [0,0,0.060] m、TCP [0,0,0.174] m、标准正装旋转为单位阵。惯量是官方 2F-85 URDF 全开姿态各零件在总质心处合成的估计值，**不是当前实物标定**；手册惯量没有明确参考点，因此没有直接填入。

本机 Desk 版本为 **4.2.2**，实际导入方式是 Settings → END-EFFECTOR → Other → Mechanical Data 编辑 → UPLOAD **`endeffector-config.json`** → APPLY；这个版本不上传 tar.gz。可先 DOWNLOAD 对比导入值，应用后运行 `.venv/bin/python scripts/check_desk_tool.py` 核对实时反馈。2026-10-29 已应用并核验质量、重心、完整惯量和 TCP；完成 10.74 秒低速回位，J1–J6 沿用原始 home，J7 目标 -0.2953981633974483 rad，结束后 Idle、无错误，最大关节偏差 0.000372 rad。记录在 `reports/initial_pose_check_20261029/`。

该工具配置不包含相机/支架质量。2026-10-30 D405 已接入 USB，但尚未安装到末端；序列号已填写，正式腕部视角 SFT 示范仍等待安装。实际安装 D405 与支架后需更新整体工具模型并检查视野。手册的 “Camera + 2F-85” 指 Robotiq 自有 Wrist Camera，不能套用给 Intel D405。

D405 组合工具配置待支架及安装位置确定后计算；启用的 Desk 模型必须对应当时实际安装的硬件。当前单夹爪模型不预先计入未安装的相机。


## 历史流程：严格对齐试采（已由上面的 RoboLab 入口替代）

在 4090 的终端执行 `cd ~/pi05_plug_demo && ./scripts/try_one.sh`。仅启动一轮，退出会停止本轮 bridge；不自动恢复或重复启动。当前 D405 放在旁边，Desk 仍使用 2F-85 单夹爪工具模型。首次试动任务是小幅空中动作与夹爪开合，数据单独放在 `data/trial_raw_20261030/<session>/`，不混入正式插接示范。

1. 首先只读 GELLO，并读取夹爪当前开度；此时尚未启用 Franka 控制。按预摆位表调整 GELLO，全部对齐后出现绿色“预摆位完成”，按 Enter 启动试采准备。这里比较的是默认姿态参考，尚不能开始录制。
2. 出现“准备完成”后按 Enter：Franka 低速回到既定默认姿态，此时不录制。回位期间保持 GELLO 的预摆位。
3. 回位后对齐表改为与 Franka 实际位置比较。J1 从 GELLO 底座数起，J7 在末端；表格显示映射后的 GELLO 目标、Franka 当前角度及还需调整量。先调误差最大的轴，让“还需调”的绝对值变小；不需要自行换算第七轴。出界角度也能显示，但仍被控制与接管检查拒绝。
4. 整体出现绿色“可以开始”后，再按 Enter；看到“正在跟随并录制”才开始动作。单轴 OK 或绿色“预摆位完成”不代表已经开始录制，也不会自动接管。
5. 做 10–15 秒小幅空中动作与夹爪开合，按 Enter 停止，再按 s=本次试动正常 / f=异常 / a=放弃。也可直接 s/f/a 停止并标记。保存一条后自动结束。q 或 Ctrl-C 退出。
6. 退出后自动打印数据路径、时序检查结果和 `episode_check.json`。本轮日志和两路相机截图在 `reports/one_episode_trial_20261030/<session>/`；这只证明本轮数据链路情况，D405 未安装的画面不作为正式腕部视角训练数据。

预摆位在启动实时控制之前进行，可以从容调整。进入控制后如果反馈间隔超过 10ms，仍会停止并保留日志；不要在故障后继续按键尝试接管。此前原地保持通过了 30 秒检查，但继续等待时出现过一次 11ms 间隔并停止，因此尚不能认定链路稳定。

标准配置的 `hardware_verified` 仍保持 false。此入口使用已授权且核对过工具模型的独立试采配置。回位、绝对 GELLO 映射、J7 固定偏移、1.72°/5mm 接管门槛、限速、阻抗、碰撞保护、250ms 命令看门狗均保留。

本次修正了反馈间隔与下一步预测时长混用的问题：每次使用机器人最新 q_d/dq_d/ddq_d 同步后，只向前预测 1ms。试采配置显式允许最高 10ms 的接收间隔，超过立即停止；未指定此配置时仍使用 2ms 的间隔门槛。新增回归测试检查漏周期后不额外跳跃、轨迹导数限制及长中断拒绝。bridge 状态会报告回调次数、间隔次数、最长间隔和控制命令成功率，用于评估实际链路；不会伪造缺失的采集帧。

### 插接 Reflex 诊断

2026-10-31 按用户要求，将 RoboLab 采集配置的三个平移方向碰撞阈值设为各 50 N（原 20 N）；转动阈值各 15 Nm、关节力矩阈值不变。连续采集与单条试采均在下次启动 bridge 时生效，未启用 100 N。50 N 是碰撞检测阈值，不是插入目标力。
控制异常后，桥接保存 libfranka 已缓存的历史到本场日志目录的 `franka_fault.csv`，并额外保存 `franka_fault.csv.wrench.json`。
后者包括机器人时间、关节位置速度、基座/刚度坐标系下的外力估计（前三项 N、后三项 Nm）、关节外力矩和碰撞指示；它是故障前有限窗口，不是整条 episode 的连续力传感记录。若异常不含历史，记录为空，不补造数据。
日志写入只发生在控制退出之后，不在实时回调里写文件。
