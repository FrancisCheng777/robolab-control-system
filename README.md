# robolab-control-system

2026-10-31：使用者已在真机上认可 `responsive` 的跟随手感，并授权发布到 `main`。最终数采兼容检查与限制见 [发布检查](docs/RELEASE_20261031.md)。最初可用版本保留在 `baseline-20261031`。

2026-10-31 当前可用控制/数采代码基线。来自 4090 的 `/home/heinrich/pi05_plug_demo`，含当天实时调度修复及 Panda + Robotiq 2F-85 + D405 工具配置。源码与配置的逐文件 SHA256 见 `docs/baseline/source_manifest.json`；原 README 见 `docs/baseline/UPSTREAM_README.md`。

本仓库独立运行于 `/home/heinrich/robolab-control-system`。原数采与训练目录 `/home/heinrich/pi05_plug_demo` 未被覆盖。

纯遥操作（不启用相机、不录数据）：

```sh
./scripts/try_latency.sh responsive --execute --teleop-only
```

连续数采（启用真实 Kinect/D405，100 Hz 控制、15 Hz 记录）：

```sh
./scripts/collect_session.sh
```

只检查数采配置、不启动设备：`./scripts/collect_session.sh --inspect`。

`responsive` 的 J1–J4/J5–J7 参考速度分别为 0.3/0.5 rad/s。录制时自动选用 `responsive_recording`，腕部参考上限为 0.4 rad/s，适配原有 15 Hz、每帧 0.03 rad 的训练动作范围；导出仍检查实测动作，禁止剪裁。两种模式均为前馈 1.0、J7 Kp=90；J2/J3 阈值为使用者要求的 30 Nm，其他关节和笛卡尔阈值未扩大。新接触阈值尚未再次真机验证。

纯遥操作仍需分别确认启动、回位、跟随。数采沿用最新 pipeline 的两次确认：第一次确认后检查设备并自动回位、开爪；第二次确认才跟随并录制。回位结束不自动开始录制。

## 当前实际入口

- `scripts/collect_session.sh`：使用 `responsive_recording` 连续遥操作数采。
- `scripts/try_one.sh`：单条试采，使用 `reports/robolab_restore_20261030/{bridge,collection}.trial.json`。
- `cpp/bridge/`：libfranka 1 kHz torque PD / position backend、Robotiq 串口 worker、状态/命令 Unix socket。
- `pi05_demo/`：GELLO 映射、会话状态机、相机与数据记录/导出。
- `vendor/`：控制依赖及上游许可证；未包含训练代码、数据、权重或构建产物。

## 构建与环境

本仓库尚非完全自包含安装包。Python 依赖和相机/USB/实时权限见原 README；现有 4090 的 `.venv` 可只读复用。训练相关的原 README 段落仅作历史说明，相关训练入口不在此仓库。USB 1 ms 缓冲与网卡实时优先级是临时系统设置，重启后需重新检查。

```sh
cmake -S cpp/bridge -B build -DPI05_WITH_FRANKA=ON -DPI05_FETCH_DEPS=ON
cmake --build build -j4
(cd build && ctest --output-on-failure)
```

以上构建与测试不启动真机。`scripts/try_one.sh` 是会操作真机的人工入口，勿作为自动化测试运行。
