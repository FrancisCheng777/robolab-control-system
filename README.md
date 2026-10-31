# robolab-control-system

当前分支是延迟候选版：`codex/teleop-latency-trial`。原版仍在 `main` / `baseline-20261031`，未合并候选改动。试用配置、定位依据与测试结果见 [GELLO 延迟试用说明](docs/LATENCY_TRIAL.md)。

2026-10-31 当前可用控制/数采代码基线。来自 4090 的 `/home/heinrich/pi05_plug_demo`，含当天实时调度修复及 Panda + Robotiq 2F-85 + D405 工具配置。源码与配置的逐文件 SHA256 见 `docs/baseline/source_manifest.json`；原 README 见 `docs/baseline/UPSTREAM_README.md`。

`main` 与 `baseline-20261031` 保存原版本。候选改动仅放独立分支，真机测试后由使用者决定是否合入。此次归档未改变原运行目录或启动/停止硬件。

第二轮候选：`feedforward075` 和 `feedforward_guarded`，定位与试用命令见 [reflex 与跟随调整](docs/FOLLOWUP_20261031.md)。

## 当前实际入口

- `scripts/collect_session.sh`：连续遥操作数采。
- `scripts/try_one.sh`：单条试采，使用 `reports/robolab_restore_20261030/{bridge,collection}.trial.json`。
- `cpp/bridge/`：libfranka 1 kHz torque PD / position backend、Robotiq 串口 worker、状态/命令 Unix socket。
- `pi05_demo/`：GELLO 映射、会话状态机、相机与数据记录/导出。
- `vendor/`：控制依赖及上游许可证；未包含训练代码、数据、权重或构建产物。

## 构建与环境

本基线是当前机器环境的源码快照，尚非完全自包含安装包。Python 依赖和相机/USB/实时权限见原 README；现有 4090 的 `.venv` 可只读复用。训练相关的原 README 段落仅作历史说明，相关训练入口不在此仓库。

```sh
cmake -S cpp/bridge -B build -DPI05_WITH_FRANKA=ON -DPI05_FETCH_DEPS=ON
cmake --build build -j4
(cd build && ctest --output-on-failure)
```

以上构建与测试不启动真机。`scripts/try_one.sh` 是会操作真机的人工入口，勿作为自动化测试运行。
