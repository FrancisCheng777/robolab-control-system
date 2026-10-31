# Franka / stock Robotiq 2F-85 + D405 + R2 PETG

2026-10-31 已生成组合工具参考配置。用户照片确认：原厂夹爪和指尖、原联轴器、R2 PETG 侧装相机支架；没有黄色 TPU 指尖。用户授权按 PETG 材料估算打印件质量。

## 交付与状态

- **Desk 4.2.2 导入文件：`endeffector-config.json`**。
- `.tar.gz` 仅给需要该格式的其他版本使用；当前 Desk 使用普通 JSON。
- `assembly-inputs.json`、`geometry-derived.json`、`provenance.json` 记录实装信息、计算参数、来源和限制。
- 旧单夹爪配置保留在相邻 `robotiq_2f85_no_camera/` 目录用于历史核对。
- 2026-10-31 已应用至 Desk，界面显示 APPLIED；FCI 已恢复激活，2026-10-31 15:43 只读检查通过：质量、重心、惯量、F_T_NE 和 F_T_EE 全部匹配，差异字段为空。原配置备份和应用记录见 reports/d405_desk_apply_20261031_1523。未恢复机器人或执行运动。

## 参数（米、千克、kg·m²）

| 项目 | 参考值 |
| --- | --- |
| 工具总质量 | 1.009855054 kg，约 1.010 kg |
| 质心，法兰系 | [0, -0.005963193, 0.060715107] m |
| TCP，法兰系 | [0, 0, 0.174] m，沿用原厂指尖及原配置 |
| TCP 旋转 | 单位旋转，沿用原标准正装约定 |
| 质心惯量对角线 | [0.001657493, 0.002067619, 0.001562927] kg·m² |
| 附加抓持物 `load_mass` | 空爪 0 kg；不重复计入整套工具 |

完整非对角惯量见 JSON。`transformation` 和 `inertia` 按列展开。

质量分解：原夹爪及联轴器 921 g + D405 60 g + PETG 支架 26.855 g + 新增相机螺丝及垫圈 2 g 工程余量。支架复用的夹爪螺丝不重复加入。

## 计算依据和精度

[R2 仓库固定提交](https://github.com/sri299792458/robotiq-2f85-d405-mount/tree/96941676e8112bccbda49420375acfa5d4c586e5) 的 STEP/STL、生成器、机械检查均通过 SHA-256 核对。实体体积 21.1457 cm³，采用 [Prusament PETG 数据表](https://prusament.com/wp-content/uploads/2022/10/PETG_Prusament_TDS_2021_10_EN.pdf) 的参考密度 1.27 g/cm³。品牌密度未实测；完整实体质量一般高于非实心打印的实际质量，未将 40% 填充错误地直接乘到总体积上。

安装关系沿用旧工具的标准正装坐标约定，并用本次照片核对原厂指尖、相机侧装位置及支架版本。名义 CAD→法兰映射为 `(x, -z, y + 0.0112)` m：CAD X 沿开合方向、Y 向指尖、+Z 为相机侧；相机位于名义法兰 -Y 侧。11.2 mm 轴向偏移来自厂家手册的含联轴器 TCP 174 mm 与闭合本体高度 162.8 mm 之差。该映射是沿用标准安装的**名义工程模型**，不是从照片测出的毫米级标定，也不是手眼标定。

各部件惯量先旋转到法兰轴，再通过平行轴定理合成到总质心。D405 按 42×42×23 mm 均匀长方体估算；相机紧固件分为两个 1 g 点质量，位于后安装板螺孔轴。原夹爪沿用既有全开姿态的官方 URDF 惯量估算。线缆/连接器的附加质量和牵拉力未单独建模。此文件不包含相机碰撞外形。

## 复算与验证

需要 Python 3 和 NumPy，在本目录运行：

```sh
python3 derive_geometry.py
python3 build_profile.py
```

检查覆盖源文件校验和、STL 体积与包围盒、旋转正交性与正行列式、惯量对称/正定/主惯量三角约束、总质量、TCP 保持及归档一致性。积分另用解析四面体检验，合成另用解析双点质量和法兰原点惯量守恒检验。

## Desk 应用

Settings → END-EFFECTOR → Other → Mechanical Data 编辑 → UPLOAD → 下载待应用值核对 → APPLY。本机实际导入 `desk-import-with-existing-collision-model.json`，它保留原有 collisionModel，机械参数与 `endeffector-config.json` 完全相同。Desk 科学计数法字段曾在首次导入后导出异常；重新导入后全部数值一致才应用，应用后导出与 FCI 读回也已通过。仅生成、上传或同步文件都不代表 Desk 已应用。

应用后在 4090 的 `/home/heinrich/pi05_plug_demo` 执行 `.venv/bin/python scripts/check_desk_tool.py` 只读核对。若 Desk 仍是旧模型，更新后的 bridge 期望值应拒绝通过；不应通过改回旧期望值来绕过不一致。读回匹配仅表示参数生效，不代表负载辨识、碰撞验收或相机手眼标定。
