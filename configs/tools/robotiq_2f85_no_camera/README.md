# 2F-85 原厂联轴器，标准正装，暂不安装 D405

这是供初次调试的工具模型，未对当前实物进行负载辨识。生成此包不改变机器人；必须在 Franka Desk 导入、选择并保存后，再读取 FCI 状态核对。

| Desk 参数 | 值及来源 |
| --- | --- |
| Mass | 0.921 kg，原厂手册含联轴器 |
| Center of mass | 法兰系 [0, 0, 0.060] m，原厂手册 |
| TCP translation | 法兰系 [0, 0, 0.174] m，原厂手册 |
| TCP rotation | 标准正装单位旋转，Rx/Ry/Rz = 0 |
| Inertia at COM | 官方 URDF 全开姿态的组件惯量合成估计，详见 JSON；不是实物标定 |
| Additional held payload | 空爪为 0 kg；不要把夹爪重复计入 `setLoad` |
| D405 / camera bracket | 本包不包含；安装后要更新工具模型 |

`endeffector-config.json` 使用米、千克、kg·m²，矩阵按列展开。惯量对角线约为 [0.00124613, 0.00203808, 0.00115342] kg·m²，完整矩阵保留了官方组件模型合成的小非对角项。公式是 `I_COM = Σ(I_i + m_i (||d_i||² E - d_i d_iᵀ))`，其中 d_i 是组件质心到总质心的位移。组件质量总和 0.920864 kg，与手册 0.921 kg 的舍入一致；手册提供安装后的重心/TCP，URDF 用于估计质心惯量。这不是厂家为当前装配出具的完整惯量标定。

原厂手册表中的 4180/5080/1250 kg·mm² 没有明确说明参考点，未直接填入 Franka 所需的质心惯量。也未把 Robotiq 自有 Wrist Camera 的参数套给 Intel D405。

## 在 Desk 使用

1. 本机已确认是 Desk **4.2.2**：进入 Settings → END-EFFECTOR → Other，点击 Mechanical Data 旁的编辑图标，再用 UPLOAD 上传 **`endeffector-config.json`**。这个版本使用普通 JSON，不上传 `.tar.gz`；压缩包仅为其他兼容版本保留。
2. 查看质量、重心、惯量、TCP 后点击 APPLY。可先 DOWNLOAD 导出待应用值，与原 JSON 逐项比较，尤其检查小数的科学计数法指数；仅上传不代表已成为活动工具。应用后可能需要正常解锁并重新启用 FCI。
3. 在 4090 的 `~/pi05_plug_demo` 中运行 `.venv/bin/python scripts/check_desk_tool.py`，只读核对实时 FCI 参数。`matches_expected_tool=true` 只证明 Desk 已应用该模型，不代表负载/碰撞验收完成。
4. 本次回位目标继续沿用原 J1–J6，J7 为 -0.2953981633974483 rad；空爪、慢速检查方向。此配置包不启动运动，也不改变碰撞阈值。

如果其他 Desk 版本没有上传入口，可选择自定义工具按 JSON 填写；若 UI 不支持完整惯量矩阵，应先确定字段格式，不能自行丢弃字段。浏览器自签名证书提示及登录需由操作者本人完成。

2026-10-29 已通过 4090 上登录的 Desk 应用此配置，并从 FCI 核对全部字段。首次保存值中有一项惯量丢失指数，检查器在回位前拒绝通过；重新上传原 JSON、核对下载值并应用后，实时读回一致。已完成约 10.74 秒的低速回位，目标 J7 为 -0.2953981633974483 rad，结束后 Idle、无错误，最大关节偏差约 0.000372 rad。详细证据在远端 `reports/initial_pose_check_20261029/` 和 `reports/robotiq_sdk_20261029/`。这些结果不替代实物负载标定或操作者对夹爪方向的目视确认。

正式数采的硬件验收标记仍为 false。D405 未安装不影响机械臂/夹爪独立调试；双相机训练数据采集需要等它安装、填写序列号并检查画面。组合工具配置可以提前计算，但启用的工具质量、重心和惯量必须对应当时实际安装的硬件，不能提前计入尚未安装的相机。D405 支架和安装位置目前未确定，用户计划到货后提供；组合版尚未生成或应用。

来源：[Robotiq 手册 §5.2.3，第 82 页](https://assets.robotiq.com/website-assets/support_documents/document/2F-85_2F-140_General_PDF_20210623.pdf)、[官方 URDF 固定提交](https://github.com/robotiq/robotiq-ros/blob/f95169e1b085254e0a98159ca93bdd1aff7a681f/grippers/robotiq_description/urdf/robotiq_2f_85_macro.urdf.xacro)、[Franka 工具导入格式](https://download.franka.de/ride-cli-doc-v1.1.1/end_effector_commissioning.html)。完整来源和模型限制记录在 `provenance.json`。
