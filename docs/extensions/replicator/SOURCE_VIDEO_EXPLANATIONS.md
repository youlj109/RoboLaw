# 源几何解释视频

`scripts/extensions/replicator/render_geometry_explanation.py` 只读取既有数据，用 CPU 生成解释视频。原始 RGB、原始视频、仿真状态和安全阈值不变。当前范围为 G 型 mustard→bowl 的 human / property / robot 三类。

## 输入与时间轴

- 两条成功的物理任务记录：无新增实体基线、实际新增实体执行；源布局、控制参数、相机标定和录制步长必须相同。
- 相同变体布局 JSON，以及对应两条记录的 `audit_variant_source_geometry.py --expect risk/safe` 完整报告。工具验证 result、physics、layout、源资产和几何实现哈希；不能把另一条轨迹的数值贴到视频上。
- 图像来自 `images.h5` 第 i 帧，表示 action i 执行前的 obs。每帧读取相机 `position_world`、`quaternion_world_ros_xyzw` 和 `cameras.json` 中的 K。
- 两侧按原始 action 索引、原速播放，无阶段或事件对齐；短侧结束后明确标记保持末帧。
- 最近点标记只用当前图像对应 obs 重算。`Ensuing action min` 是独立源审计中随后这个 action 的物理子步最小距离，**其未来姿态不投影到当前图像**。能对应前一个 action 末子步时，还验证 obs 与该真实子步及源审计距离一致。

## 画面含义

- 紫色：源碰撞几何轮廓。原 mesh 使用其三角面；Cube 使用精确面；Cylinder 仅在可视化中作 64 边近似，距离计算仍使用原解析 primitive。
- 橙色：每个源 collider 的世界 AABB 加原安全 margin 后取投影并集，是保守解释边界，不是精确等距安全曲面。
- 左侧实体永远标为 `OFFLINE HYPOTHETICAL HAZARD`；右侧使用实际实体及实际逐帧姿态。动态财产逐帧读取 obs 中的 hazard pose。
- 源轮廓和边界为 x-ray 投影，未重建遮挡；近裁剪面为 0.01 m，并裁剪屏幕范围。二维重叠不能证明三维碰撞、安全或财产破损。

## 真实已测诊断命令

下面是旧 `_01` 人体样本的工具测试，只能带 `--diagnostic`。输出有醒目诊断横幅，不能恢复旧样本交付资格。路径已存在时拒绝覆盖，重新运行须换输出名。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/render_geometry_explanation.py \
  --baseline datasets/avoidance_trials/replicator/base/a20261023_humanbase01 \
  --episode datasets/avoidance_trials/replicator/human/a20261023_01 \
  --layout configs/extensions/replicator/avoidance/human_repaired03.json \
  --risk-report reports/replicator_execution_20261023/g_mustard/human_baseline_source_cpu02.json \
  --safe-report reports/replicator_execution_20261023/g_mustard/human_actual_source_cpu01.json \
  --camera overview_camera --diagnostic \
  --output exports/avoidance-trial-videos/replicator_human_source_explanation_diagnostic01.mp4
```

该测试实际完成 561 帧、15 fps、37.4 秒全量解码；人工抽查 0/309/317/560 帧。结果与独立抽帧复核分别在同名 `.validation.json`、`.sampled_visual_cpu01.json`。property 和 robot 的源 mesh / Cube / Cylinder 路径已支持，但尚未用两类完整录制做端到端解释视频验证，不能写成三类全部实测。

## 新样本

把命令中的基线、实际 episode、risk/safe 报告换成**同一新轮次的确切路径**，使用新的输出名，去掉 `--diagnostic`。默认模式硬性要求两条记录的 startup 审计及 `audit_saved_collection_evidence` 通过。解释视频生成/解码通过仍不能替代原任务、源几何、配对必要性、原始视频和独立视觉检查。

需要独立复核采集时间轴和源码时：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_collection_evidence.py \
  --episode <确切新episode目录> --output <新的CPU复核报告.json>
```

该命令不会推断旧样本缺失的物理步，任一缺失或不一致返回非零。
