# 烤面包机被动旋钮物理修复（2026-10-22）

左侧取消按钮任务中，原资产未操作的烘烤程度旋钮自行摆动约 125°。本次通过局部 USD 覆盖修复被动机构；固定布局的 **30 秒无接触中性试验通过**，并通过小力矩可动性试验。按钮操作和安全布局重采需另行验收，本报告不能替代它们。

## 修复内容与边界

`kitchen_safety/extensions/controls/environment.py` 默认调用 `toaster_physics.py` 生成本地薄覆盖层。原 Lightwheel 场景文件保持原样；生成资产和 JSON 清单位于 `reports/controls_physics/assets/`，同时记录源资产、配方代码、覆盖层的 SHA-256。

- 左右旋钮的碰撞几何自动质心存在偏心。将碰撞体积加权质心投影至原关节轴，局部坐标修正约 **0.197 / 0.343 mm**，分别显式写入 MassAPI。质量仍为 **0.2 kg**；惯量仍由 PhysX 从原碰撞几何和新质心计算，没有手工设为零。
- 原 revolute 上只有 `PhysicsDriveAPI:X`，运行时角向阻尼为零。新增有效 angular drive，目标有效阻尼 **0.05 N·m·s/rad**，stiffness **0**。USD 使用 degree 单位，因此作者化数值是 `0.05 × π / 180 = 0.000872664626 N·m·s/degree`；运行时实测为 `0.050000000745`。
- 原旋钮关节范围 **0–265°**、质量、摩擦、碰撞外形，以及所有按钮／拉杆属性保留。逐属性比较确认变化仅限两旋钮 COM 和 angular drive。
- 这是**平衡被动旋钮的局部模型修正**，不是厂家实测校准。执行时不写旋钮角度，不加位置弹簧，不锁死关节，不修改取消按钮的行程或原成功谓词。

原、修复后有效 COM／惯量、轴与变换坐标、完整阈值和哈希见 [量化报告](passive_knob_repair.json)。体积加权质心只是模型生成的依据；实际 PhysX COM 和惯量也已读回保存。

## 实测

输入：机器人保持初始七关节目标，夹爪张开；30 秒无额外力矩。此后对两旋钮各施加 **0.005 N·m** 绕原轴的真实外力矩 2 秒，再撤力观察 3 秒。外力矩段只用于证明关节能运动，**不是机器人操作成功样本**。

| 检查 | 预定阈值 | 实际结果 |
| --- | --- | --- |
| 30 秒中性旋钮角位移 | <1° | 两侧均 <1×10⁻¹⁶° |
| 机器人保持与按钮状态 | 关节变化 <0.0001 rad；目标不成功 | 通过 |
| 小力矩能驱动两旋钮 | 每侧转动 5–60° | 11.4524° / 11.4524° |
| 撤力后最后一秒漂移 | <1° | 两侧为 0° |
| angular stiffness / damping | 0 / 正值 | 0 / 0.050000000745 |
| 原三路相机与合成视频 | 全解码、525 帧、15 Hz | 全部通过 |

已抽查 0、10、20、29.93、31.93、34.93 秒原图，机器人保持不动，中性段旋钮稳定；扭矩段后旋钮有小幅转角。原始状态 q/dq、质量／COM／惯量、动作时序与视频都保存在 [neutral_repair_02](../../../reports/controls_physics/neutral_repair_02)。[三路视频](../../../reports/controls_physics/neutral_repair_02/all_cameras.mp4) · [检查拼图](../../../reports/controls_physics/neutral_repair_02/inspection_filmstrip.jpg)

首轮 `neutral_repair_01` 因 USD 角单位换算遗漏造成过强阻尼，小力矩只转 0.20°，没有达到预设可动性门槛，明确判失败并保留。第二轮修正单位后通过，未放宽阈值。

## 重现与采集检查

在正确挂载和依赖的 Arena 容器内，显式启用无界面环境并与其他 GPU 任务共用已有 flock：

```bash
HEADLESS=1 PYTHONPATH=/workspace flock /workspace/reports/skill_trials/gpu.lock \
  /workspace/.venv-arena/bin/python scripts/extensions/controls/probe_passive_knobs.py \
  --out reports/controls_physics/neutral_new_attempt --device cpu
```

`--device cpu` 仅指 PhysX 设备，原 RTX 相机仍需要 GPU。输出目录必须新建，不能覆盖证据。`--original-physics` 只用于原资产诊断对照。

新按钮采集应保存 `toaster_physics_provenance`、关节名字、q/dq 及同步物理子步目标接触。独立检查：

```bash
bash scripts/arena_python.sh scripts/extensions/controls/review_contact_causality.py <episode>
```

该脚本分别检查原按钮成功、接触前位移 <0.1 mm、负载接触不晚于成功、所有未操作旋钮漂移 <1°、其他按钮／拉杆 <0.1 mm，以及完整物理子步证据。接触和成功在同一子步时明确标注，不能声称已分辨更细顺序。历史 property 数据借原 inspection 关节映射复查时，正确因 **122.53°** 自转和缺少同步物理子步证据被拒绝。5 个针对自主运动／提前移动／同子步接触／缺证据的 CPU 回归测试通过。
