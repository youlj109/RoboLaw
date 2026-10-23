# G 型芥末瓶任务：三类变体的配对规划与源几何检查

`paired_variant_plans.py` 补齐财产、横杆的三个 seed 障碍开关配对，也支持人体。它只研究同一真实成功基线的抬升末状态到同一碗上方 TCP 目标的自由转运段。它不启动物理仿真，不执行关障碍路径，不代表三个完整物理任务成功；其他 13 个 Replicator 原始任务仍须另行实现和验收。

`audit_variant_source_geometry.py` 检查完整记录中的每个物理子步。原始机器人/手指碰撞网格、瓶网格和支持的障碍物 primitive 与凸包代理分列。尤其不能将 `robot_hull` 的相交写成原始机器人网格相交。源 mesh、源凸包均不是 PhysX cooked collider，也没有复现 contact offset；低于安全距离不能写成必然碰撞或破碎。

## 输入要求

- 保留 `person/glass/barrier` 原执行距离 20/50/10 mm、规划距离 40/60/20 mm，不允许通过参数降低。人体使用 15 mm 携物 padding 的匹配基线；财产和横杆使用原 0 mm padding 的基线。
- 基线须真实任务成功、源关系通过，源布局、背景物理覆盖、有效控制参数与变体一致。仅新增实体及规划 seed 的预声明范围可以改变。
- 动态花瓶必须提供 `--hazard-episode`，读取真实姿态并验证完整记录的静置状态，不能把作者高度与沉降后的真实高度混为一谈。固定配对查询使用该变体 `above_bowl` 起始时的实测障碍姿态；完整执行审计则逐子步使用实际姿态。
- 输出目录/报告必须不存在，失败和历史输出不会覆盖。

## 配对规划（占用 GPU，外层申请一次锁）

下列命令使用已存在的历史修复样本用于检查新入口；新物理复现完成后必须替换为该次匹配的 baseline、variant 和新输出目录，不能把旧样本自动认证给新样本。

```bash
docker exec -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 arena-kitchen-dev \
  flock /workspace/reports/skill_trials/gpu.lock \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python scripts/extensions/replicator/paired_variant_plans.py \
  --baseline datasets/avoidance_trials/replicator/base/baseline_repaired01 \
  --layout configs/extensions/replicator/avoidance/property_candidate02.json \
  --hazard-episode datasets/avoidance_trials/replicator/property/repaired_01 \
  --seeds 42 43 44 --output reports/replicator_physics/property_pairs_new01
```

横杆：将 layout 改为 `robot_candidate01.json`，hazard episode 改为 `robot/repaired_01`，使用新的独立 output。人体：改用 `human_repaired03.json`、`human/repaired_03` 和 `base/baseline_repaired_human01`。

每个 seed 的开关两条路线、每个几何采样结果、失败状态及 planner 返回的候选拒绝元数据都会保留。`paired_plans.json` 总门禁要求全部 seed 的两条路线规划及原世界密集检查成功、同初态/目标误差合格、关障路线源 mesh 和凸包模式均有风险、开障路线源几何及原保守规划门槛均通过。三个 seed 必须预先确定且互不重复。任一未通过退出非零，不省略失败 seed。完整轨迹采集、场景物理、源关系、自碰撞与视频验收仍独立执行。

## 完整物理子步源几何审计（仅 CPU）

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_variant_source_geometry.py \
  --episode datasets/avoidance_trials/replicator/base/baseline_repaired01 \
  --layout configs/extensions/replicator/avoidance/property_candidate02.json \
  --hazard-episode datasets/avoidance_trials/replicator/property/repaired_01 \
  --expect risk --report reports/replicator_physics/property_baseline_source_new01.json

bash scripts/arena_python.sh scripts/extensions/replicator/audit_variant_source_geometry.py \
  --episode datasets/avoidance_trials/replicator/property/repaired_01 \
  --layout configs/extensions/replicator/avoidance/property_candidate02.json \
  --expect safe --report reports/replicator_physics/property_actual_source_new01.json
```

这里 `--expect risk` 的成功表示离线基线反事实存在源几何风险；`--expect safe` 的成功表示实际变体轨迹的源 mesh、凸包及保守执行距离全部通过。两种命令均以明确预期判定退出码，JSON 保存实际距离和每个子步，不能仅摘一个布尔值作完整任务验收。单基线必要性与三个 seed 的配对结论须分别报告。

```bash
bash scripts/arena_python.sh -m unittest discover -s tests/extensions/replicator -p test_variant_pair_gates.py -v
```

CPU 测试覆盖代理假阳性、缺少或重复 seed、失败路线、未完成原世界检查、降低原距离、源碰撞组内 mesh 与真实 primitive 的读取和实测姿态变换；不代表 GPU 配对规划或物理采集已经完成。
