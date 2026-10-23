---
name: robolaw-replicator
description: 为RoboLaw学生A独立扩展Replicator 全部14个抓放环境，完成障碍物增广、cuRobo执行、少量采集与验证；用于该组端到端开发，不负责其他组任务或200条批量采集。
---

# 学生A：Replicator 全部14个抓放环境

**开始开发、复现或验收前必读 [本次复查与验收要求](references/review-gates.md)。历史 12/12 不再作为交付验收；原任务成功、场景物理质量与避障必要性须分列。**

独立负责 **14个原始环境**；新分支 **ylj-replicator**。完整任务ID、原始YAML和源码哈希见 [assignment.json](references/assignment.json)。先读 [workflow.md](references/workflow.md)，它定义代码边界和验收要求；优先按 [avoidance-trial.md](references/avoidance-trial.md) 的真实入口、无障碍控制和强制因果验收复现 G 型 mustard→red bowl 首任务。本技能自包含，不依赖其他学生产出。三条旧记录因背景冰箱自主运动不用于交付。修复后已重新完成两种基线与三个变体，40s中性检查通过；人体另有三个seed转运段配对，财产/横杆限单基线证据。最新场景修复、人体重新摆位与配对实测见 [repair-20261022.md](references/repair-20261022.md)。历史 baseline、参数和审计见该实测页与 coverage，当前交付结论以 review-gates.md 为准。其余13个原始任务未实测。[verified-trial.md](references/verified-trial.md) 仅保留第一轮 skill_trials 三条物理场景历史，不能将它们当作有效避障样本。

## 首次执行

在用户指定的RoboLaw仓库中运行以下真实预检（把技能目录替换成实际路径；无需Isaac/GPU）：

```bash
python3 /path/to/robolaw-replicator/scripts/preflight.py --repo /path/to/RoboLaw
```

预检通过仅代表分配YAML的内容/ID与依赖版本声明一致，不代表整个上游源码或运行时已验证。可用本技能的 `scripts/runtime_inventory.py` 在实际Python解释器下只读列出包版本（不导入CUDA/Kit，也不写仓库报告）。若技能放在仓库student_skills下，可用 `bash scripts/arena_python.sh student_skills/robolaw-replicator/scripts/runtime_inventory.py` 检查容器解释器；先确认ARENA_CONTAINER和挂载。包元数据存在不代表原生库可加载或GPU可运行。先列出缺失资产和运行时，再复现已实现的首个任务 `kitchen_bench_replicator_g_shape_mustard_bowl`。用户只要求审查/预检时，到该范围为止；用户要求开发时，继续完成本组端到端实现，不停留在计划或占位文件。

首任务规划清单可用下面的真实命令生成；它保留源任务关系，但还不是可运行场景。输出文件已存在时会拒绝覆盖，请换新文件名：

```bash
python3 /path/to/robolaw-replicator/scripts/preflight.py --repo /path/to/RoboLaw --task kitchen_bench_replicator_g_shape_mustard_bowl --emit-skeleton /tmp/robolaw-replicator-first-task.json
```

## 本组实施重点

先做 G 型 mustard_bowl，无障碍基线通过后做三类代表变体，再扩展其余四种厨房的 mustard_bowl；接着做四种 banana_bowl，最后五种 bowl_sink。五种厨房包括 G、L 带中岛、L、半岛、U；半岛没有 banana_bowl，禁止补造第15个上游环境。
保留原任务 robot stand_height、朝向、支撑面关系；固定初始布局须由原 OnRelationValidator/NextToValidator 验证，台架原点不能视为足迹中心。每条源关系、固定台架和真实q自碰撞独立sidecar均须通过。瓶、香蕉、碗分别有抓取区域；水槽是厨房内部目标区域，不是可以夹持的独立碗。目标落点、物体携带包络和放置后支撑状态按资产实际几何计算。每种厨房的碰撞世界、台面高度、world/local 坐标都重新验证。不要把一字型厨房的绝对坐标直接复制进去。

## 代码位置与必读实现

只在以下本组目录新增实现，公共基线保持可回归：

- `kitchen_safety/extensions/replicator/`
- `configs/extensions/replicator/`
- `scripts/extensions/replicator/`
- `tests/extensions/replicator/`
- `assets/extensions/replicator/`
- `docs/extensions/replicator/`

先读原始YAML，再参考：

- `kitchen_safety/environment.py`
- `kitchen_safety/curobo_backend.py`
- `kitchen_safety/collision_planner.py`
- `scripts/collect_kitchen.py`
- `scripts/validate_collection.py`
- `scripts/audit_collection_clearance.py`
- `kitchen_safety/grasp_geometry.py`
- `kitchen_safety/placement_geometry.py`
- `kitchen_safety/guided_transport.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/assets/background_library.py`

G型首任务已有真实 `scripts/extensions/replicator/run_trial.py` 入口；从包含它的ylj统一提交分支开始复现。其余任务的入口与配置仍须由本组实现，不能把设计中的路径/命令当现有功能。无GPU时可先完成清单、资产引用核对、配置与CPU级测试，但标记物理采集未验证。最终按workflow中的小规模验收标准提交自己的分支。
