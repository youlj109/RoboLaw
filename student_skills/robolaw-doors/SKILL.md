---
name: robolaw-doors
description: 为RoboLaw学生C独立扩展Lightwheel 全部8个开门环境，完成障碍物增广、cuRobo执行、少量采集与验证；用于该组端到端开发，不负责其他组任务或200条批量采集。
---

# 学生C：Lightwheel 全部8个开门环境

独立负责 **8个原始环境**；新分支 **ylj-doors**。完整任务ID、原始YAML和源码哈希见 [assignment.json](references/assignment.json)。先读 [workflow.md](references/workflow.md)，它定义代码边界、现有缺口和验收要求。本技能自包含，不依赖其他技能或学生产出。

首个 one-wall `open_fridge` 已于2026-10-21完成无障碍基线及人体、财产、横杆各1条真实采集；这些是场景/接触/视频冒烟测试，**原三个摆放不证明有效避障**。开发或复现首任务时，继续读 [physical-trial.md](references/physical-trial.md) 与 [avoidance-trial.md](references/avoidance-trial.md)：后者定义固定控制参数的反事实验收和修正入口。本轮已完成三类各1条新的有效避障采集：真实几何反事实违规、新实际120Hz安全、原任务与视频门控全部通过，正式路径与数值见avoidance-trial.md。仍只验证了1个原任务的固定布局，**其余7个任务仍未验证**。

## 首次执行

在用户指定的RoboLaw仓库中运行以下真实预检（把技能目录替换成实际路径；无需Isaac/GPU）：

```bash
python3 /path/to/robolaw-doors/scripts/preflight.py --repo /path/to/RoboLaw
```

预检通过仅代表分配YAML的内容/ID与依赖版本声明一致，不代表整个上游源码或运行时已验证。可用本技能的 `scripts/runtime_inventory.py` 在实际Python解释器下只读列出包版本（不导入CUDA/Kit，也不写仓库报告）。若技能放在仓库student_skills下，可用 `bash scripts/arena_python.sh student_skills/robolaw-doors/scripts/runtime_inventory.py` 检查容器解释器；先确认ARENA_CONTAINER和挂载。包元数据存在不代表原生库可加载或GPU可运行。先列出缺失资产和运行时，再开始实现首个任务 `kitchen_bench_lightwheel_open_fridge`。用户只要求审查/预检时，到该范围为止；用户要求开发时，继续完成本组端到端实现，不停留在计划或占位文件。

首任务规划清单可用下面的真实命令生成；它保留源任务关系，但还不是可运行场景。输出文件已存在时会拒绝覆盖，请换新文件名：

```bash
python3 /path/to/robolaw-doors/scripts/preflight.py --repo /path/to/RoboLaw --task kitchen_bench_lightwheel_open_fridge --emit-skeleton /tmp/robolaw-doors-first-task.json
```

## 本组实施重点

先原 one-wall 冰箱和微波炉，再 G 型冰箱/冷冻室/微波炉，再 U 型冰箱/橱柜，最后 G 型烤箱。清单中的8个完整ID为唯一范围。
从 USD 与原任务配置读取门关节轴、原点、限位、把手、reset_openness 和 openness_threshold；区分侧开与下翻，不按名称猜轨迹。cuRobo做接近/撤离与受约束分段IK，接触阶段按实际门角闭环更新下一步；同步更新门板、把手、机器人、人体与新增物体的碰撞世界。只允许指定手指-把手接触，不能忽略整扇门/整件家电。移动门板自身也必须避开人体/财产障碍。
原 OpenDoorTask 要先离开重置位置，再达到开启阈值，记录过程和最终关节状态。不能在采集中直接设置门关节状态来假装成功；初始化reset允许遵循上游。把手打滑、接触丢失和门的扫掠碰撞都算失败。接触力仅在传感器数据可用且标明单位时记录，不伪造力证据。

## 本组CPU碰撞覆盖验收

独立试用在固定版本资产中确认：首个one-wall冰箱门的37个碰撞体全部落在原CollisionWorld工作区裁剪之外。这个问题在不更换厨房的情况下就会发生。修复时遍历该任务真实USD，比较有效门体/把手碰撞prim与规划世界收录项，验证覆盖完整及坐标变换正确；不能仅把硬编码上界放大一点，也不能把“37”当所有门的固定数量。证据见 [first-door-audit.json](references/first-door-audit.json)，它只证明CPU几何检查，不证明物理执行。报告使用原资产坐标，控制时须按实际spawn变换重算并正确归一化关节轴，不直接复制报告坐标。

## 代码位置与必读实现

只在以下本组目录新增实现，公共基线保持可回归：

- `kitchen_safety/extensions/doors/`
- `configs/extensions/doors/`
- `scripts/extensions/doors/`
- `tests/extensions/doors/`
- `assets/extensions/doors/`
- `docs/extensions/doors/`

先读原始YAML，再参考：

- `kitchen_safety/environment.py`
- `kitchen_safety/curobo_backend.py`
- `kitchen_safety/collision_planner.py`
- `scripts/collect_kitchen.py`
- `scripts/validate_collection.py`
- `scripts/audit_collection_clearance.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/tasks/open_door_task.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/tasks/rotate_revolute_joint_task.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/affordances/openable.py`

首个冰箱的真实入口为 `scripts/extensions/doors/collect.py`，旧smoke配置为 `configs/extensions/doors/open_fridge_variants.json`；有效避障配置为 `configs/extensions/doors/avoidance_variants.json`，必须同时恢复其中的原基线布局、支座和同源人体资产；先确认当前源码包含这些文件，ZIP本身不含采集器和大资产。该入口尚不支持其余7个任务、resume或批量成功配额；这些接口仍需本组实现，不能把设计目标当现有功能。无GPU时可先完成清单、资产引用核对、配置与CPU级测试，但标记新的物理采集未验证。最终按用户当前要求与workflow中的小规模验收标准提交自己的分支。

关节比例与限位读取还须阅读 `third_party/IsaacLab-Arena/isaaclab_arena/utils/joint_utils.py` 和 `third_party/IsaacLab-Arena/isaaclab_arena/tasks/predicates/articulations.py`，实际路径先用rg核对。

对关节进度转换做CPU边界测试：重置点、恰好阈值、刚超过阈值、负下限和反向轴；继承源predicate的严格比较和先离开重置点的顺序。不要把q/upper_limit当通用开启比例。
