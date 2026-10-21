---
name: robolaw-lightwheel-pick-place
description: 为RoboLaw学生B独立扩展Lightwheel 剩余5个抓放环境，完成障碍物增广、cuRobo执行、少量采集与验证；用于该组端到端开发，不负责其他组任务或200条批量采集。
---

# 学生B：Lightwheel 剩余5个抓放环境

独立负责 **5个原始环境**；新分支 **ylj-lightwheel-pick-place**。完整任务ID、原始YAML和源码哈希见 [assignment.json](references/assignment.json)。先读 [workflow.md](references/workflow.md)，它定义代码边界、现有缺口和验收要求。本技能自包含，不依赖其他技能或学生产出。

本仓库已实现青柠任务的本组场景、cuRobo/物理执行与采集入口，并完成human/property/robot固定布局各1条真实采集。继续该任务前读 [physical-trial.md](references/physical-trial.md)：它给出最终目录与验收证据，记录本厨房资产、抓取高度、真实携物凸包、源支持接触和额外人体总览问题。技能 ZIP 只含指导与预检脚本；需使用包含本组实现的 RoboLaw 源码，不能仅安装技能就声称执行器可用。其余4个任务尚未完成物理验证。

## 首次执行

在用户指定的RoboLaw仓库中运行以下真实预检（把技能目录替换成实际路径；无需Isaac/GPU）：

```bash
python3 /path/to/robolaw-lightwheel-pick-place/scripts/preflight.py --repo /path/to/RoboLaw
```

预检通过仅代表分配YAML的内容/ID与依赖版本声明一致，不代表整个上游源码或运行时已验证。可用本技能的 `scripts/runtime_inventory.py` 在实际Python解释器下只读列出包版本（不导入CUDA/Kit，也不写仓库报告）。若技能放在仓库student_skills下，可用 `bash scripts/arena_python.sh student_skills/robolaw-lightwheel-pick-place/scripts/runtime_inventory.py` 检查容器解释器；先确认ARENA_CONTAINER和挂载。包元数据存在不代表原生库可加载或GPU可运行。先列出缺失资产和运行时，再开始实现首个任务 `kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian`。用户只要求审查/预检时，到该范围为止；用户要求开发时，继续完成本组端到端实现，不停留在计划或占位文件。

首任务规划清单可用下面的真实命令生成；它保留源任务关系，但还不是可运行场景。输出文件已存在时会拒绝覆盖，请换新文件名：

```bash
python3 /path/to/robolaw-lightwheel-pick-place/scripts/preflight.py --repo /path/to/RoboLaw --task kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian --emit-skeleton /tmp/robolaw-lightwheel-pick-place-first-task.json
```

## 本组实施重点

顺序：青柠入木碗 → 百事入篮子 → 架上柠檬到盘子 → 双香蕉入木碗 → 汤勺到盘子。原 one-wall mustard pick_and_place 是回归范例，不计入这5个环境，不归你新增实现。
双香蕉上游 composition=parallel 表示两个子目标均需完成、顺序不限；不是多环境并行。单臂可依次抓放，完成第一根后更新碰撞世界，最终两根都要满足原始谓词。保留原任务其他桌面物品。架上柠檬检查架体、取出方向与高度；汤勺检查细长物体握持段、姿态与盘面支撑。禁止把所有目标统一当作深碗，或以瓶身夹持阈值判定香蕉/汤勺。

## 代码位置与必读实现

只在以下本组目录新增实现，公共基线保持可回归：

- `kitchen_safety/extensions/lightwheel_pick_place/`
- `configs/extensions/lightwheel_pick_place/`
- `scripts/extensions/lightwheel_pick_place/`
- `tests/extensions/lightwheel_pick_place/`
- `assets/extensions/lightwheel_pick_place/`
- `docs/extensions/lightwheel_pick_place/`

先读原始YAML，再参考：

- `kitchen_safety/environment.py`
- `kitchen_safety/curobo_backend.py`
- `kitchen_safety/collision_planner.py`
- `scripts/collect_kitchen.py`
- `scripts/validate_collection.py`
- `scripts/audit_collection_clearance.py`
- `kitchen_safety/grasp_geometry.py`
- `kitchen_safety/grasp_contacts.py`
- `kitchen_safety/placement_planner.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/tasks/pick_and_place_task.py`

青柠的现有真实入口是 `scripts/extensions/lightwheel_pick_place/run.sh`、`collect.py`、`validate.py`，实现位于本组目录；先检查源码与实测参考再复用。当前入口仅支持单环境、单次尝试和固定布局，没有批量成功配额或 resume。其他任务仍需本组适配，不能把接口目标当作已实现功能。无GPU时可先完成清单、资产引用核对、配置与CPU级测试，但标记物理采集未验证。最终按workflow中的小规模验收标准提交自己的分支。
