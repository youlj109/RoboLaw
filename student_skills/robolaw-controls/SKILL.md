---
name: robolaw-controls
description: 为RoboLaw学生D独立扩展Lightwheel 左右按钮与温度旋钮3个环境，完成障碍物增广、cuRobo执行、少量采集与验证；用于该组端到端开发，不负责其他组任务或200条批量采集。
---

# 学生D：Lightwheel 左右按钮与温度旋钮3个环境

**开始开发、复现或验收前必读 [本次复查与验收要求](references/review-gates.md)。历史 12/12 不再作为交付验收；原任务成功、场景物理质量与避障必要性须分列。**

独立负责 **3个原始环境**；由学生自己从 `origin/ylj` 创建以**本人姓名缩写**命名的分支（例如 `zxw`、`lgd`、`lwx`）。姓名未明确时先询问该学生，不按A/B/C/D猜测对应关系；`ylj`仅作为共同基线。完整任务ID、原始YAML和源码哈希见 [assignment.json](references/assignment.json)。先读 [workflow.md](references/workflow.md)，它定义代码边界、现有缺口和验收要求。本技能自包含，不依赖其他技能或学生产出。

左取消按钮已完成一轮机构及三类避障布局修复；开始该任务先读 [修复后的实测与复现](references/repaired-left-button.md)，再按 review-gates.md 核查当前证据。历史三条的旋钮自转与弱避障结论仍保持撤回，不能按旧 accepted 交付。右按钮与旋钮任务仍未验证。

原首轮三条成功只证明物理任务和采集流程；人体及横杆未使原路径失效，不得标记为有效避障增广。新增安全样本必须先读 [避障设计与对照验收](references/avoidance-design.md)，用同源布局、同控制参数的无障碍成功轨迹证明新增实体确实造成安全距离违规，再验证重规划后的真实执行。场景里出现实体、规划器成功或路径不同都不能替代这项证据。

## 首次执行

在用户指定的RoboLaw仓库中运行以下真实预检（把技能目录替换成实际路径；无需Isaac/GPU）：

```bash
python3 /path/to/robolaw-controls/scripts/preflight.py --repo /path/to/RoboLaw
```

预检通过仅代表分配YAML的内容/ID与依赖版本声明一致，不代表整个上游源码或运行时已验证。可用本技能的 `scripts/runtime_inventory.py` 在实际Python解释器下只读列出包版本（不导入CUDA/Kit，也不写仓库报告）。若技能放在仓库student_skills下，可用 `bash scripts/arena_python.sh student_skills/robolaw-controls/scripts/runtime_inventory.py` 检查容器解释器；先确认ARENA_CONTAINER和挂载。包元数据存在不代表原生库可加载或GPU可运行。先列出缺失资产和运行时，再开始实现首个任务 `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian`。用户只要求审查/预检时，到该范围为止；用户要求开发时，继续完成本组端到端实现，不停留在计划或占位文件。

首任务规划清单可用下面的真实命令生成；它保留源任务关系，但还不是可运行场景。输出文件已存在时会拒绝覆盖，请换新文件名：

```bash
python3 /path/to/robolaw-controls/scripts/preflight.py --repo /path/to/RoboLaw --task kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian --emit-skeleton /tmp/robolaw-controls-first-task.json
```

## 本组实施重点

先左取消按钮，再右取消按钮，最后烤箱温度旋钮；难点在接触精度，不以任务数平均工作量。无需等待学生C提供开门控制器；在本组目录独立实现最小接触执行流程。
按钮分别绑定 sideL_button_cancel_joint 和 sideR_button_cancel_joint，继承原 is_pressed 判定；YAML描述与源码predicate都要核对，不擅自增加或降低成功阈值。接近时保持间隙，接触时沿真实按压轴小步推进，读取按钮行程并判断成功；限制其他夹爪/手臂部分与烤面包机碰撞。
旋钮绑定 knob_temp_joint，原目标 level 3，reset_level=-1；从 affordance 读取角度到档位的映射，不把目标档位直接当弧度。抓持后沿旋钮轴转动，校验实际关节变化、接触保持及撤离。不能直接赋值旋钮关节以完成任务。cuRobo负责自由空间和分段IK，不应宣称它自带可靠力控；有需要时实现并验证接触反馈控制。
按钮和旋钮 get_mimic_env_cfg() 在本固定源码里未实现；自己实现采集器，不将 Mimic 当现成依赖。

## 代码位置与必读实现

只在以下本组目录新增实现，公共基线保持可回归：

- `kitchen_safety/extensions/controls/`
- `configs/extensions/controls/`
- `scripts/extensions/controls/`
- `tests/extensions/controls/`
- `assets/extensions/controls/`
- `docs/extensions/controls/`

先读原始YAML，再参考：

- `kitchen_safety/environment.py`
- `kitchen_safety/curobo_backend.py`
- `kitchen_safety/collision_planner.py`
- `scripts/collect_kitchen.py`
- `scripts/validate_collection.py`
- `scripts/audit_collection_clearance.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/tasks/press_button_task.py`
- `third_party/IsaacLab-Arena/isaaclab_arena/tasks/turn_knob_task.py`

先核实本组试用入口是否随当前仓库版本存在；缺失的入口、右按钮和旋钮流程仍需本组实现，不能把参考中的路径当已存在功能。无GPU时可先完成清单、资产引用核对、配置与CPU级测试，但标记物理采集未验证。最终按workflow中的小规模验收标准提交自己的分支。

## 经独立试用确认的判定细节

- 按钮YAML写“press and hold”，但当前固定源码的成功predicate只有 `is_pressed`，没有持续时间条件。保留原predicate，并记录实际按压持续时间；若加稳定保持检查，单独报告，不能说它是上游已有阈值。
- `affordances/turnable.py` 的注释说rounding，实际 `get_turning_level` 使用 `torch.floor`。以固定源码的实际行为验收，增加档位边界CPU测试，不擅自改上游算法。当前30°–150°、5档配置下，level 3数学区间约为120°到150°之前，实际判断须考虑关节坐标与浮点精度；别把档位3解释成3弧度或第三个从1计数的档位。
- `turn_to_level` 内部直接设置关节状态，只用于原任务初始化/reset；执行采集时不能调用它跳到目标档位。真实旋转必须由机器人接触动作产生。
- 必须阅读 `third_party/IsaacLab-Arena/isaaclab_arena/affordances/pressable.py`、`affordances/turnable.py`（后者同一isaaclab_arena根目录）和 `utils/joint_utils.py`，确认归一化、正负方向和边界。
