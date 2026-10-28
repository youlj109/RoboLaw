# 有效避障增广：强制验收

**2026-10-22 复查：先读 [当前交付验收](review-gates.md)。下面保存旧单基线试验的参数和报告，历史 accepted 不等于当前交付通过。**

本组青柠 human/property/robot 三条真实抓放完成有原始状态、双指接触和画面支持，单基线几何/裕度避让证据仍保留。human 主要靠冗余关节姿态调整，TCP 变化较小，展示应标出受约束的手臂部位，不能称为明显末端绕行。尚未完成下述多规划 seed 成对验证，不作稳定必要性或总体成功率结论。

旧 `datasets/skill_trials/lightwheel_pick_place/` 三条保留为物理采集流程验证。它们的基线反事实仍可行，不能作为有效避障样本。新证据放 `datasets/avoidance_trials/lightwheel_pick_place/`，失败记录另存同级尝试目录，不回填旧result。

## 控制对照

无障碍基线必须真实抓取、提升、放入木碗，原终态 `object_on_destination` 当前状态为真，完整物理/RGB验收通过。基线和三变体使用同 `fixed_layout.json` 的原源reset姿态、同复位关节、seed42、35度倾角/yaw0及源几何中心上15mm抓点。`--fixed-layout` 对base也可用，防止重新采样。`--variant-config` 仅选择新增物体；不移动原机器人/目标/容器来制造绕行。

若修改起步速度或其他控制参数，重新采匹配基线，并在 `avoidance_audit.json` 核对 `same_control_parameters`。障碍感知MotionGen可选择新路径或冗余IK姿态，保持原TCP抓点/朝向。默认禁用旧25/20度抓取fallback；用于诊断时应另开匹配对照，不能混入固定35度比较。

## 摆位与证据

`scan_avoidance.py` 只在CPU读实际轨迹筛候选。首先检查新增物体与所有原厨房/源物件及真实固定台架/base没有非支撑交叠，再确保任务初态和目标可行。正常扫法要求原接触段姿态也可行；`--endpoints-only` 只放开机器人冗余IK姿态，仍需另证同一抓点/朝向的接近和提升可行，不放宽碰撞距离。

最终 `audit_avoidance.py --baseline <成功基线> --episode <新变体>` 使用120Hz记录的真实arm、各指爪和lime姿态重放几何。**不实际执行危险反事实**。必须同时满足：

- `same_source_layout` 和 `same_control_parameters` 为真。相同源reset之外，首帧实测7臂关节差异必须≤1e-6 rad，lime和木碗实测初态必须在1mm/1度内；逐一报告所有源物体实际位置/姿态/速度差异，不能把名义reset相同说成物理状态完全相同。
- `baseline_counterfactual.passed` 为假（安全检查未通过），`violates_safety_margin` 与顶层 `avoidance_effective` 为真：基线在新增障碍下至少有一个实测样本违反既有执行间隙，新路径安全。保存风险阶段、最小距、所需距和所有样本。
- `augmented_actual.passed` 为真，实际新轨迹每个物理步安全；动态花瓶使用逐物理步实测hazard_pose，独立检查全程平移≤5mm、倾角≤2度；不宣称模拟了破碎。
- `initial_and_goal_feasible`、`hazard_environment_legal` 为真，任务原当前终态成功，独立 `validation.json`、全世界、自碰撞、固定台架、源支持接触与 `source_background_audit.json` 通过。后者将两条120Hz真实路径分别放入两套实测初始背景（木碗+全部三种源杂物）检查，且最终audit进一步减去所有15Hz实测源背景位姿的保守表面位移包络；任何背景差异单独使旧路不安全，都拒绝因果归因。
- 保留实际自由段q/TCP/携物轨迹差异和`path_comparison.png/pdf` 或同相机对比视频。随机路径不同不能代替基线危险与新轨迹安全这两个核心证据。

人体保留原三路RGB，同时使用独立同步 `--overview-camera` 展示可辨认的人与任务。先实际试拍；不能只凭视锥推断可见。原三路内外参、数据及合成视频不替换。

## 已完成的实测范围

共同严格无障碍基线为 `datasets/avoidance_trials/lightwheel_pick_place/base_attempt04`：481动作/3848物理步，原当前终态、原始RGB/接触/完整世界/源支持/自碰撞均通过。下表三条的历史独立门控均记录 `avoidance_audit.accepted=true`；新增中性保持/多 seed 门槛不能由该字段推断。

| 类别 | 正式episode | 原基线在新增实体下 | 新真实轨迹最小距离 | 动作 / 120Hz物理步 |
| --- | --- | --- | --- | --- |
| 人体 | human_attempt01 | 437步低于20mm，最小11.811mm | 38.065mm | 474 / 3792 |
| 财产 | property_attempt01 | 102步低于50mm，最小37.576mm | 115.972mm | 490 / 3920 |
| 障碍物 | robot_attempt03 | 181步低于10mm，真实Cylinder与旧路相交 | 55.854mm | 498 / 3984 |

每条均保留三路原RGB/视频、完整原始状态与接触、`path_comparison.png/pdf` 和配对视频；人体额外保留同步 `overview_camera.mp4`。障碍物最终使用明确新增的0.28m短悬臂资产，位置[4.095,-.12,.922]，配置为 `avoidance_variants_robot03.json`；实际transfer的lime/TCP路径相对基线最大改变134.903/124.339mm，前接近q仅差0.00147rad，改变集中在受横杆约束的搬运段。

三个样本都保留相同原源reset/任务/控制；非任务杂物的实际settling差异单独披露，并通过两条实际轨迹×两套实测源背景及全部记录位姿的运动包络检查，不能把名义布局相同称作每个物理位姿完全一致。财产花瓶全程平移约0.000504mm、倾角0.000239度，满足≤5mm/≤2度，未模拟破碎。

这只是每类1条有单基线证据的试采，不代表批量成功率或200条采集已完成。保留robot01不可规划、robot02闭爪源支持失败等历史；没有放宽原支持带或新增实体间隙来取得robot03。当前控制/源支持字段必须存在、非None且值相同，不能让两边缺失的None==None误过。旧 `base_attempt12` 缺新增当前终态字段，保留历史，不补写成新合格基线。其余4个分配任务仍未物理验证。

## 原源台面支持边界

本轮使用实际原生rest碰撞凸包底界减0.5mm（任意资产root原点高度不作为穿透量），限制精确原源台面、源XY半径5cm及接近/夹持/抬升阶段；允许尚未离台的物理微滚转。真实凸包底高于台面2mm且源接触消失后永久撤销该例外，不能掉回重新启用。逐physics保存原始源接触和凸包底，独立审计重算；人体/花瓶/横杆间隙不变。旧单调底界失败记录保留。源支持审计重新计算发生变化时另存 `.recheckNN.json`，不覆盖历史结果。

## 复现验收顺序

先对base及三个变体用 `validate.py <episode>` 重算原任务/接触/图像/源支持，并通过共享GPU锁运行 `audit_self_collision.py <episode>`。`audit_fixed_base.py <episode>` 只用于human/property/robot，当前实现不接受base；它验证新增实体与固定台架的关系，不替代base的完整厨房/源支持/自碰撞检查。三个变体再CPU运行 `audit_source_background.py --baseline <base> --episode <variant>`，最后运行 `audit_avoidance.py --baseline <base> --episode <variant>` 和 `plot_avoidance.py`（同参数）。基线同样需原任务、源支持、全部RGB/视频及每物理步自碰撞通过。未齐全的几何预审另命名，不可当最终accepted。

只在新采集目录首次执行上述验收。`validate.py`、`audit_fixed_base.py`、`audit_self_collision.py` 和 `plot_avoidance.py` 会写回同名报告或图片，并非自动保留历史；复审已有目录时先另存原报告/图与哈希，避免覆盖既有证据。`audit_source_background.py`、`audit_avoidance.py` 则拒绝覆盖已有默认输出，可按各自 `--help` 选择新的报告路径。

真实圆管反事实使用公共 `kitchen_safety.geometry_audit` 的解析Cylinder/Cube等几何，不用世界AABB角代替真实碰撞风险；执行仍保持既有保守世界检查。CPU摆位扫描的primitive AABB必须与live场景构建一致，非零yaw不能把旋转后的紧OBB当作live世界AABB。


## 明确新增资产的边界

若原障碍几何让同源任务起点/目标没有合法可行交集，可以新建明确命名的障碍资产变体，但不能静默缩小既有碰撞体。当前robot诊断新增 `assets/extensions/lightwheel_pick_place/short_projecting_crossbar.usda`：悬臂0.40m改为0.28m，圆管半径/底座/立柱不变，端盖和警示环同步移到真实短杆位置。原资产和失败episode保留。候选仍必须证明真实解析Cylinder会使原成功轨迹违规，并完成全部实际执行门控；不能把单纯变短或候选可规划算作避障成功。


原CLI的固定布置随机种子参数拼写为 `--placement_seed`（下划线），不是 `--placement-seed`。复现当前固定控制对照可用：

```bash
bash scripts/extensions/lightwheel_pick_place/run.sh collect.py --device cpu --variant base --seed 42 --placement_seed 42 --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout.json --output /workspace/datasets/avoidance_trials/lightwheel_pick_place/base_new
bash scripts/extensions/lightwheel_pick_place/run.sh collect.py --device cpu --variant human --seed 42 --placement_seed 42 --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout.json --variant-config configs/extensions/lightwheel_pick_place/avoidance_variants.json --overview-camera --output /workspace/datasets/avoidance_trials/lightwheel_pick_place/human_new
bash scripts/extensions/lightwheel_pick_place/run.sh collect.py --device cpu --variant property --seed 42 --placement_seed 42 --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout.json --variant-config configs/extensions/lightwheel_pick_place/avoidance_variants.json --output /workspace/datasets/avoidance_trials/lightwheel_pick_place/property_new
bash scripts/extensions/lightwheel_pick_place/run.sh collect.py --device cpu --variant robot --seed 42 --placement_seed 42 --fixed-layout configs/extensions/lightwheel_pick_place/fixed_layout.json --variant-config configs/extensions/lightwheel_pick_place/avoidance_variants_robot03.json --output /workspace/datasets/avoidance_trials/lightwheel_pick_place/robot_new
```

采集命令成功只代表物理任务出口；还必须按上面的完整独立验收顺序生成accepted，失败目录不可复用/覆盖。以上是既有单基线执行入口，尚不包含新版中性保持或三seed成对规划；当前缺项见 [review-gates.md](review-gates.md)，须先补齐相关实现，不能仅运行这四条采集命令就报告新版完整验收通过。
