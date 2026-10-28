# 有效避障：强制因果对照与修正记录

**2026-10-22 修复轮：先读 [本组最新修复实测](repair-20261022.md)。三类旧样本因共享背景冰箱自主运动而撤回场景质量验收；修复后两种基线与三类新样本已完成，确切路径、当前命令与范围见最新修复页。**

**2026-10-22 复查：先读 [当前交付验收](review-gates.md)。下面保存旧单基线试验的参数和报告，历史 accepted 不等于当前交付通过。**

本组芥末瓶三条的真实抓放完成仍有记录支持。但 human/attempt_04 的“必要性”未证实：盒代理给出 15.382 mm，而独立整瓶源顶点凸包加机器人/夹爪碰撞网格复核，原基线全部 3848 个物理子步最小 21.047 mm，高于原 20 mm 要求，33 个代理违规消失。没有校验 PhysX cooked shape/contact offsets，不能反向称其绝对无风险。该人体摆位须重设计、重采；property/robot 仍保留单基线的安全裕度/几何相交证据，不能升级为稳定必要性。

第一轮 skill_trials 的三条只证明场景/真实物理/RGB完成，不能当成有效避障。当前修正试验在 `datasets/avoidance_trials/replicator/`，失败与旧数据全部保留。本轮仍只覆盖 G 型 mustard→red bowl，即 1/14 个源任务；其余13个未验证。

有效性门槛必须同时成立：

1. 同原任务、同原始 reset 布局与成功谓词，先采真实成功无障碍baseline，不能拿源关系不合法的旧 diagnostic_05 做对照。
2. baseline与变体的seed、真实抓取/放置目标、抓取参数、有效规划配置一致。本组human有原15mm携物padding，因此须用 `--variant base --control-profile human` 的专用无人体控制，不能偷换成零padding的base。
3. 对baseline真实关节、夹爪、物体位姿离线逐物理子步检查，加入障碍后中间段违反既有间隙或发生几何相交；严禁实际执行危险对照。
4. 新障碍对厨房、固定台架、任务对象无非法初始交叠；原始任务目标可行。原任务布局和安全阈值不能为制造阻挡而修改。
5. 新轨迹由cuRobo重规划并真实物理完成，逐子步外部安全、原谓词、源关系、台架、自碰撞、原三路RGB及独立overview全通过。
6. 每条 `avoidance_audit.json` 明确accepted、baseline路径/哈希、同源布局/有效控制、违规部位/帧/距离/实际常量、原与新路径比较。轨迹不同本身不是有效性证据。

G型已实现入口：`run_trial.py --trial-root avoidance_trials --layout-json <候选JSON> --overview-camera`；`--layout-json` 逐项禁止改变 obstacle 之外的原布局/规划字段。可加 `--baseline-episode <真实控制目录>` 指定对照。runner只有独立避障审计accepted和退出码均通过才验收。`search_avoidance.py` 是CPU候选扫描，`probe_property_proxy.py` 是cuRobo球/膨胀碰撞体诊断，均不代替物理成功；`audit_avoidance.py` 做真实baseline回放、有效参数比对并保存path_comparison.png。

规划内部路点与任务目标要分清。本组局部 `curobo_backend.py` 对所有场景使用同一规则：优先原pregrasp高度/垂直lift；原路点不安全时，搜索内部悬停高度并检查到原抓取点的完整approach，或先验证25mm垂直脱离台面，再从固定的横向/高度候选集合自动选择内部lift目标并由cuRobo绕行。保留原35°/180°抓取、抓放目标、28°携物姿态约束和全部间隙。保存被拒候选、选中路点、0条被拒动作执行；无障碍baseline须使用同策略。没有放宽碰撞检查或以FCL-RRT替代cuRobo路径。

历史审计状态：human/attempt_04、property/attempt_02、robot/attempt_01 曾记录 accepted=3/3；复查已撤回 human 的真实几何必要性结论，其余两条限于单基线证据。覆盖仍为1/14个原始任务。不得把第一轮物理样本补作缺失的有效避障样本。

对照距离来自原有安全模型：机械臂capsule、机器人mesh包络、夹爪和携物包络，以及原障碍几何。模型间隙不足/包络相交不得夸大为真实精确表面必然接触。最终数据另外必须通过实际PhysX接触与独立物理检查。旋转primitive候选须从最终姿态重建CollisionWorld，避免先原点建world-AABB再旋转造成与live建模不一致。

104个原背景刚体也须独立比较实际初始/settled位姿，不能只比名义布局。`audit_background_consistency.py` 对有意义的settling差异作双轨迹×双背景检查，`background_consistency_audit.json passed=true` 是硬gate。当前property两轨迹对变化背景全部物理历史的保守扫掠包络最小仍245.374mm；robot控制与变体104体初始物理pose完全相同。

CPU回归的真实命令（需要容器内FCL/Usd，但不启动GPU/Kit）：
```bash
bash scripts/arena_python.sh -m unittest discover -s tests/extensions/replicator -v
```
不要使用缺少FCL的host python运行新增measurement tests。6项回归已实际通过。

## 历史单基线样本与复现

正式目录均位于 `datasets/avoidance_trials/replicator/`，对应独立报告位于 `reports/avoidance_trials/replicator/`。原三路视频、all_cameras和单独overview都保留；根汇总额外生成同视角、原时间速度的comparison.mp4，不以加速/同步错位制造效果。

| 变体 | 样本 | 配对无障碍控制 | 动作帧/物理子步 | 原轨迹加入危险物体后的模型最小间隙 | 新轨迹模型最小间隙 | 执行要求 |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| human | human/attempt_04 | base/baseline_human_03 | 515 / 3880 | 15.382mm，33个违规子步 | 40.479mm | 20mm |
| property | property/attempt_02 | base/baseline_01 | 525 / 3960 | 37.400mm，102个违规子步 | 111.670mm | 50mm |
| robot | robot/attempt_01 | base/baseline_01 | 512 / 3856 | 0mm，165个违规子步 | 45.352mm | 10mm |

采样为动作/RGB 15Hz、物理120Hz。原轨迹最小值来自真实baseline的全部保存物理子步；新轨迹表中最小值来自动作状态独立审计，完整物理子步另按同一执行阈值全部验证。名义源布局保持一致；观测状态需单独比较：robot控制与变体完全一致，human初始关节/机器人/瓶完全一致、碗角差约0.000005°，property任务物体settling差小于2微米/0.003°，不能将它们写成比特完全相同。property从首个记录物理子步起的位移最大0.680微米、旋转0.000407°；全部3960帧通过2mm/2°及直立限制。静态人体/横杆以非RigidBody USD、无动画时间采样及实测settled根变换证明固定，npz中零pose占位不是动态观测。

新布局只改obstacle：human `[1.60,-0.55,0]` yaw40°，property `[0.625,-0.59,0.896899501]` yaw0°，robot `[0.61,-0.84,0.896899501]` yaw180°。原厨房、芥末瓶、红碗、机器人及全部7条上游关系不变。人体候选由粗/细/东西两侧CPU网格筛选，最终原pregrasp、抓取、lift和放置端点合法，只约束中间转运；候选04逐动作原轨迹有17个转运状态低于40mm规划间隙，其中最小15.588mm低于20mm执行间隙。

下面命令已将人体配置更新为修复后的 human_repaired03；最新完整流程（含中性/可动性/三seed配对）见 repair-20261022.md。它们在当前实现下重新生成配对控制与样本。每个输出名必须尚不存在；不要把旧baseline_01与当前v2策略的新采样混用，验收会拒绝有效控制不一致。

```bash
python3 student_skills/robolaw-replicator/scripts/preflight.py --repo .
python3 scripts/extensions/replicator/restore_assets.py
python3 scripts/extensions/replicator/verify_assets.py
bash scripts/arena_python.sh -m unittest discover -s tests/extensions/replicator -v
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --trial-root avoidance_trials --variant base --attempt repro_base --overview-camera
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --trial-root avoidance_trials --variant base --control-profile human --attempt repro_human_base --overview-camera
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --trial-root avoidance_trials --variant human --attempt repro_human --layout-json configs/extensions/replicator/avoidance/human_repaired03.json --baseline-episode datasets/avoidance_trials/replicator/base/repro_human_base --overview-camera
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --trial-root avoidance_trials --variant property --attempt repro_property --layout-json configs/extensions/replicator/avoidance/property_candidate02.json --baseline-episode datasets/avoidance_trials/replicator/base/repro_base --overview-camera
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --trial-root avoidance_trials --variant robot --attempt repro_robot --layout-json configs/extensions/replicator/avoidance/robot_candidate01.json --baseline-episode datasets/avoidance_trials/replicator/base/repro_base --overview-camera
```

本机入口使用HEADLESS=1和共享 `reports/skill_trials/gpu.lock`，一条有界采集后释放锁；不终止他人任务。`--no-rgb`只用于诊断，不能完成正式验收。先确认自己的容器挂载和arena_python.sh，CPU几何也需要容器FCL/Usd；资产恢复可用能读USDC的host pxr。248源资产、真实MDL依赖、104动态背景、源关系/默认支座的细节仍见历史页，但旧物理样本不能替代本轮避障证据。

## 保留的失败与限制

本轮human01/02在悬停规划被拒，0个任务动作执行；human03在获得GPU锁前因更好CPU候选出现而取消，明确记录0动作取消，不当成失败仿真或成功样本。候选03横向lift目标仅做CPU预检；不要宣称备选lift分支已物理验证。最终candidate04优先使用原pregrasp/lift目标，障碍感知规则仍保留拒绝候选记录。property01真实抓取后lift预检拒绝，保留失败；只将花瓶改为candidate02，经精确cuRobo覆盖球/膨胀碰撞体CPU检查再重采，安全阈值未放宽。被拒绝的路径不执行。

独立审计修订保留 `avoidance_audit_v1.json`、v2、v3和revision记录；v2纠正unsafe baseline的passed语义，并增加实际初态/旋转检查；v3增加源背景差异排除。没有覆盖原视频、图或物理记录。有效性只针对本次固定单样本；不能扩展成总体成功率、人体伤害/器物破碎模型、200条批采、其他13原任务或多环境最大速度结论。

历史名单与当时门禁（不代表本次新增验收已通过）：`docs/extensions/replicator/avoidance_coverage.json`、`reports/avoidance_trials/replicator/summary.json`。每条当前summary包含collection/source_relations/fixed_base/export/validation/audit/self_collision/overview/background_consistency/avoidance共10个退出码全0；两条有效控制的完整RGB/物理/源关系/self/overview同样通过。human04全子步反事实最小15.381528mm，新轨迹动作率最小40.478707mm；原pregrasp高度与lift仍是默认目标，无需使用横向lift备选分支。
