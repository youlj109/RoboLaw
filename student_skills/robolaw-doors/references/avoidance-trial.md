# One-wall 冰箱：有效避障修正

2026-10-21 的 `datasets/skill_trials/doors/*` 三变体只证明原任务真实开门、物理安全和视频采集。旧障碍没有足够的必要性证据，不将它们计为有效避障。

最初CPU候选搜索以已成功的 `datasets/skill_trials/doors/base_attempt_08` 为参考：330 个动作帧，seed 42、把手高偏移 +0.12 m、相同原始布局和动作控制。CPU 搜索只读取真实测量 q，不执行危险轨迹。物理新样本写 `datasets/avoidance_trials/doors`，不得覆盖旧样本。正式反事实统一采用本轮重新采集、显式记录控制参数的baseline，路径见下方最终实测表。

## 判定与入口

- `scripts/extensions/doors/search_avoidance_layouts.py`：沿原轨迹搜索，检查原厨房/实际台架/门扫掠；`--pedestal` 的支柱整个实体都检查。输出是候选，不是通过。
- 正式复现使用已保存的 `configs/extensions/doors/avoidance_variants.json` 和其中的小型支座/wrapper资产；需要时按下文命令重建同源人体。`build_avoidance_pedestals.py` 是早期探索工具，会生成历史候选配置，不能拿它覆盖最终配置。新布局另存配置，并重新验收。
- `scripts/extensions/doors/collect.py --variant property --adaptive-handle-height --variant-config configs/extensions/doors/avoidance_variants.json --output datasets/avoidance_trials/doors/<new_attempt>`：沿用原控制参数，cuRobo 对新增几何重规划。人体同样传 `--adaptive-handle-height`；横杆不传。无障碍base必须用与对应增广样本相同的flag。通过环境 `HEADLESS=1` 启动；本 CLI 没有 `--headless`。实际 Python/容器入口按本机部署配置。
- 依次运行 `validate_trial.py`、`audit_self_collision.py`、`audit_avoidance.py`，均传实际 episode 路径。`audit_avoidance.py` 同时输出 hazard clearance 与固定台架补充报告。cuRobo 自碰撞复核使用保存的每个 120 Hz 物理状态，新样本不可只抽 15 Hz。共享 GPU 使用仓库约定的互斥锁；CPU 搜索无需锁。

`avoidance_audit.json.accepted` 必须结合原任务、完整 RGB/视频、完整物理碰撞、自碰撞各独立报告通过。核心证据：旧真实 baseline 在同一增广几何下有安全阈值违规；增广后实际 q 全部安全；原任务最终成功；源布局与控制参数一致；初态/目标和增广支撑合法；自由段的轨迹变化与被阻断通道对应。路径差异仅作解释，不能单独决定通过。

## 本轮几何事实与边界

原台架碰撞顶实测约 0.7944 m，原花瓶仅 0.245 m 高。粗略照搬厨房台面上的花瓶没有影响约 1.15–1.45 m 高的开门接近轨迹。因此可用新增真实落地展示座，整座必须检查机器人、厨房、台架与门扫掠，不能悬空花瓶。

同一 seed 不保证增广前后源布局相同：关系求解器会把新障碍纳入优化，初次新增展示座时曾把机器人沿 x 移动约 0.403 m，该样本必须拒绝。`fixed_source_layout.json` 从原成功 episode 的实测 panda_link0 与原组合 USD 的局部矩阵准确还原原 asset pose；增广时仅锚定这个原求解结果，保留 On/NextTo，删除已消费的 RotateAroundSolution marker，不能手选新的源位置。启动时断言实际 base/yaw 与基线一致，审计再独立比较布局与初始 q。

若固定默认把手点被人体占据，可以让相同的障碍感知控制策略按顺序尝试原把手轴线上的合法高度，记录拒绝原因与最终选择；无障碍 baseline 也必须使用同一策略并实际采集。当前 `--adaptive-handle-height` 候选是原默认 +0.12 m、随后 +0.18 m/+0.22 m，各候选在任何动作前先由 cuRobo 检查。不能按 human 标签硬写不同点。当前样本若在短暂成功后轻微回弹到阈值下，入口通过有界真实握把闭环补拉恢复；原严格开放阈值保持，终态原谓词与补拉全程双指接触仍要通过。

原 MPFB 腕高约 1.085 m。仓库中 `posed.blend` 有原骨骼和蒙皮，可以在同源模型上摆臂；不得只移动人体碰撞代理而不改变视觉模型，或让脚漂浮。`repose_human_arm.py` 在 Blender 中做原骨架双骨 IK，原脚/root/其余非右臂骨骼保持；`wrap_reposed_human.py` 给全部6个实际导出网格赋碰撞。生成数据写本组 assets，原资产保持不变。姿态报告需保存源哈希、目标误差、肢段长度和未改骨骼误差；完成后检查实际导出人体图像。只有新的真实采集和反事实审计全部通过，才可计为人体避障成功。

正式实测表见下方；其余7个原门任务、随机化、11类覆盖及200条采集仍未验证。

## 本轮纠正后必查的实现细节

- 必要性必须由真实机器人碰撞 hull、实际各手指网格和原障碍几何建立。保守 arm/finger 胶囊或旋转后的 world-AABB 相交不能单独证明实际任务需要避障。本组 `physical_hazard_geometry.py` 单独重建真实 Cylinder/Cube/Capsule，最终审计同时保留代理和真实几何结果。候选0人体只让代理违规、真实间距27.76mm，因此已撤销；真实候选来自 `human_physical_candidates.json`。
- Arena 的关系求解器要求新增 Object 根 yaw 为90°整数倍。非整数旋转可烘焙进新增资产子 Xform，Object root yaw=0；几何世界位姿不改变。`bake_hazard_yaws.py` 实现此操作。搜索 primitive 时也须匹配 live constructor 的 world-AABB；不能 CPU 用旋转 OBB、live 却用大 AABB。
- cuRobo 膨胀球/box 的初态可能比 FCL 真网格更保守。`probe_planner_initial.py` 从完整 URDF 和原球配置重建初态；用它筛合法候选，而非降低原60/50mm财产阈值。原宽花瓶被拒绝后，改用已有窄颈玻璃瓶及真实展示座，保存来源和完整支座检查。
- `freeze_source_layout.py <baseline> --output <new-file>` 从原真实基线重建 source pose，拒绝覆盖；本轮重建与保存配置逐字段完全一致。
- 同源 MPFB 生成入口：`bash scripts/extensions/doors/build_reposed_human.sh /path/to/blender`。Blender 4.0.2 实测；源为 `assets/people/mpfb_corridor/posed.blend` 与 textures。生成 USDC/textures 留本地并由局部 `.gitignore` 排除，小型 wrapper、pose、artifact_manifest 可追溯。`audit_human_feet_blender.py` 独立导入生成 USD，比对源模型20cm以下2948个顶点，双向最大误差0；源脚/root未移动。人物全局新增摆放另记在variant配置中。
- 原三相机保持。人体近处背部遮挡 external_camera，但 external_camera_2 可辨手、机器人和把手，是本轮原速动作对照视角；不能把补渲染全景当真实轨迹采集。
- 正式验证顺序应为 `validate_trial.py` → 锁内 `audit_self_collision.py` → CPU `audit_avoidance.py --baseline <matching-baseline>`。审计对 baseline 与新样本的 result/validation_full/self_collision 各报告都要求通过；baseline必须显式记录controller_parameters，缺失时须补采，不能套用历史默认值。`baseline_counterfactual.passed=false` 表示旧路径在新增风险下失败，须同时 `violates_safety_margin=true`；整体 `accepted=true` 才是通过。审计拒绝覆盖已有报告，修订使用 `--report-name avoidance_audit_revisionNN.json` 并保留原报告。

## 本轮最终实测（3/3有效避障）

| 变体 / 正式episode | 帧 / 物理记录 | 旧真实几何最近距离 | 新真实几何最近距离 | 原执行阈值 |
|---|---:|---:|---:|---:|
| human / `human_attempt_02` | 328 / 2952 | 相交 | 42.864 mm | 20 mm |
| property / `property_attempt_04` | 327 / 2943 | 41.707 mm | 100.484 mm | 50 mm |
| robot / `robot_attempt_02` | 326 / 2934 | 1.206 mm | 42.555 mm | 10 mm |

三条的 `result.accepted`、`validation_full.accepted`、`self_collision_audit.passed` 及选定固定台架/避障审计全true；每条原3路与合成视频解码/对齐通过。三条的统一入口均为 `avoidance_audit.json` 和 `path_comparison.png`。robot补采同版本baseline后的正式修订是 `avoidance_audit_v2.json`；原canonical报告/图先改名保留为v1，再将已通过v2复制为canonical，详见 `evidence_revision.json`。robot目录的 `controller_equivalence.json` 保存控制参数与实现哈希一致性证据，所有旧数据保持不变。human/property 使用实际 `datasets/avoidance_trials/doors/base_adaptive_attempt_01`（330帧、2970物理检查）；robot 使用本轮固定目标 `datasets/avoidance_trials/doors/base_fixed_attempt_01`（330帧、2970物理检查）。该baseline与robot样本均不传 `--adaptive-handle-height`，controller_parameters及collect/environment/collision/config四个实现SHA256全部一致，无需推断历史默认参数。所有baseline各自成功、RGB/物理/self报告通过。

human/property 同一顺序候选策略因原+.12m点碰撞IK失败而自动选择+.18m；robot保持+.12m。自由段TCP最大变化依次124.76、101.15、122.09mm，必要性由真实几何旧路径违规决定，不由变化量决定。财产所有物理记录最大平移0.000862mm、旋转0.000350°、绝对竖直偏差0.000339°，远低于5mm/2°门槛；实测落座pose的旧路径最小41.707mm是最终证据，名义候选39.81mm不作为最终值。

首个物理成功但移动源机器人约0.403m的property02已拒绝；property03因cuRobo初态不可行拒绝；根yaw不符要求的初次human/robot初始化失败也保留。`avoidance_pre_self.json` 是self报告完成前的真实拒绝状态，正式证据为后来新写的 `avoidance_audit.json`，不覆盖历史。

最终机器汇总为 `reports/avoidance_trials/doors/summary_v2.json`。旧summary与robot v1保留为修订历史，交付时读取各entry的selected_avoidance_report。
