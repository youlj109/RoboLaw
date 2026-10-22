2026-10-22 复审更正：本页保留历史过程，不能以其中 passed/accepted 或“通过”代替当前交付状态。A人体真实几何必要性未证实，D三条机构自转及避障设计待修；其余8条仅保留单轨迹证据。当前结论见 docs/extensions/AVOIDANCE_REVIEW_20261022.md 与 avoidance_quality_review.json。

# One-wall 原冰箱任务的有效避障修正

旧 `datasets/skill_trials/doors` 三类样本只作真实开门/相机/数采 smoke test，不能算有效避障。新输出独立在 `datasets/avoidance_trials/doors`，每条须通过 `avoidance_audit.json.accepted`，且原任务/native最终角度/双指真实接触/原三路RGB与视频/120Hz完整碰撞/自碰撞均独立通过。

源任务、冰箱轴、严格开放阈值18°、原机器人初态和厨房布局未改。源 pose 由 `freeze_source_layout.py` 从原无障碍成功 episode 和原组合 USD 精确重建；同 seed 下原关系求解器会避开新物体而移动机器人，因此本轮保留原 On/NextTo 并锚定已求解的原 pose，运行时和独立审计均复查。

必要性使用真实碰撞 mesh/hull 与 USD Cylinder/Cube/Capsule。规划/执行仍保留原更保守的胶囊和AABB模型；仅这些代理违规不足以通过必要性。实际机器人运动、门运动、动态玻璃瓶姿态每个120Hz物理步检查。危险旧路径只CPU反事实评估，绝不在新增障碍下执行。

## 选定布局和设计理由

- 人体：原 MPFB 模型通过真实骨架/蒙皮举肘，源 `posed.blend` 和双骨长度不改。脚部2948个源世界网格顶点与导出USDC独立双向比较误差0；整个人物新增摆放 `[3.7,-1.60,0]`、yaw220°，其旋转烘焙进新增子Xform供Arena根90°限制使用。旧机器人真实碰撞hull会与手臂相交。新增人体不能碰原厨房、台架或门扫掠。原camera2能看清手、机器人和把手；camera1被背部遮挡是实际遮挡，未改原外参。
- 财产：使用已有 `property/glass_bottle.usda`，原尺寸/质量/碰撞保持；新增真实碰撞落地展示座，瓶初始 `[4.34,-1.17,.88]`，落座稳定后按真实pose复核。旧轨迹真实间距约39.8mm，低于原50mm财产安全距离。原宽花瓶候选因为cuRobo保守初态模型拒绝而撤销，不降低间隙。
- 实体障碍：已有 `robot/projecting_crossbar.usda`，新增落地固定支座，位置 `[4.88,-.99,.86]`、yaw270°。旧轨迹对真实圆柱最小约1.21mm，低于10mm阈值。整根支座包括在机器人、厨房、台架和门扫掠检查内。

## 控制策略与对照

财产/人体使用同一 `--adaptive-handle-height` 策略：从原把手同轴高度偏移+.12、+.18、+.22m依序尝试，任何动作前由cuRobo检查；无障碍 baseline 也实际运行同策略，默认选+.12m。选择由障碍可行性决定，没有按类别写不同目标。每次requested handle/pregrasp pose、失败原因和选点保存到metadata。真实双指接触、源把手范围和最终原谓词继续独立检查。

人体本轮实际选+.18m，原默认点被cuRobo IK_FAIL拒绝。若短暂开放后回弹，控制器仅通过原真实接触补拉，原严格开放阈值不改。横杆沿用默认固定目标策略，与本轮相同入口、相同显式控制参数的330帧固定目标baseline对比。

## 验证入口

```bash
HEADLESS=1 # 是环境变量，CLI不支持 --headless
# 所有Kit/cuRobo运行还要使用仓库共享gpu.lock；详见学生skill。
bash scripts/arena_python.sh scripts/extensions/doors/validate_trial.py EPISODE
# cuRobo自碰撞需要GPU互斥锁
bash scripts/arena_python.sh scripts/extensions/doors/audit_self_collision.py EPISODE
bash scripts/arena_python.sh scripts/extensions/doors/audit_avoidance.py EPISODE --baseline BASELINE
```

审计要求baseline和actual各自result/validation_full/self_collision都通过，严格比较源布局和控制参数；财产还要求所有物理状态平移≤5mm、转动≤2°、绝对竖直偏转≤2°。每个新增实体单独确认floor或其展示座支撑。`baseline_counterfactual.passed=false`表示旧路径在新增风险下失败，只有整个`accepted=true`才通过。已有审计、图片拒绝覆盖；修订用不同`--report-name`并保留历史。

最终 `path_comparison.png` 画真实TCP顶视/侧视和真实几何间距，明确旧15Hz动作前后、新120Hz物理+postcheck。视频保持原速、原相机；控制/规划/物理重构图不冒充实际采集视频。正式结果表如下；其余7个原门任务仍未验证。

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
