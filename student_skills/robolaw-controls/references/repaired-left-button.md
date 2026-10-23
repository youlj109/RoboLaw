# 左取消按钮：2026-10-22 修复后的入口

本轮只验证本组首任务 `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian`，人体／花瓶／横杆各一条真实采集，以及每类三个规划 seed 的障碍开关配对。用户的一条要求覆盖 workflow 默认三条实采要求；不代表右按钮、温度旋钮或批量成功率。历史 `human_attempt02`／`property_attempt01`／`robot_attempt02` 保持撤回。

## 当前配置与物理约束

使用 `configs/extensions/controls/repaired_left_button.json`。`environment.py` 默认通过本组 `toaster_physics.py` 局部覆盖两被动旋钮：约 0.2–0.34 mm 的 COM 偏心投影至原轴，新增有效角阻尼 0.05 N·m·s/rad、刚度零。原几何、0–265°关节范围、质量、按钮行程／谓词不变；不是逐步写关节角或锁关节。

默认修复有 30 秒中性静置和外力矩可动性实测：中性两旋钮漂移 <1e-16°，施加 0.005 N·m 真实力矩 2 秒后转动约 11.45°。外力矩仅是机构诊断，不能作为机器人任务成功。USD angular drive 使用 degree，作者化阻尼是有效 rad 阻尼乘 π/180；读取运行时参数确认单位转换。源／配方／覆盖层哈希通过 `toaster_physics_provenance` 写入每条 result。

新增人体位于机器人右侧，使用本组 `assets/extensions/controls/human_reach_elbow_up/`，根节点、脚部和骨长保持源模型；`artifact_manifest.json` 绑定 person.usdc 和纹理，不能只哈希 collidable.usda 外壳。花瓶在原台面前缘，横杆在原台面锚固。机器人、原按钮、相机均未移动。

接近策略为原 TCP 目标的固定冗余 IK 分支 + cuRobo joint-goal。同一分支种子用于无障碍和三种新增实体，并跨规划 seed 保持；没有插入手写绕行路点。候选与失败保留，不能为强调绕行重新选择异常长基线。

## 当前实测证据

最终基线：`datasets/avoidance_trials/controls/base_repaired02`，289 帧。

| 变体 | 目录尾部 | 帧数 | 旧实际短路线的源形状距离 | 新实际严格模型距离 | 原执行距离 |
| --- | --- | ---: | ---: | ---: | ---: |
| 人体 | `human_repaired01` | 279 | 相交 | 32.15 mm | 20 mm |
| 花瓶 | `property_repaired01` | 361 | 40.52 mm | 101.05 mm | 50 mm |
| 横杆 | `robot_repaired01` | 319 | 相交 | 52.72 mm | 10 mm |

三条原按钮最大位移约 2.03 mm，保留原严格成功阈值 1.508303 mm。负载接触在成功前建立，120 Hz 同步接触／q/dq证据和所有非目标关节静置通过。实际外部安全逐物理更新检查；独立限位／自碰撞仍为动作率，不能称为更密采样。新真实接近路径长约 0.936／1.338／0.637 m，原短基线为 0.278 m。

`reports/repair_20261022/controls/paired_final_02` 中 seed 42、43、44 各包含共享的无障碍路径及三个有障规划。每一配对起始 q、关节终点、原 TCP 目标和控制参数相同。全部 12 个规划通过原密集检查；三个 seed 的旧路径均违反对应源形状安全约束。有障路径是可行规划结果，未证明全局最短，重复路径也不能视为独立随机样本。配对采用名义出生位姿；花瓶落稳约 1 mm，真实执行审计另使用实测位姿。

花瓶只是保护距离不足，不是碰撞或损伤证明。物理静置的源花瓶最低 `Wall00_*` 环本来与底座同为 z=0；约 40 纳米的数值重叠仅对该已核实底环／原台面允许 1 微米容差。其他侧壁、其他环境接触和机器人 50 mm 距离不变；原拒绝报告保留为 `avoidance_audit_rejected_bottom_ring.json`。此特例不能推广到其他资产。

## 可复制步骤

先确认本组脚本、资产、正确容器挂载和 GPU 锁均存在。在本机重新生成 `reports/skill_trials/controls/inspection/stage.usda`，不要假设其他学生的 ignored 数据已随 Git 提供。所有输出换新目录；下列是本组首任务入口，不能套用到右按钮／旋钮。

```bash
bash scripts/extensions/controls/run.sh probe_passive_knobs.py --out reports/controls/neutral_new --device cpu
bash scripts/extensions/controls/run.sh collect_trial.py --config configs/extensions/controls/repaired_left_button.json --variant base --output datasets/controls/base_new --seed 42 --max-steps 650 --device cpu
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/controls/base_new
bash scripts/extensions/controls/run.sh audit_self_collision.py datasets/controls/base_new
bash scripts/arena_python.sh scripts/extensions/controls/review_contact_causality.py datasets/controls/base_new

bash scripts/extensions/controls/run.sh collect_trial.py --config configs/extensions/controls/repaired_left_button.json --variant human --output datasets/controls/human_new --seed 42 --max-steps 650 --device cpu
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/controls/human_new
bash scripts/arena_python.sh scripts/extensions/controls/review_contact_causality.py datasets/controls/human_new
bash scripts/arena_python.sh scripts/extensions/controls/audit_fixed_base.py datasets/controls/human_new
bash scripts/arena_python.sh scripts/extensions/controls/audit_augmentation.py datasets/controls/human_new --baseline datasets/controls/base_new/records.json --report-name strict_clearance_audit.json
bash scripts/extensions/controls/run.sh audit_self_collision.py datasets/controls/human_new
bash scripts/arena_python.sh scripts/extensions/controls/audit_avoidance.py datasets/controls/human_new --baseline datasets/controls/base_new
bash scripts/arena_python.sh scripts/extensions/controls/render_avoidance_comparison.py datasets/controls/human_new --camera external_camera

bash scripts/extensions/controls/run.sh paired_planning.py --config configs/extensions/controls/repaired_left_button.json --baseline datasets/controls/base_new --stage reports/skill_trials/controls/inspection/stage.usda --output reports/controls/paired_new --seeds 42 43 44 --variants base human property robot
```

同样采集并审计 property／robot，两者有自己的新目录。三个配对 seed 是规划检查，不自动计为三条物理采集。`--device cpu` 仍使用 GPU 进行 cuRobo 和 RTX 相机；`run.sh` 已持共享锁，不要外套同一 flock。

读取每份报告实际 gate，新增配置／资产／原任务后重新验收。数据集里的 `config.json` 是读取时冻结副本；源资产、机构配方、状态、视频及审计哈希必须对应同一回合。真实危险关障路线只做离线检查。源相机第二路在人类手臂附近被遮挡时仍完整保留，展示对照可选原第一路，但不移动相机或隐藏人体。

仓库内详细结果在 `docs/extensions/controls/REPAIRED_LEFT_BUTTON.md` 和 `repaired_left_button_results.json`。未通过项、未测项和旧撤回单列，不能依据本轮三条使旧回合恢复合格。
