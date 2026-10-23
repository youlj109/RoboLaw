# 按钮与旋钮的避障增广

**修复进展：** 新的左按钮 `human_repaired01`、`property_repaired01`、`robot_repaired01` 与 `base_repaired02` 已另行验证；按 [修复后的实测与复现](repaired-left-button.md) 使用新配置。下述历史撤回及严格验收要求保留，不能据新样本复活旧回合。

**2026-10-22 复查：先读 [当前交付验收](review-gates.md)。下面保存旧单基线试验的参数和报告，历史 accepted 不等于当前交付通过。**

本组视频操作的是烤面包机左取消按钮，不是微波炉。human_attempt02/property_attempt01/robot_attempt02 虽有原按钮按压成功记录，场景却存在未操作的 sideL_knob_doneness_joint 自转；机器人中性保持的独立对照也复现自转。因此三条场景物理质量不合格，不能作为正式交付。它们只排除一条曲折旧路线，短路径及跨障碍检查显示避障设计偏弱，须修复机构问题并重新设计/验证，不能把旧 accepted=true 当作新验收。

用户指出首轮试用只在场景中摆放实体，缺少避障设计。保留原数据用于物理与采集流程测试，但不能继续将其作为有效避障样本。

## 设计顺序

1. 采集无新增障碍的真实成功轨迹，保存原环境、机器人、家电初始状态、任务 YAML 哈希、seed、控制参数及视频。目标是可复现的对照；不能使用源布局不同、任务未成功的轨迹。
2. 对接近按钮/旋钮的自由运动段，读取实际机械臂、夹爪和相关携物几何的扫过范围。在此范围放置新增实体。只改增广物体，保留源任务对象、目标和机器人初态。
3. 在 CPU 中把这条旧轨迹放入新障碍世界做几何反事实检查，不能将危险旧轨迹真实回放碰撞。初态和操作目标应保留安全间隙，风险出现在可调整的中间段；按压轴、旋钮轴和必须接触的目标不被任意移动。
4. 检查新增实体自身有真实支撑、不穿厨房、固定台架、家电或其他实体。若使用支架或展示座，支撑部件也要参与所有碰撞检查。人体在地面合法站立，不能把脚移入台架以换取手臂挡路。
5. 使用相同控制参数重新规划并真实执行，完整保留各类别原间隙及接触许可。必须验证实际状态、原任务成功、接触证据、数据和视频。不能为跑通而降低距离、禁用碰撞或移动原目标。

## 验收证据

每条有效增广保存 `avoidance_audit.json`：

- 基线目录与轨迹哈希、原任务哈希、初态对齐误差、seed/控制参数。
- 旧轨迹在新障碍下的违规动作、阶段、机器人部位、障碍部位、实际表面距离及要求距离。需有明确裕量，不用浮点噪声制造违规。
- 新轨迹的实际最小距离，逐物理更新外部安全、独立自碰撞、真实任务成功和视频对齐。
- 初态/目标可行、障碍与环境/固定台架合法。
- 自由段路径长度、轨迹对照或可视化。路径差异用于解释绕行或姿态调整，单独不同不能作为有效性的证明。

明确分别报告物理采集通过和避障设计通过；缺少对照标为“避障未验证”。单条固定轨迹的反事实只证明这条旧路径需要调整，不证明所有无障碍可能路径必然碰撞，也不代表随机化或批量成功率。

当前本组搜索入口 `scripts/extensions/controls/search_avoidance.py` 只生成稀疏采样候选，不能当成正式验收。采集支持 `collect_trial.py --config PATH`，把选中布局存入独立配置，禁止覆盖历史回合配置。最终 `audit_avoidance.py EPISODE --baseline BASELINE` 读取全部动作前后真实状态，并要求物理、数据、固定台架、自碰撞报告。CPU 几何审计仍不等于逐物理步审计。

## 历史单基线复现命令（仅作溯源）

以下使用历史 `configs/extensions/controls/avoidance_left_button.json`，用于解释旧记录；**当前启动请使用 [repaired-left-button.md](repaired-left-button.md) 的配置、资产恢复和完整验收步骤，不执行本段作为新交付。** `left_button_trial.json` 同样仅保留作首轮物理流程记录。历史命令如下，旧数据不覆盖：

```bash
bash scripts/extensions/controls/run.sh collect_trial.py --config configs/extensions/controls/avoidance_left_button.json --variant base --output datasets/controls/avoidance_base_01 --max-steps 550
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/controls/avoidance_base_01
bash scripts/extensions/controls/run.sh audit_self_collision.py datasets/controls/avoidance_base_01
bash scripts/extensions/controls/run.sh collect_trial.py --config configs/extensions/controls/avoidance_left_button.json --variant human --output datasets/controls/avoidance_human_01 --max-steps 550
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/controls/avoidance_human_01
bash scripts/arena_python.sh scripts/extensions/controls/audit_fixed_base.py datasets/controls/avoidance_human_01
bash scripts/arena_python.sh scripts/extensions/controls/audit_augmentation.py datasets/controls/avoidance_human_01 --baseline datasets/controls/avoidance_base_01/records.json --report-name strict_clearance_audit.json
bash scripts/extensions/controls/run.sh audit_self_collision.py datasets/controls/avoidance_human_01
bash scripts/arena_python.sh scripts/extensions/controls/audit_avoidance.py datasets/controls/avoidance_human_01 --baseline datasets/controls/avoidance_base_01
bash scripts/arena_python.sh scripts/extensions/controls/render_avoidance_comparison.py datasets/controls/avoidance_human_01
```

`property`、`robot`分别执行相同步骤。先按旧试用文档生成本机的 inspection stage；固定台架审计使用本机原组合 USD。逐项读取报告的布尔门槛，不仅检查退出码。数据进入 `datasets/avoidance_trials/controls` 时可由统一 `summarize_trials.py --trial-root avoidance_trials` 统计。

几何结果是当前安全模型中的距离：运动臂包含碰撞网格包络与保守胶囊，USD primitive 在本组规划/执行模型中使用保守世界 AABB。不要把包络相交说成实际表面必然碰撞。候选旋转后必须重新计算与 live 相同的 world AABB，否则离线初态可行结论可能错误。完整的实际逐物理步检查仍为最终硬门槛。

目标姿态检查按物理单位分别计算位置误差和旋转角，沿用原 cuRobo 的 2 mm / 0.02 rad 到达容差；不能把混合了米和无量纲元素的矩阵差值当位姿阈值。该检查不改变原按钮成功谓词，也不改变任何安全间隙。

独立反事实优先核查真实碰撞形状。横杆使用共享 `kitchen_safety.geometry_audit` 的真实 USD Cylinder/Cube/mesh，排除纯世界 AABB 角落误报；机器人使用碰撞 hull 与逐关节手指 hull，补检时移除冗余胶囊。此补检只加强“原路径确实需要避让”的证据，不能削弱原规划/执行的保守安全模型。

花瓶反事实使用实测静置位姿，而非名义出生坐标。落在原台面上的花瓶底面接触单列，仅允许确定的 Base/原 countertop 支撑对及最多0.5mm垂直数值容差；侧壁、厨房其他部位、台架和机器人间距没有此例外。保持全部实际姿态、稳定性和原任务门槛。

## 2026-10-22 历史实测（场景异常，撤回交付验收）

三条均与 `datasets/avoidance_trials/controls/base_attempt02` 配对。原机器人、烤面包机初态完全相同，原按压目标与控制参数相同；按压成功仍使用原严格行程阈值1.508303mm。以下旧路径只做CPU反事实，没有执行危险碰撞。

| 变体及正式目录 | 帧数 | 原路径最小距离 | 新路径最小距离 | 原执行安全距离 |
| --- | ---: | ---: | ---: | ---: |
| `human_attempt02` | 283 | 0（碰撞hull相交） | 55.940mm | 20mm |
| `property_attempt01` | 268 | 15.741mm | 109.242mm | 50mm |
| `robot_attempt02` | 283 | 0（真实Cylinder相交） | 90.978mm | 10mm |

三条历史记录都有 `avoidance_audit.accepted=true`，但当前复查的场景物理质量拒绝，独立原任务/数据验证、固定台架、原严格距离及自碰撞报告通过，实际按钮位移分别为2.031、2.039、2.034mm。实际外部安全由原逐物理更新检查提供；上表和反事实几何采用全部动作前后状态，不能称作更密的测量。花瓶全部15Hz动作前后实测位姿保持稳定竖直；逐物理记录提供外部安全检查，未记录相同密度的花瓶位姿，不能声称120Hz稳定性。

每条 `comparison.mp4` 使用原camera2的两条真实录像，原速并排；较短视频仅在结尾保持末帧。下方红线是原轨迹在新增风险下的CPU反事实，绿线是真实新轨迹。完整解码证据保存在 `comparison_validation.json`。比较曲线不是真实碰撞录像。

修订证据保留：原横杆45°候选因离线旋转AABB与live模型不一致拒绝；已修复候选模型，并使用yaw0°重新完整采集。花瓶报告修订保留 `avoidance_audit_v*.json` 与 `audit_revision*.json`，分别说明位姿单位、实测静置位姿和唯一台面支撑数值接触；未改变任何机器人安全距离。三个固定样本不代表右按钮、旋钮、随机化或200条批量成功率。

最终控制参数采用原始记录显式比较，不能由默认值推断。早期base01/human01缺该字段，因此新采base02/human02；两条新实测轨迹与各自早期轨迹的全部动作前后q完全相同。property01/robot02的数据不变，已与新base02重新全量审计并重生成对照视频，旧审计/视频保留。汇总器强制显式控制参数、全部因果gate、双方记录哈希及对照视频基线引用一致，拒绝沿用缺字段旧报告。
