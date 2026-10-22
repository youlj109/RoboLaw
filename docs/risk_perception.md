> 当前 HS 已更新为 `arena-hs-dynamic-ssm-1`（适配器 `arena-risk-observer-14-dynamic-hs`）：真实 COM 运动在线保存，完整 Warp 表面距离、动态保护距离与余量离线计算。新规则与实验参数见 [hs_dynamic_distance.md](hs_dynamic_distance.md)；以下旧在线/FCL/固定距离内容仅作历史记录。

> 2026-10-22 当前默认：所有场景采用原始证据在线采集，网格距离、风险特征、HS/PT/RS、审计和 RGB/视频离线处理。人体场景仅为验证样例，场景与资产清单从各条数据读取。启动方式见 [offline_risk_workflow.md](offline_risk_workflow.md)。以下旧在线流程和 FCL 说明保留为历史复现记录。

# huirongzeng：完整风险感知与复现

本分支发布 5090 当前的风险采集与评价代码，覆盖原有 10 个非人体场景和 1 个默认人体场景。采集在 shadow 模式下记录安全特征、HS/PT/RS 等级和证据；评价器不获得动作修改权限。当前适配器版本是 `arena-risk-observer-12-ir-disabled`，PT profile 为 `pt-hs-inspired-v4`，聚合策略为 `arena-known-results-1`。

“完整”指采样、规则、记录、报告、导出代码及其依赖均在发布范围内。材料损伤、权威机器人故障状态和视觉感知真值仍可能缺测；输出等级不代表这些信息已经采集完整。

## 实际使用的特征与规则

| 类别 | 当前输入及来源 | 主要规则 |
| --- | --- | --- |
| HS 人体安全 | 运动部件、末端、目标物到人体的 FCL 表面距离；运动接近速度；PhysX 人体接触及力/冲量；人体运行时清单 | `HS-DIST-001` 距离与保护边界；`HS-COLL-001` 人体接触；历史 TTC 项保留为审计 |
| PT 财产安全 | 逐资产机器人/搬运物 FCL 距离；逐刚体对接触净冲量与力；实际抓持、支撑、速度和下降证据；损伤与支撑裕量的缺测状态 | `PT-DIST-001`、`PT-FORCE-002`、`PT-IMP-002`、`PT-DROP-001`；legacy/audit-only 项不参与已知结果判级 |
| RS 机器人安全 | 实测关节位置及限位裕量、力矩与运行时上限比值；独立 FCL 自碰撞；命令异常、无效关节遥测和故障后运动证据 | `RS-TORQ-001`、`RS-JOINT-001`、`RS-SELF-001`、`RS-FAULT-001` |
| IR 感知/动作安全 | 危险动作生成/发送、跟踪丢失、遮挡、位姿误差等声明字段；相机来源与动作提交审计独立记录 | `IR-CMD-001`、`IR-LOSS-001`、`IR-OCC-001`；当前暂不启用，不执行这三项规则，不参与总体等级或完整性统计 |

规则及所有数值参数以 `configs/safety/profile.robot.pick_place.v4.json` 为准；Arena 对象映射、距离来源、证据协议和任务豁免以 `configs/safety_observer_arena.json` 为准。运行时展开后的逐资产规则写入每条数据的 `safety_observer_binding.json/resolved_profile`，源码指纹也会保存。

### FCL 距离的口径

最终风险距离来自 authored 碰撞几何的三角面 FCL 查询，支持完整仿射缩放。AABB 仅作候选筛选和最小值剪枝，不作为最终评价距离，也不在 FCL 缺测时回退填值。Cube 使用表面三角形；Sphere/Capsule 使用记录了分辨率的三角化表面。结果针对这些表面，不能等同于 PhysX 实际 cooked 碰撞体或实体材料的精确表面。接触力与冲量仍来自 PhysX 实测回调，几何相交不直接伪造接触力。

### 当前任务关系

- HS 的最终距离等级使用运动部件、末端和目标物；固定 `panda_link0` 支座距离豁免。人体鞋子与固定支座的指定相交对也按配置排除；原始证据继续保留在机器可读记录中。
- PT 枚举环境中启用碰撞体的厨房资产组，包括台面、橱柜、冰箱等；目标芥末瓶的机器人距离按当前任务要求排除。
- 碗是任务放置目标，机器人到碗、搬运物到碗的 PT 评价对均排除。
- 台面支撑距离的排除要求有真实接触及覆盖、法向、穿透和低速证据；阶段名称本身不足以获得排除。对应力与冲量仍按配置评价。
- RS 排除机器人固定装配、相邻关节装配及配置中明确列出的结构对，保留其来源和覆盖信息。

视频按当前简化样式展示三路画面、HS/PT/RS 三类等级、有效指标和触发依据，不显示已豁免关系的提示小字。

### 等级与覆盖分开

一个类别的等级取可用有效分项中的最高等级；没有有效分项时为未知。缺测和 audit-only 项不参与等级聚合，`data_complete`、`missing`、`known_rule_count`、`excluded_rule_count` 独立记录。看到 L0 只能说明参与判级的已知分项均为 L0；损伤、完整故障状态或视觉真值可能仍缺失。无人场景只有在运行时闭世界清单确认 `verified_absent` 后，将 HS 标记为不适用。

## 安装与预检

先按根 README 选择官方或魔搭来源，恢复场景资产及三套源码，再安装容器环境：

```bash
git clone --branch huirongzeng https://github.com/youlj109/RoboLaw.git
cd RoboLaw
export ROBOLAW_DOWNLOAD_SOURCE=modelscope  # 也可选择 official
python3 scripts/restore_modelscope_assets.py
python3 scripts/restore_downloaded_assets.py --source "$ROBOLAW_DOWNLOAD_SOURCE"
python3 scripts/fetch_source_dependencies.py --source "$ROBOLAW_DOWNLOAD_SOURCE"
export ARENA_CONTAINER=robolaw-risk
bash scripts/setup_arena.sh --accept-eula
bash scripts/arena_python.sh scripts/check_risk_perception.py
```

`requirements-collection.txt` 包含 `python-fcl==0.7.0.11` 和 `PyYAML==6.0.3`。SafeBench 参考评价器随代码放在 `third_party/safebench_reference/reference_code/evaluator`；普通依赖恢复脚本不会覆盖这个目录。三套上游源码与场景资产继续使用根 README 的官方/魔搭双来源。

`check_risk_perception.py` 不启动仿真，检查当前 profile、依赖、解析立方体的 0.25 m FCL 间距，以及已知结果判级和覆盖分离。它不替代完整采集验证。

## 采集一条默认人体数据

先只查看命令：

```bash
bash scripts/arena_python.sh scripts/collect_safety_benchmark.py \
  --ids human_default_mpfb --num-envs 1 \
  --episodes-per-id 1 --max-attempts 1 --seed 42 \
  --output datasets/risk/human_run01 --run-id human_run01 --dry-run
```

去掉 `--dry-run` 后运行。请使用新的 output/run-id。该入口会启用风险观察器、保存逐物理步状态，并通过已有的离线渲染流水线生成原三路 RGB。默认人体沿用既有的抓取倾角、偏置、lower-first 放置等参数。采集可能因任务、几何、存储或验证失败而保留失败 attempt；请查看本次输出的报告。

全部 11 个场景：

```bash
bash scripts/arena_python.sh scripts/collect_safety_benchmark.py \
  --num-envs 1 --episodes-per-id 1 --max-attempts 3 \
  --output datasets/risk/all11_run01 --run-id all11_run01
```

也可在 `--ids` 后列出多个已发布场景。入口只选择这 11 项；源码中保留的一些历史人体扩增工具还引用其他人体变体，它们不在本次 11 场景资产包范围内。

## 数据、报告和时间对齐

输出位于 `<output>/human_default/human_default_mpfb/attempt_*` 或 `<output>/nonhuman10/<场景ID>/attempt_*`。每个真实执行的 attempt 保存原轨迹和独立安全流：

| 文件 | 用途 |
| --- | --- |
| `trajectory.h5` / `render_state_metadata.json` | 动作、实测状态及离线重建状态 |
| `safety_observer_binding.json` | 身份、实际物理频率、资产及规则绑定、源码/配置指纹 |
| `safety_physics.jsonl` | 逐物理步状态、几何、接触及覆盖 |
| `safety_decisions.jsonl` | 动作前输入、规则结果、等级及缺测项 |
| `safety_commands.jsonl` / `observer_action_audit.json` | 实际命令提交与无介入审计 |
| `safety_risk_summary.json` / `result_axes.json` | 风险汇总及任务、数据、安全、障碍约束四轴 |
| `safety_observer_validation.json` | 安全数据完整性检查 |
| `rendered/images.h5` / `rendered/render_report.json` | 三路 RGB 及原始状态索引 |
| `rendered/safety_camera_provenance.jsonl` | 相机来源、时间及冻结身份 |

物理采样 120 Hz，风险评价 30 Hz，动作/RGB 约 15 Hz，具体数值以绑定文件为准。每条风险使用已完成物理状态；视频按 RGB 来源模拟时间选择当时已经生成的最新风险决策，不把后续信息倒填到早期画面。

独立检查与报告：

```bash
bash scripts/arena_python.sh scripts/report_safety_observer.py \
  datasets/risk/human_run01/human_default/human_default_mpfb/attempt_0000_seed_<实际种子> \
  --render-output datasets/risk/human_run01/human_default/human_default_mpfb/attempt_0000_seed_<实际种子>/rendered
```

上面 `<实际种子>` 是占位符，请替换为实际 attempt 目录名，不要原样执行。LeRobot v3 风险标签导出入口为 `scripts/export_safety_lerobot_v3.py`，使用独立的 `requirements-lerobot-export.txt` 环境；它不应覆盖 Arena 的 CUDA/Torch 环境。

## 导出带等级和依据的视频

视频工具读取已有的风险记录，不重新评价轨迹。需 PyAV 和中文字体：

```bash
docker exec -w /workspace "$ARENA_CONTAINER" uv pip install \
  --python /workspace/.venv-arena/bin/python --no-deps \
  -r requirements-risk-review.txt
# Linux 可安装 fonts-noto-cjk，或将已有字体路径传给 --font
```

选定实际 attempt 后：

```bash
bash scripts/arena_python.sh scripts/export_risk_review_video.py \
  --episode datasets/risk/human_run01/human_default/human_default_mpfb/attempt_0000_seed_<实际种子> \
  --output reports/risk_review/human_run01
```

输出 `risk_review.mp4`、逐帧引用 `.frames.jsonl`、抽查图片和 `manifest.json`。output 必须为新目录。`--font /path/to/font.ttc` 可指定字体，`--max-frames 3` 可只导出前三帧检查环境；短预览会在 manifest 中标记为不完整轨迹。

## 发布验证与已知限制（2026-10-21）

- CPU 预检通过：配置及依赖可加载，FCL 解析间距和当前判级策略通过。
- 原 SafeBench 参考评价器 18 项离线回归测试通过。
- 11 场景发布清单包含 10 个非人体 ID 和 `human_default_mpfb`，默认人体 catalog 已补入发布包。
- 默认人体及全部 11 场景采集命令完成 dry-run；本次发布没有重新启动仿真采集。
- 视频可移植导出检查基于已有成功 FCL 人体轨迹，前三帧编码后解码验证；完整历史示例为 779 帧、51.93 秒。
- 原 `tests/test_safety*.py` 是更早的适配器测试快照，其绑定夹具缺少当前必填的 `rs_self_geometry` / PT v4 字段；本次运行 16 项出现夹具错误，不能声称该旧测试集通过。本次保留它们用于后续迁移，当前发布预检入口见上方。
- 本次没有完成全新机器安装及 11 场景完整仿真复现，也没有补造损伤、权威故障或视觉感知证据。

原部署阶段的旧投影测距、旧缺测聚合说明保留在 `docs/pt_v4_arena.md`、`docs/safety_observer.md`，文件开头标明历史范围。当前行为以本说明、配置和源码为准。

## IR 暂停启用（2026-10-21）

默认配置 `disabled_categories: ["IR"]` 会传入运行时 profile。IR 结果为 `status=disabled`，`level=null`、`data_complete=null`，不生成 IR 规则结果，也不计入缺测或风险汇总。保留原始四槽位顺序 HS/PT/RS/IR 以兼容数据结构；新增 `risk.enabled` 掩码，禁用 IR 对应 0，等级仍为 -1、valid 为 0。读取数据时先检查 enabled，避免把禁用当成缺测。

新视频导出默认只显示 HS/PT/RS。历史记录保持原样；旧 profile 没有禁用项时仍按当时四类规则回放。再次启用需将配置改为 `disabled_categories: []`，并相应调整视频显示类别。

SafeBench 候选评价器的 `iso_ssm_candidate.py` 有一处 RoboLaw 适配：增加显式 IR 开关并从汇总中排除禁用类别。原 profile 的默认行为保持兼容，文件保留原 CRLF 格式。
