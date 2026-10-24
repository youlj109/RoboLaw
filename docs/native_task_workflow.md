# 原生多任务采集与离线评估

适用于任务契约所声明的 pick-and-place、门、按钮、旋钮。默认 120Hz 物理状态、30Hz 动作前观测/动作/离线标签。每条归档只包含一个环境的一次运行。IR 关闭。

可先用 [统一任务/变体准备入口](native_task_variants.md) 生成当前任务契约和障碍绑定；其文件摘要在采集加载时再次核对。

## 输入与职责

- `configs/risk_task_suite/<task>.json`：任务角色、原生 Arena 规格与冻结规则身份。
- `--asset-cache`：数据盘缓存。官方资产使用 `official/<域名>/<原 URL 路径>`；Lightwheel 使用 `floorplan/`、`object/`。
- `--policy-factory kitchen_safety.<module>:<factory>`：显式任务控制器。factory 接收 `env, contract, bindings, config`，返回实现 `act(state, last_result)` 的对象。每次返回 `action, phase`，可带 `operation_id, evidence`；返回 `None` 结束。控制器不得改变已冻结场景拓扑、规则或资产角色。
- `--policy-config`：控制器配置 JSON。已有 `task_action_policy:create` 只回放显式命令表，要求 task hash、asset roots、30Hz 一致。它不生成成功策略，也不把演示状态当实测状态。
- `task_cartesian_policy:create` 提供按本次实测位姿计算目标的关键帧开发控制器，说明及验收边界见 [native_task_controllers.md](native_task_controllers.md)。它不包含路径避障，不能直接作为正式批量策略。
- `--obstacle-binding`：本任务实际障碍 USD、静态标志、类别及放置依据；可选择明确世界位姿或下述本任务原生关系放置。不得将上一场景世界坐标直接套用新场景；即使厨房相同，原生放置器也可能重新确定机器人位置。

控制器执行所需状态可在线处理。风险距离、风险特征、HS/PT/RS、完整审计、RGB 和视频不在采集进程内运行。视觉策略必须另外保存真实在线视觉输入，不能用离线画面替代。

官方资产缓存按原 URI 验证完整依赖闭包及每个文件的大小/hash；复制了根 USD 或下载清单并不意味着全部资源到达。`scripts/fetch_task_suite_assets.py --cache ...` 可恢复缓存并生成依赖边；目录标识 `assetInfo.identifier` 与真正的几何/材质依赖区分。资源缺失时先恢复，不在采集时静默打开残缺场景。

## 分阶段命令

在容器 `/workspace` 中执行，下面用 `TASK`、`RUN`、`CACHE`、`POLICY` 表示本次明确选择的路径/模块。

```bash
python scripts/risk_capture_workflow.py capture --output "$RUN" --task-contract "$TASK" --asset-cache "$CACHE" --policy-factory "$POLICY" --policy-config /absolute/policy.json
python scripts/risk_capture_workflow.py audit --output "$RUN"
python scripts/risk_capture_workflow.py evaluate --output "$RUN"
python scripts/risk_capture_workflow.py render --output "$RUN"
python scripts/risk_capture_workflow.py video --output "$RUN"
```

上述阶段名是命令行位置参数；以 `--help` 为准。宿主机通过 `ARENA_CONTAINER=arena-kitchen-collect bash scripts/arena_python.sh` 调用相同脚本。每个输出目录须为新目录；错误记录保留，不覆盖历史结果。

诊断加载/保持动作使用 `scripts/inspect_risk_task.py --capture-hold-actions 3`，与正式采集目录、统计分开。它只能验证接口，不能验证任务成功。

## 原始数据格式

`capture/native_000/raw_episode/`（诊断为 `raw_hold/`）：

| 文件 | 内容 |
| --- | --- |
| `task_contract.json`、`relation_scope.json`、`rs_pair_scope.json` | 本条任务、精确资产/表面关系、独立机器人自碰撞范围 |
| `mesh_inventory.json`、`meshes/*.npz` | 碰撞三角形、实际刚体/关节图、几何哈希 |
| `telemetry_binding.json`、`raw_capture_schema.json` | 实际 PhysX views、关节名称、时钟与单位 |
| `raw_evidence.jsonl` | 有序 `initial_state`、`observation`、`physics`、`fault_event`，v2 增加末尾 `terminal_observation` |
| `variation_snapshot.json`、`capture_context.json` | 原生采样器实际抽样值、环境/放置种子和资产来源；仅 seed 不代表保存了随机结果 |
| `frozen_rules.json`、`frozen_implementation.tgz` | 参数、实现哈希、gDist 身份与新采集的源代码快照 |
| `reconstruction.usda`、`reconstruction_dependencies.json` | 后渲染场景与外部资源清单；资源清单不等于完整可移植归档 |
| `raw_capture_summary.json` | 原始哈希、物理步数、写入耗时、`pending_offline`、实际任务结果 |

`obs[t]` 位于 `action[t]` 之前，来源步号 `4*t-1`；第一帧来自初始状态，尚无四步接触窗口。随后每个动作保存四份实际物理状态，含世界位姿、COM 速度、关节 q/dq/力矩/限位、接触点/冲量、实际命令提交。坐标 m、s、rad，四元数 xyzw。缺失权威故障状态明确为 null，不能写 healthy。

v2 结束时另外保存一次 `terminal_observation`，来源为最后动作后的实际状态，不包含下一条动作。N 个动作有 N 个动作前观测和 1 个终止安全采样；离线共 N+1 个时刻，视频包含相同的终止时刻。这样最后四个物理步的接触不会遗漏。历史 v1 缺少末尾采样时，不宣称最后窗口已经获得标签。

原生入口 `--seed` 控制环境，`--placement-seed` 控制布局，未指定后者时沿用前者；默认 42。实际抽样值另存。未配置材质/光照变化时不能仅因改变种子就声称完成四维联合随机化。

## 离线结果格式与验收

`risk/native_000/safety_decisions.jsonl` 每行包含动作索引、源物理步、源时间、phase、operation_id、geometry、features、report；摘要标 `offline_replay`。按 frozen gDist 完整三角形度量查询。几何省略关系不输出虚假零距离。TTC 仅作诊断。按已知适用分项最高等级聚合，并单独保留缺测/适用性。

评价器拒绝源代码、规则、几何或后端身份不匹配。历史数据可在隔离的匹配版本中回放；显式迁移另存目录并给出迁移来源。不要覆盖当前运行环境去回放历史数据。

`evaluate` 会在加载 GPU 后端前自动执行同一原始预检，结果保存为 `risk/native_000/raw_preflight.json`，摘要记录于 `evaluation_summary.json.raw_preflight`。独立 audit 可提前诊断，但不能省去评价入口自己的核对。详见 [强制原始预检](native_evaluation_preflight.md)。

`audit` 只审计原始时间序列、实测通道和命令存在性；`passed` 不等于控制器成功、材料阈值标定或场景安全准入。

渲染使用保存的 USD 和实测位姿，必须证明物理步数不变并对齐标签源时间。当前三路相机为明确标记的离线概览相机，不冒充原始腕部观测。画面遮挡、噪声、动态帧新鲜度须额外验收。视频只保存服务器和用户本地。

正式批量前分别验收：任务控制器、资产角色/操作表面、随机范围、原始通道、规则适用性、完整运动轨迹和视频对齐。未知的铰链停止模型、损伤及故障证据继续为未知；不修改等级阈值来消除未知。

搬运物支撑证明复用冻结的支撑阈值：绑定到本任务允许的支撑/容器表面，有实际非零冲量，接触点低于几何中心，法向接近竖直、浅穿透且物体低速。单看法向或阶段名不足以证明支撑。掉落需要已知未抓持、已知无支撑和既有下降条件；抓持/支撑证据含糊时保留未知，不把正常受控下降直接当作掉落。

### 人体表面数据范围

`--obstacle-binding` 可含 `hs_surface_omissions`，必须提供 `source_usd_sha256`、非空 `relative_colliders`、`authorization` 和 `reason`。路径相对于本次人体资产根，禁止通配符、跨根或把人体全部移除。检查当前源资产版本和实际加载的碰撞清单后才生效。固定结构由当前物理关节图绑定；HS 距离和接触均排除，RS 保留独立完整范围。原始审计会确认被排除的接触没有保存。

### 摆放结果与任务验收

新采集在 `variation_snapshot.json` 和 `raw_capture_summary.json` 同时保存 `placement_validation`：`available`、`passed`、`failed_checks`、`attempts`、`final_loss`、`fallback_used`。这些是读取原生放置器本次 reset 已有结果；不在线计算风险距离、风险特征或等级。

`audit_native_task.py` 的 `passed` 只说明原始证据时序/覆盖/一致性审计通过；另看 `placement_accepted`，它不能被审计成功覆盖。备用摆放、不可达或未知检查结果仍保留原始数据供失败分析，但不能据此声明完整任务已验收。用户当前要求为评估体系正确性，无每组 200 条配额；这些记录只用于必要验证。

### 跨任务障碍放置

`placement_mode: native_relations` 使用当前任务资产作为参考，并要求 `task_sha256` 精确匹配本次任务契约。`placement_relations` 每项给出 `kind`、`params`，二元关系另给 `reference`（任务资产 ID）。支持 `on`、`next_to`、`face_to`、`rotate_around_solution`；未知字段、跨任务哈希、未声明资产与混用世界位姿均拒绝。

示例语义：玻璃杯 `on` 当前 `counter_top`，并 `next_to` 本次 `mustard_bottle`；由原生放置器联合求解本次实际摆放。这些 ID 只是对应任务实例，其他任务须选其自己的资产。当前原生 live sampler 没有把 `RandomAroundSolution` 写入最终布局，此标记会明确报错。使用 `placement_parameter_ranges` 在求解前抽取 `next_to.distance_m`、`next_to.cross_position_ratio` 或 `rotate_around_solution.yaw_rad`；每项显式给出 `relation_index`、`parameter` 和 `uniform: [min,max]`。不随机修改验证容差。抽样基于本次 placement seed，记录 RNG 身份、Python 版本和所有实际值；采样范围不得从旧场景自动继承。

原生放置器用于配置场景，风险距离与等级依旧离线。声明的相对距离是摆放约束，不是 HS/PT 安全测距结果，也不是“安全”证明。每次采集保存声明、种子、实际采样位姿与摆放检查。`on` 也不证明运动过程中的稳定支撑；离线支撑/掉落仍按实测接触判断。

旧 `placement_mode: world_pose`（默认）继续要求完整 `position_xyz` 和 `rotation_xyzw`，只能用于已明确核对过本次任务的位姿。相对放置入口不自动声明各任务 12 种风险变体或四维随机范围已经验收。

静态关系资产使用独立的 USD 根位姿写入器，保持静态碰撞体，不改为运动学刚体。保存原生抽样位姿、写入后的世界矩阵和回读误差；逐条采集仍须检查碰撞清单、实际 PhysX 场景与离线风险范围。


### 离线外观与相机配置

新增 `--appearance-config` / `--appearance-seed` 和 `--camera-config` 渲染选项，格式与限制见 [native_offline_appearance.md](native_offline_appearance.md)。外观只改变视觉输入；世界相机配置同时绑定任务与原始数据哈希。视频导出保存并校验外观、相机文件。图像非黑不代表目标可见，必须单独检查遮挡。


### 单条数据的终止边界

原生环境在初次 reset 之前配置 `configure_episode_limit(1)`，任务结束不启动替换 episode，防止重新摆放场景或重置成功标志。记录 `native_episode_index`、`native_episode_limit`、`native_outcome`（实际终止/超时标志与已有任务判定）以及 `stop_reason`。最后观测与摘要的来源动作、物理步和 episode 必须一致，原始审计独立检查。控制器主动结束、动作上限与环境终止分别记录；这些字段不等于完整任务验收。

边界诊断仅缩短了独立诊断运行的任务超时时间至 0.05 秒，保持原始任务成功条件；2 个动作后实际超时，8 个物理步和 3 个安全时刻通过审计与 gDist 离线评价。正式任务时间限制未改动，诊断不计入配额。

### Articulation 操作范围诊断

使用 `scripts/probe_native_operation_suite.py` 对当前目录中的开门、按键、旋钮任务运行短段实测保持命令，逐帧验证操作 ID 与阶段联合控制的距离省略、剩余表面、接触规则及独立 RS 范围。命令、输出与验收边界见 [native_operation_scope_probes.md](native_operation_scope_probes.md)。该诊断不执行完整任务，不计正式配额；运行时检查通过不能替代门板停止模型、正常结构接触及完整运动验证。

### 外部关节运动诊断

可选诊断命令及原始输出字段见 [native_articulation_motion.md](native_articulation_motion.md)。该模式明确标为 diagnostic_external_actuation；实际驱动命令在物理步前提交并读回，风险计算仍离线。默认采集不启用。诊断直接推动资产，不能把原生 success 字段解释成机器人任务成功。
