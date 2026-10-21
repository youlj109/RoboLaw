> 历史接入记录（截至 2026-10-20 的早期版本）。当前分支的 FCL 测距、已知结果判级和任务豁免以 [risk_perception.md](risk_perception.md) 及配置文件为准；下文包含旧投影测距和旧缺测聚合描述。

# Arena 抓放任务的只读安全评价

本接入依据 `SafeBench-Arena-MD-Code-20261019.zip`。原始说明和参考代码保存在 `third_party/safebench_reference/`；35 个参考文件已与包内来源清单核对，未修改。复用四类评价器，未导入源端控制器或整个 Simbox workflow。

当前默认 PT 已升级为 **v4**，详见 [PT v4 几何、抓持适配与数据边界](pt_v4_arena.md)。

## 运行边界

- 原 Arena／Isaac Lab、cuRobo/FCL、固定相机、抓放阶段与原安全检查继续使用。
- 新模块只采样、评价和写文件。风险 L0／L1／L2／L3、未知或评价异常都不改变动作、速度、夹爪、规划和任务推进。
- 机器人为运行时枚举的 Arena DROID：Franka 机械臂＋Robotiq 2F-85，13 个实际关节、7 个机械臂目标和二值夹爪动作。关节名称、原始 drive 参数、限位和单位保存在每条数据的绑定文件里。
- 物理频率为运行时的 120 Hz，动作和 RGB 为原来的 15 Hz，风险标签为 30 Hz。四步接触窗口来自被复用 PT 规则的明确统计定义；不改变物理步长来凑标签频率。
- 使用新输出目录。原 nonhuman10 的 200 条及默认人体的 20 条数据和既有 LeRobot 输出保持原样。

## 启动

在宿主工程目录，通过现有容器解释器运行。`arena-kitchen-collect` 必须已启动并挂载本工程到 `/workspace`。

```bash
cd /home/user/LinjingYou/Isaacsim_RoboDojo_A_H_Adaptation

ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/collect_safety_benchmark.py \
  --ids human_default_mpfb \
  --episodes-per-id 1 --max-attempts 3 --num-envs 1 \
  --run-id safety_human_new_001 \
  --output datasets/augmentation/safety_observer_v1_20261019/human_new_001
```

本机 `datasets/augmentation/safety_observer_v1_20261019` 已链接到 USB 数据盘；新批次在其下创建独立子目录。

省略 `--ids` 选择现有 10 个非人体场景和 `human_default_mpfb`，共 11 个。此调度器对非人体保留原有各 ID 的抓取／运输设置；默认人体采用已经验证的 lower-first 设置，避免统一覆盖四个困难场景的参数。`--dry-run` 仅列出实际命令。生产并行设置为 `--num-envs 4`。

继续同一个批次时，保持原参数并追加 `--resume`。`--episodes-per-id` 为原严格验收配额，成功做完任务和满足障碍作用要求分别记录；人体的 `obstruction_unconfirmed` 仍可能使严格配额未满，程序结束码 1 不等于所有任务执行失败。查看下面的 `successful_tasks.jsonl` 和四轴结果。

统一底层入口为 `scripts/collect_obstacle_library.py`，新增 `--observer-config configs/safety_observer_arena.json` 和 `--run-id`；只读评价运行在现有 vectorized 后端。未传入 observer 配置时，旧固定布局和已有采集流程维持原行为。

`finish_default_human_collection.py` 也可显式传入 `--observer-config`，但必须指定新的 `--output`。该模式排除历史人体数据，不将旧 20 条计入新 observer 配额。

## 风险证据与限制

| 类别 | 实际输入 | 保留未知的部分 |
|---|---|---|
| HS 人员 | 全环境人体碰撞体清单；机械臂、末端、瓶到人的距离；实际接触；闭合速度 | 初始接触、没有足够相邻状态的初始闭合速度 |
| PT 物体 | 机器人与目标、碗、厨房各资产组的距离；携瓶与各受保护资产距离；逐资产刚体对冲量和力；真实双指抓持；支撑接触；带时间的掉落候选 | 材料损伤；无法确定抓持或支撑的状态；未覆盖的接触 |
| RS 机器人 | 按实际关节名匹配的机械臂关节裕度、运行时 effort 上限和 actuator torque 比；实际自接触 | 未提供的故障遥测；禁用或无法确认自碰撞报告时的“没有自碰撞” |
| IR 感知／动作 | 动作生成／下发证据及三相机时间来源 | 当前为 GT 几何规划，没有视觉策略的跟踪丢失、遮挡真值、位姿估计误差或危险动作分类器，不能填成 L0 |

PT v4 使用真实 authored 几何的紧致世界 AABB 间距与中心轴投影间距的较大值，仍是保守下界，明确区别于独立 FCL 安全距离；零间距不证明实际接触。旧 AABB 值另存供审计。HS 已采用独立人体几何测距，具体来源记录在绑定的 `geometry_sources.HS`。实际接触证据来自 PhysX，不由 AABB 推断。

力统计为**每个物理步、每个刚体对的冲量向量和的模长除以实际 dt**。PT 使用过去四个已完成物理步的最大值。冲量、净力、接触点力和事件累计冲量不混用。无接触为零仅在对应回调报告覆盖完整时成立。

允许夹持的距离豁免仅针对 v4 配置明确列出的手腕、手爪和手指机构完整路径，以及接近、抓取、抬升阶段；不豁免整条机械臂。两个实际内指仍用于真实双指抓持判定。正常放置的支撑接触单独记录，仍保留力和冲量。掉落候选的速度与下降距离阈值写在 `drop_protocol`，只对搬运相关阶段判断，不把无法确认的状态当成“未掉落”。

## PT 全场景资产接入记录（2026-10-20，随后升级至 v4）

配置入口为 `configs/safety_observer_arena.json` 的 `pt_asset_coverage`，
首次接入版本为 `arena-risk-observer-3-pt-scene-assets`，当时使用 v2/AABB。
当前已升级为 `arena-risk-observer-4-pt-projected`，以下资产范围和支撑策略继续生效。

- 启动时枚举每个环境的已启用碰撞体。机器人和人体分别属于机器人/HS范围，
  从 PT 被保护资产清单排除；芥末瓶保留为目标。
- 碗直接按对象分组；厨房按 `kitchen/<独立资产组>` 分组，
  例如台面、冰箱、橱柜、抽屉、水槽、灶台、微波炉、洗碗机、咖啡机、
  烤面包机、刀架以及墙地面。新出现的其他顶层资产也自动进入清单。
  门、把手、抽屉的碰撞体保留在各自资产组中。
- 每个受保护环境资产生成 `robot → asset` 和 `held_object → asset` 两个评价对；
  目标瓶保留原 `robot → mustard_bottle` 评价对。搬运物距离是否适用继续使用实际抓持证据；
  抓持未知时保留未知，释放后仍保留物体与资产的碰撞力、冲量。
- `safety_observer_binding.json` 中的 `binding.pt_asset_inventory` 保存每组
  ID、完整路径、碰撞体清单、排除清单和支撑策略；`resolved_profile` 保存逐对象阈值与评价对。
- `safety_physics.jsonl` 的距离表增加各资产；`safety_decisions.jsonl` 的
  `input.features.pt.protected_pairs` 保存逐对象原始距离、各链接距离、力、冲量和接触依据。
  静态资产接触按 `collider0/collider1` 精确归属，再计算每刚体对净冲量；
  不以共享的厨房上层刚体路径替代资产身份。

### 允许支撑的距离项处理

夹爪—瓶的豁免使用当前 v4 精确路径配置。新增的支持关系仅作用于搬运物：

| 对象 | 允许的任务阶段（归一化后） |
|---|---|
| 碗 | place、release、retreat、settle |
| 主台面、右台面 | approach、grasp、place、release、retreat、settle |

阶段匹配还不足以获得豁免。必须同时有最新物理步完整的接触覆盖、真实接触点；
全部接触点在目标碰撞几何中心以下；法向竖直分量绝对值至少 0.8；
穿透不超过 3 mm；物体线速度不超过 0.02 m/s；原始距离有效。
这些数值为显式记录的工程代理条件，未做材料或物理支撑资格校准。
机器人碰到碗或台面不在此支撑白名单内，侧撞、快速碰撞、深穿透、缺证据均不获得距离豁免。

实现保留原评价器结果的 `source_risk_levels`，只对符合条件的
`PT-DIST-001` 标记 `arena_support_distance_exempt`，并在
`arena_pt_support_overrides` 保存原等级、原缺测项及关系 ID，再聚合 PT。
力、冲量、掉落和损伤规则不被豁免；无损伤传感数据仍为缺测。
因此正常支撑不必持续触发距离 L2，但过大支撑力仍能触发 PT 风险。

`report.arena_pt_distance_summary` 提供最近有效评价对的对象 ID、actor、距离、
有效适用对数量及缺测数量，排除已证明的支撑豁免、未适用搬运物和正常夹持豁免。
显示时应使用“PT 已覆盖资产的最近有效间距”，同时展示对象名和当前投影几何来源；
不得把存在缺测的最小值称为全场景真实最小距离。

### 数据与部署边界

本次改动只接入代码和默认配置，没有重新采集或重算历史轨迹，也没有重制视频。
旧轨迹保留原版本与旧覆盖范围；新采集必须使用新的 run/output。
新资产沿用 `protected_obstacle` 的未校准实验代理参数，不代表各材料已校准。
仅有视觉模型而没有启用碰撞体的资产不在这一几何覆盖范围内。
本次未运行测试或仿真采集，实际运行结果待后续采集确认。

完整 profile 位于 `configs/safety/profile.robot.pick_place.v4.json`，每条轨迹会按实际场景重新绑定。所有跨机器人停止模型、材料力和冲量阈值均标记为**未校准的实验代理**。这些是可追溯的仿真 benchmark 标签，不能据此声称 ISO 认证或实体机器人安全限值。

## 时间与无介入审计

风险观察槽 0 来自首次录制动作前的独立状态，接触未知；槽 1 来自已经完成的第 3 个物理步。当前 `arena-causal-physics-2` 在原 `scene.write_data_to_sim()` 提交命令前复制输入并完成评价，保存单调时钟供提交先后校验。评价器只持有普通 Python 数据，不能访问机器人写接口。物理步必须逐一引用已经存在的风险观察槽，尾段也保留。

原动作继续经 Isaac Lab actuator 处理。当前实现记录原始批量发送的完整环境索引以及本轨迹对应的后端行，不能把单环境行号冒充整个调用的发送掩码。审计拦截真正的 PhysX `root_view.set_dof_*` 提交边界，保存 incoming 和 submitted 数值、顺序、控制通道和调用次数，并与原 `trajectory.h5/action` 对比。验证器不是只检查 `mode=shadow` 配置。

每个环境使用 run、attempt、env、episode generation 和 local physics step 联合身份。现有执行器在一批中不会实际 reset 完成的环境；下一批新建 observer 和身份。按完整刚体路径过滤接触，不依赖可能重复的 env buffer 下标。

RGB 流水线重建录制的 `obs[t]` 并渲染，不推进原物理。相机记录 source action／physics step、环境身份、capture／receive 墙钟、序列和状态。离线图像明确标记为 `offline_measured_state_render`，不冒充动作执行时已经可用的视觉感知。缺帧、重复和跨环境身份不能通过验证。当前验证器还核对相机序号覆盖完整的 `0..N-1`、与 source action 一一对应、源模拟时间与原 action 周期一致，防止整段错移仍因帧数相等而漏检。

## 输出与四轴

每个 attempt 保留原轨迹、图像、原验证和独立审计，并增加：

| 文件 | 内容 |
|---|---|
| `safety_observer_binding.json` | 运行身份、DOF、对象／人体／相机绑定、实际频率、完整生效 profile |
| `safety_physics.jsonl` | 每个物理步的世界坐标状态、接触点／法向／冲量、距离及覆盖 |
| `safety_commands.jsonl` | 名义动作、实际目标、真实发送边界调用和 unchanged 证据 |
| `safety_decisions.jsonl` | 动作前输入、四类报告、缺失字段、因果引用、评价时间和摘要 |
| `sim_raw_gt.json` | Arena 时序 schema 和上述原始流索引；不是伪装的旧 Simbox 数组 |
| `observer_action_audit.json` | 实际命令提交审计汇总 |
| `safety_risk_summary.json` | 已知最大风险、unknown／complete 覆盖、阶段和耗时 |
| `safety_task_evidence.json` | 实际抬升、释放、最终误差、落稳及新旧成功判据差异 |
| `result_axes.json` | 任务、数据、安全、障碍约束四轴独立结果 |
| `rendered/safety_camera_provenance.jsonl` | 三视角的冻结身份、时间、序列和原状态引用 |

批次 `indexes/` 包含 `all_attempts`、`successful_tasks`、`positive_demonstrations`、`obstruction_confirmed`、`risk_trajectories`、`negative_samples` 六份 JSONL。初始化／采样／规划失败也保留在 all_attempts，不能用零步“成功数据”替代。失败物理轨迹只要有可用的原状态，也进入三视角渲染；缺失状态时保留明确原因。

数据完整性分别保留 `legacy_passed`、`safety_extension_passed` 和 `combined_passed`。安全证据错误不会抹掉已经通过的原数据检查，但整体数据验收仍不通过；LeRobot 的数据结果轴使用整体结果。

原成功判据不被替换。新增落稳证据使用采集前固定协议，两者不一致记录 mismatch。未知风险不会因为任务成功或距离审计通过自动变成 L0。正示范索引要求原独立距离审计通过、指定风险条件、完整性和最终稳定性；因此可能为空。

独立检查一个 attempt：

```bash
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/report_safety_observer.py \
  /workspace/NEW_BATCH/SCENE/ATTEMPT \
  --render-output /workspace/NEW_BATCH/SCENE/ATTEMPT/rendered
```

`validate_collection.py` 对含绑定文件的新数据自动增加 observer 校验。原 `audit_collection_clearance.py` 保持独立。

## LeRobot v3

保留原 8 维 action、13 维关节 state、三相机和任务索引，在监督／评价字段中增加：

- `risk.level[4]`：顺序 HS、PT、RS、IR；未知为 -1。
- `risk.valid[4]`：等级是否有证据；有等级仍可能仅是风险下界。
- `risk.data_complete[4]`：每类规则证据是否完整，与 valid 独立。
- `risk.observation_id`、`risk.source_physics_step`、`risk.profile_index`。
- `result.axes[4]`：任务、数据、安全、障碍；未知为 -1。

每条 action 仅使用其开始时已存在的观察槽，不将同一动作稍后的风险标签前移。GT 标签不放进 `observation.*`，避免默认成为策略输入。全部原始安全证据、完整 profile 和摘要保存在 episode sidecar。

```bash
env -u PYTHONPATH .venv-lerobot-export/bin/python \
  scripts/export_safety_lerobot_v3.py \
  --selection datasets/augmentation/safety_observer_v1_20261019/human_new_001/human_default/manifest.json \
  --selection-scope task_success \
  --output datasets/augmentation/safety_observer_v1_20261019/human_new_001_lerobot_v3
```

统一调度分别在 `nonhuman10/manifest.json` 和 `human_default/manifest.json` 保存各组清单；导出时选择对应清单，上面的示例对应人体单组。

`benchmark` 保留原严格筛选；`task_success` 选择任务、数据和原独立审计通过的轨迹，包括障碍作用尚未确认者；`all_recorded` 允许导出 RGB／状态验证通过的失败轨迹作为负样本，并保留真实结果。输出目录必须是新目录。导出后原生 LeRobot reader 逐行检查原字段、新风险字段、掩码和来源索引，并完整解码全部视频。

配置、profile、执行代码、时钟或 schema 改变时使用新 run。当前实现也把基础 layout 和现有 11 场景的资产目录定义作为普通配置内容纳入比较。摘要及代码版本比较来自压缩包明确要求，复用现有普通配置续采比较；没有增加风险等级驱动的控制门。

## 验证状态

2026-10-19 已完成主仓库部署与接入验收。详见 [验收报告](../reports/safety_observer/acceptance_report.md) 和 [逐项产物检查](../reports/safety_observer/final_artifact_audit.json)。

11 场景各执行一次：8 条任务成功、3 条任务失败，11 条原数据与安全扩展验证全部通过；完整四环境另有 2 成功、2 失败，全部数据通过。默认人体和花瓶保留障碍作用未确认，严格验收计数独立。最终单环境完整轨迹、四环境含失败轨迹及短失败诊断均已通过 LeRobot v3 原生回读和视频解码。

主入口相同配置续采没有重复计数或改写既有 episode；单项配置变化在启动仿真前被拒绝。原全场景批次保留采集时的代码版本，最终验证器只读复核报告另存，未将旧数据伪装成新版本实采。

## 耗时口径

`sampling_shared_batch_s` 和 `features_shared_batch_s` 是共享物理批次耗时，每个环境报告中都会引用，不能按环境数重复求和。`physics_record_s` 包含部分序列化和写入成本；`write_s`、`evaluation_s` 与它存在嵌套，也不能直接相加。原生批次 `batch.json` 保留整体墙钟和阶段统计。

风险证据采用同步追加，没有异步风险队列；报告明确记录该模式和写盘墙钟延迟。RGB 沿用原来的有界后台写入器，`rendered/render_report.json/camera_timings` 保存等待、关闭排空和最大 pending batch 数，完整帧数由原验证与新增来源校验共同确认。没有通过风险等级调节速度或模拟时间步。
