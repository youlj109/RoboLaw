# 5090 Isaac Lab Arena：安全采集与只读评估部署说明

面向现有厨房 Pick and Place 工程  版本 2.0  2026年10月19日

> 本 Markdown 版附有源端参考代码。先阅读 [参考代码使用说明](reference_code/README.md)，再按本文实施。代码路径与 SHA256 见 `reference_code/SOURCE_MANIFEST.json`。源代码是未修改快照；Arena 接入层仍需实现。

本说明将现有 SafeBench 四类安全数据采集与评价能力接入 Isaac Arena／Isaac Lab＋CuRobo/FCL 的厨房 Pick and Place 工程。目标是持续记录和评价 HS、PT、RS、IR 风险，而不让新增评价结果修改原动作、速度、夹爪、规划或任务推进。即使标签为 L1、L2、L3 或未知，新增 observer 也只写日志。

部署对象为 Docker 容器 arena-kitchen-collect，宿主机工程为 /home/user/LinjingYou/Isaacsim_RoboDojo_A_H_Adaptation，容器内工程为 /workspace。沿用现有四个并行物理环境、三视角记录、流水化 RGB 渲染和独立安全距离审计。原有 CuRobo/FCL 规划碰撞约束仍作为采集基线。

本次交付包括 Markdown 说明、源端参考代码、建议的新增配置文件、环境探测脚本和验收清单。新增配置是目标适配层的实现契约，必须由工程读取后才生效；它不是现有入口已经支持的隐藏开关。目标入口和数据状态依据实际使用信息整理；源端评估 API 已定向核对，目标工程的运行时绑定与仿真验收属于下述实施步骤。

## 一 当前工程与交付范围

| 项目 | 当前情况 |
| --- | --- |
| 宿主工程 | /home/user/LinjingYou/Isaacsim_RoboDojo_A_H_Adaptation |
| 容器及挂载 | arena-kitchen-collect；容器内工程路径 /workspace |
| 执行链路 | Isaac Arena／Isaac Lab → CuRobo/FCL 规划 → 物理执行 → 三视角记录 |
| 并行方式 | 4 个物理环境；RGB 渲染与物理执行流水化 |
| 场景范围 | 10 个非人体场景，加 human_default_mpfb；当前共 11 场景 |
| 已有人体数据 | 20 条通过任务完成、数据完整与现有安全检查；仍标记 obstruction_unconfirmed |
| 本次新增 | 四类只读风险评价、物理级证据、阶段与动作审计、报告及导出字段 |
| 仍需绑定 | 真实机器人实例和 DOF、夹爪模型、相机名字、控制与物理频率、材料及停止模型参数 |

正常示范、风险轨迹和负样本分别建立索引。当前 20 条只能称为通过前三项检查的人体场景轨迹；不能写成 20 条已证实由人体挡路而产生绕行的样本。后续新增四类标签也不能自动消除 obstruction_unconfirmed。

## 二 四个独立结果轴和风险标签

| 结果轴 | 回答的问题 | 结果解释 |
| --- | --- | --- |
| 任务完成 | 是否抓起、搬运、放置、释放并满足最终任务条件 | 保留原成功判据及其版本 |
| 数据完整 | 三视角、状态、动作、时间轴和新增安全数据是否齐全且连续 | 原数据与安全扩展覆盖分别报告 |
| 安全检查通过 | 现有独立距离审计和新增风险规则分别得到什么结果 | 不要将距离审计通过等同于全部风险 L0 |
| 障碍约束确认 | 障碍是否实际限制了名义运动或改变了可行轨迹 | 可为 confirmed、unconfirmed、not_applicable 或 unknown |

HS/PT/RS/IR 是附加的时序风险标签，不替代四个结果轴。保留现有 validate_collection.py 与 audit_collection_clearance.py 的原输出，并在汇总层增加新结果。实际字段名通过显式映射表对接，不能把旧字段原地改名后失去历史语义。

```text
# 建议的汇总结构；属于新增字段规范，不是假定旧文件已有这些键。
task_result: {status, protocol_version, evidence}
data_result: {legacy_integrity, safety_coverage, alignment}
safety_result: {legacy_clearance_audit, HS, PT, RS, IR}
obstruction_result: {status, method, evidence, reason}
observer_result: {mode: shadow, action_modified: false}
```

障碍约束确认需要独立证据，例如固定起终点、随机种子和规划器配置的有障碍／无障碍离线对照，或证明无障碍名义路径与该障碍相交、当前路径避开，并记录所用几何模型与裕量。最近距离较小、人体可见、FCL 没有碰撞都不足以单独证明障碍改变了运动。对照规划结果仅用于审计，不能作为风险触发的替代执行命令。

## 三 实施位置与最小改动清单

| 现有文件 | 接入内容 |
| --- | --- |
| scripts/collect_obstacle_library.py | 统一读取 observer 配置；每个 attempt 分配身份；汇总四轴结果和风险覆盖 |
| scripts/run_nonhuman_augmentation.py | 沿用非人体场景与随机化调度，透传 observer 配置及独立输出目录 |
| scripts/finish_default_human_collection.py | 保留断点续采语义；新的 observer 批次使用新的 run 身份和计数 |
| scripts/collect_kitchen_vectorized.py | 按 env_id 接入物理前后采样、接触、相机 provenance、episode 生命周期 |
| kitchen_safety/vectorized_episode.py | 提供抓取、搬运、放置、撤离阶段；记录原动作和最终下发动作；不消费风险报告 |
| scripts/validate_collection.py | 增加安全扩展完整性、时序、环境隔离、风险输入与执行引用校验 |
| scripts/audit_collection_clearance.py | 保持独立几何距离审计；与时序标签并列输出，不覆盖 |
| scripts/export_safety_lerobot_v3.py | 沿用现有机器人动作定义和三视角；增加风险、有效性、结果及证据指针 |

建议新增 kitchen_safety/risk_observer.py、risk_binding.py、risk_evidence_writer.py、risk_profile_loader.py，以及独立报告入口。名称是实施建议；不是已核实存在的目标文件。risk_observer 只接收深拷贝或不可变数据，不持有机器人动作写入、驱动力修改或规划器重置接口。

源端可复用的核心是 safety_risk/online_iso.py、evaluator_factory.py、iso_input.py、iso_ssm_candidate.py、iso_traceable.py、online_report705.py 及其依赖。取得有权限的源码后，保留来源和文件摘要。不要复制旧 Simbox 整个 workflow 来替换 Arena 的 vectorized 执行循环。

PhysXDataCollector 依赖旧 task/robot/object 接口；它提供字段、接触口径和采样顺序参考，不能在 Isaac Lab 环境中未经适配直接使用。Arena 侧应将自身 Articulation、RigidObject、传感器和 contact 数据映射到评估器输入。

## 四 容器环境检查和版本固定

先保留目标机已经能完成规划与采集的容器环境。无需为此次评价模块重装 Arena、Torch 或 CuRobo。源端 Isaac Sim 5.0 的依赖版本不能当作目标 5090 的锁定要求；核心规则计算主要使用 Python，目标 CUDA 和自定义扩展仍沿用当前可工作的版本。

```bash
cd /home/user/LinjingYou/Isaacsim_RoboDojo_A_H_Adaptation
docker inspect --format '{{.Config.Image}}' arena-kitchen-collect
docker inspect --format '{{json .Mounts}}' arena-kitchen-collect
docker exec arena-kitchen-collect nvidia-smi

# 查明现有脚本参数；help 输出不等于执行新采集。
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/collect_kitchen_vectorized.py --help
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/validate_collection.py --help
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/export_safety_lerobot_v3.py --help
```

随文提供 probe_safety_observer_env.py。将其放到目标工程 scripts/ 下后，通过原 arena_python.sh 运行；该脚本读取版本、GPU、关键文件摘要和随机化配置场景信息，不启动仿真，不修改现有数据。其输出必须来自容器内原采集环境。

```bash
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/probe_safety_observer_env.py \
  --project /workspace \
  --out /workspace/datasets/augmentation/safety_observer_v1_20261019/manifests/environment.json
```

Torch 小矩阵运算通过不代表 CuRobo CUDA 扩展、FCL、PhysX 或三视角渲染都正常。部署前还需原基线短回归。若遇到 GPU 架构错误，再按固定版本处理扩展构建；不要复制 4090 编译缓存到 5090。驱动要求按实际 Isaac Sim 版本核对，官方资料见文末。

## 五 配置文件与读取规则

建议新增 configs/safety_observer_arena.json。随文提供同名文件，内容为实施模板。目标代码需要显式加载、检查必填 binding，并将最终生效配置写入每批次 manifest。仅把文件放入 configs 不会自动生效。

模板默认采用 shadow，所有 observer 写动作权限为 false；pilot.num_envs 已设为 1，production_reference.num_envs 为 4。30Hz 为源数据参考频率，需要与目标实测物理周期核对。真实机器人、三相机和物体绑定留空，启动验收前必须填入运行时身份，不能用臆造的 Panda 名称填充。

```json
{
  "schema_version": "arena-safety-observer-config-proposal-1",
  "implementation_required": true,
  "mode": "shadow",
  "enabled": true,
  "action_authority": {
    "modify_commands": false,
    "retime_trajectory": false,
    "change_gripper_drive": false,
    "risk_based_replan": false,
    "risk_based_hold": false,
    "risk_based_termination": false
  },
  "recording": {
    "retain_all_attempts": true,
    "record_each_physics_step": true,
    "record_submitted_commands": true,
    "unknown_policy": "preserve_null_with_validity"
  }
}
```

完整模板还包含容器内输出目录、随机化配置、human_default_mpfb 白名单、时序策略、机器人绑定、profile 路径和四轴输出。其字段是目标适配层契约，不与旧 Simbox safety_evaluation 或 safety_eval 字段混用。旧系统中 shadow 与 intervention_policy.enabled=false 的含义可以参考，但 Arena 必须通过实际执行路径证明没有介入。

若实现者复用了旧控制组件，应同时关闭旧 safety_gate、bounded_braking、gripper_force_response、supported_grip_probe、pt_obstacle_retreat、stop_identification、controlled-input hold 和风险结束逻辑，并禁止加载源 Franka 专用 drive profile。本次推荐只导入评估器，因此这些控制组件原则上不进入目标执行链路。

新增 observer 的评估异常不得变成隐式 hold 或风险等级 L2。保存 error、unknown 和缺失证据，任务仍依原逻辑执行。输出盘满、设备故障等原有基础设施故障可以按原流程终止，但应明确 termination_source，不能伪装成风险介入。启动前配置缺失可以拒绝开始新批次，这是配置验收，不是运行中风险控制。

## 六 四环境生命周期与三视角流水对齐

每个物理环境需要独立的 observer 状态。唯一身份至少由 run_id、attempt_id、env_id、episode_generation 和 local_physics_step 组成。env_id 被 reset 后会复用，因此不能单独用 env_id 或 GPU buffer 下标关联 episode。

| 阶段 | 必须执行的记录或约束 |
| --- | --- |
| 创建或 reset | 生成新 attempt 身份；清空该环境历史接触和 observer；记录初始配置和对象姿态 |
| 发送动作前 | 记录观测、任务阶段、风险输入及原名义命令；只使用已完成的物理状态 |
| 控制器发送边界 | 记录实际关节顺序、控制模式、目标、夹爪与 effort；逐项审计 |
| 每个 physics substep 后 | 采集关节、物体、接触、距离与执行事实；保存实际 dt |
| 提交渲染 | 冻结 env_id、generation、physics step、相机名、捕获时刻和序列 |
| 异步图像返回 | 按冻结身份归档；禁止按当前 env_id 归到 reset 后的新 episode |
| 结束 | 保存失败或成功状态；等待本 episode 图像完成或超时标记；原子提交索引 |

三相机均记录 capture_timestamp、receive_timestamp、acquisition_sequence、camera_id、env_id、episode_generation、source_physics_step 和状态。第三视角不能在导出时沿用旧双相机代码而漏掉。实际相机名称应从目标运行时枚举，不能假定为 head、hand、side。

不能为了让每条记录看似齐全而取“最新到达的图像”填入过去帧。异步完成顺序不等于曝光时间顺序；数据完整要求核对真实采集身份。missing 和 stale 保留独立掩码，禁止跨 episode 前向填充。

标签可以在单独线程或进程中稍后计算，但输入快照必须在动作前冻结，输出保留 decision_time 与 evaluation_complete_time。不要让后台评估器读取持续被仿真改写的 Tensor 或 NumPy view。GPU→CPU 传输和异步事件也要保证快照完成后才交给评估器。

队列满时不得默默丢证据或反馈一个“安全减速”命令。可采用有界缓冲与独立落盘，记录 overflow 并使该 episode 的数据完整性不通过；若采用等待机制，需要记录墙钟延迟并验证不改变模拟时间步和原规划行为。风险本身不驱动背压策略。

## 七 物理时序与评估器 API

源端 ObservationISOEvaluator 使用固定物理步与观察槽协议。例如 120Hz 物理和 30Hz 标签时，每 4 步一个观察槽：槽 0 使用独立初始快照，槽 1 使用第 3 步已完成的状态，后续依此类推。真实碰撞和接触仍按物理步保存，不能只在相机帧上采样。

Isaac Lab 的 env.step 可能包含多个 physics substep。必须查明控制 decimation、实际仿真 dt 和动作下发频率；不能把一次 env.step 当成一个物理步。继续沿用原控制周期，重复或插值命令时记录实际有效命令，不因安全采样而提高 policy 或 CuRobo 规划频率。

现有 physics_stride 要求标签频率整除物理频率。若目标频率不满足，先选可整除的标签频率，或另行实现有版本标记的时间戳协议；不要伪造 120Hz。30Hz 不是强制改动目标物理步长的理由。

```python
# 接入伪代码。adapter 和现有控制对象由目标工程实现。
from safety_risk.online_iso import ObservationISOEvaluator

online = ObservationISOEvaluator({
    'profile_file': profile_path,
    'mode': 'shadow',
    'physics_dt_s': actual_dt,
    'decision_hz': label_hz,
    'runtime_metadata': bound_metadata,
})

# 以下循环按单个环境的 local physics step 理解。
for step in episode_physics_steps:
    snapshot_data = adapter.frozen_completed_history()
    online.evaluate_before_action(
        snapshot_data, step, camera_provenance, simulation_time)
    nominal = existing_control_action_for_this_step()
    submitted = existing_submit(nominal)
    existing_physics_substep()
    adapter.append_post_physics(step, submitted)
    online.record_execution(step)

save_episode_raw(adapter.export_raw(), online.export())
```

evaluate_before_action、record_execution 和 export 是已核对的源 API；示意中的 adapter、existing_* 和保存函数为待实现接点。多环境不是复制四次 world.step：原向量化循环仍只推进一次共享物理，再分别写四个环境的局部记录。每个环境单独维护 step、观察槽和执行引用。

不要每步深拷贝无限增长的全历史。实现者可按源 raw_frame 的历史需求设计缓存和窗口，再与完整历史回放对照验证；不能擅自截断掉落、速度或持续接触所需证据。源接口若仍要求完整数组，先正确接通，再单独做等价性能优化。

## 八 机器人 对象与 PhysX 数据绑定

机器人型号尚未由实例配置明确，应从 collect_kitchen_vectorized 的场景配置与运行时 Articulation 枚举确认。记录机器人实例名、每个 DOF 名称与类型、控制索引、上下限、速度和 effort 单位、夹爪控制通道与测量通道。按名称对齐 CuRobo、FCL 和 Isaac Lab，不能硬编码“前七轴是机械臂”。

源 Franka 导出使用 8维 state/action 与 4×9 物理命令块，这不是目标机器人通用格式。目标应沿用 export_safety_lerobot_v3 已有动作维数、相机布局和语义，再添加安全字段。6轴、双臂、升降轴或不同夹具都需显式模型。

两指夹爪需要两个实际接触组、目标物 identity 和碰撞刚体路径。源 build_grip_evidence_binding 可参考，但它要求两组对置接触并使用旧 robot 标签格式。新后端需建立精确映射，不能填不存在的 panda_leftfinger。吸盘或多指手必须另定义抓取证据。

对象位姿统一为 world_m，并明确 quaternion_order；相对位姿必须在同一时刻和同一坐标系计算。并行环境的平移不能被自动单位推断误当成厘米或毫米。四环境中每个 episode 只引用本环境对象、机器人和人体，显式共享的全局实体另行声明。

接触数据需要碰撞体、接触报告／传感器配置、每环境过滤和逐步 coverage。源 collector 构造时订阅 PhysX contact report；Isaac Lab 目标适配需通过当前版本实际接口实现。只订阅回调、不为对象启用报告，或只统计净力向量，都不足以覆盖所有接触对。

保存每个接触对的 bodyA/bodyB、位置、法向、冲量和采样时间。源端力值由接触冲量向量模长除以实际 dt 得到；若目标接口已提供力，不要再除以 dt。单步最大刚体对力、窗口最大值、求和冲量与净力是不同统计量，字段必须注明聚合方式。

原 FCL 距离、CuRobo 碰撞球距离、PhysX 碰撞形状距离和实际接触证据分别注明来源。不同几何近似不能在一列中混用而不标识。独立距离审计继续保留，必要时对其几何体和在线距离采用同一坐标及配对清单进行交叉校验。

## 九 四类规则与 profile 迁移

| 类别 | 需要接入的事实 | 与当前工程的关系 |
| --- | --- | --- |
| HS 人员安全 | 机器人／末端／物体到人体距离、闭合速度、人员接触与人体清单 | human_default_mpfb 是人员实体；身体与手臂都需覆盖 |
| PT 物体与任务相关安全 | 目标与环境距离、接触力和冲量、掉落、支撑、损伤 | Place 支撑接触与异常碰撞需区分 |
| RS 机器人安全 | 自碰撞、关节裕度、力矩比、故障与故障后运动 | 原 FCL 无碰撞不替代力矩与关节证据 |
| IR 感知和动作风险 | 遮挡、丢失、位姿误差、危险动作生成／阻止／实际下发 | GT 或几何规划不等于视觉策略；没有观测器的项保持未知 |

评估器 family 固定为 ssm_priority_candidate_v1；源核查版本为 ssm-priority-four-category-candidate-1.7-hs-pre-skill-relation。完整 profile 使用 iso-profile-1，包含 binding、rules、input_mapping、parameters、references 和所选 PT 模式的配对规则。不能用一个新 YAML 中的四个距离阈值替代完整参数文件。

新 profile 必须重新绑定 robot_id、operation_mode=simulation、payload_profile、object_class、人体实体、环境受保护对象、允许接触 link 与技能阶段。对 human_default_mpfb 不得声明空人体清单；只有确实无人并完成闭世界枚举的非人体场景才能采用无人适用性规则。

停止反应时间、制动时间、制动距离、几何不确定性、材料力和冲量阈值都需要来源。跨机器人复用未校准数值只可标成实验代理，不能伪装实测。关闭实际介入并不使 SSM 停止模型参数失效：其仍作为风险评价的参考假设，并应明确不是本次实际停止结果。

允许夹持接触的豁免按指定 link、对象和阶段定义，不豁免整条机械臂；正常 Place 的支撑接触也需阶段规则。豁免距离判断不等于删除接触力、冲量、夹持滑移与释放后倾倒记录。

```python
# 在项目原容器 Python 环境中调用已核对的规则入口。
from safety_risk.iso_traceable import load_json
from safety_risk.evaluator_factory import create_evaluator

p = load_json('/workspace/configs/safety/profile.robot.pick_place.v1.json')
assert p['evaluator_family'] == 'ssm_priority_candidate_v1'
e = create_evaluator(p)
print(e.profile_hash)
report = e.evaluate({'features': {}, 'metadata': dict(p['binding'])})
assert report['data_complete'] is False
```

上例只验证入口、配置结构和未知处理。profile 能加载不代表传感器或几何绑定正确。每条评价记录需要 input_sha256、profile_sha256、evaluator_version、binding_verified、四类等级和缺失字段。源 profile 文件字节摘要与 JSON 语义摘要分别保存。

## 十 数据契约与缺失规则

若直接采用源 current_snapshot 适配路径，step_ids 必须从零连续，并且下列每步数组长度与其一致。数组中的实体标签和内容结构要按 iso_input 映射；填同名空数组不能当作测量完成。

```text
robot_human_distance_matrix_gt
ee_human_distance_gt
object_human_distance_gt
link_env_distance_gt
link_target_mesh_aabb_distance_gt
link_target_projected_clearance_gt
object_env_distance_gt
collision_pair_gt
contact_force_gt
contact_impulse_gt
contact_coverage_per_frame
joint_position_q_gt
joint_torque_gt
low_level_command_sent
target_object_linear_velocity_gt
```

同时保存 runtime_physics.physics_dt_s、关节与对象元数据、人体现身清单、三相机 provenance 和任务阶段。PT 环境距离可使用已存在的 pt_environment_projected_clearance_gt 通道；新增来源要记录几何口径。若改走 feature snapshot 入口，应固定其 input_mapping 并验证与源 raw_frame 结果一致，不再假装自己是原 PhysX 原始数组。

初始快照应在首次动作前独立取得。没有首步接触、历史命令或速度证据时保持未知，不用 warmup 累计值或未来帧补齐。源 initial_snapshot 明确约束初始接触与命令证据。观察间短接触保留为物理后果，不能事后冒充观察时已知输入。

| 字段组 | 保存要求 |
| --- | --- |
| 状态及动作 | 实测状态、名义动作、最终下发动作、关节名和顺序、单位、控制模式、有效性 |
| 风险 | HS/PT/RS/IR、级别、理由、data_complete、valid、缺失项、输入／配置指纹 |
| 介入 | mode=shadow；applied=false；记录基线控制行为的真实来源 |
| 物理 | 接触、力、冲量、距离、位姿、速度、dt、env 和 episode 身份 |
| 视觉 | 三个真实相机通道；捕获和接收时刻；序列；age；缺失或陈旧标记 |
| 结果 | 四轴结果、任务各阶段、失败原因、原成功与扩展成功协议 |
| 运行异常 | 评估错误、队列溢出、图像超时、reset、基础设施终止原因 |

JSON 未知用 null，导出张量可用 -1 加 valid mask。无接触必须有 coverage=true 支持；未覆盖不是零力。损伤未知不等于未损坏。intervention.level 为 L0 或 none 只说明 observer 未介入，不能替代四类风险。

## 十一 Pick and Place 阶段与示范质量

vectorized_episode.py 应输出或映射 approach、grasp、lift、transport、place、release、retreat、settle 阶段。保持现有任务成功定义，同时新增可追溯的阶段证据。末端到达目标点不等于物体已稳定放好；抓取成功也不能直接等同于完整 Pick and Place 成功。

至少记录物体实际抬升、抓持证据、运输到达、释放时刻、目标区域、终态位置与姿态误差、最终稳定性和观察时长。任务位置公差、稳定速度阈值和持续时间在采集前固定，并写入 protocol_version。新结果与旧 success 不同就保留 mismatch，不覆盖历史字段。

安全评价与示范筛选分开。用于正常模仿的正示范通常需要任务通过、数据完整、动作语义有效、终态稳定以及指定风险条件满足；障碍约束确认是额外筛选轴。失败和高风险 attempt 仍保存，用于风险评估或负样本。

当前 SafeBench 示例说明这项区分的必要性：第一条轨迹抓起杯子约 15.31 厘米且最终 grasped，但 PT 有 L1；第二条已知最高风险全 L0，原始抓持却标记 slipping。该样例只有拿起和抬升，不能用作完整 Place 成功判据。

## 十二 保存 报告和 LeRobot v3 导出

新 observer 批次使用独立目录。保持已有 nonhuman10_n4_20261019 和 human_default20_n4_20261019 原始数据及其 lerobot_v3 不变；历史数据补评时另存带 profile/version 标记的派生结果。历史日志缺少接触或因果时钟时不能补造真值。

```text
datasets/augmentation/safety_observer_v1_20261019/
  manifests/
    environment.json
    resolved_observer_config.json
    resolved_risk_profile.json
    source_files.sha256
  attempts/<attempt_id>/
    <原采集数据与三个相机记录>
    sim_raw_gt.json
    observer_action_audit.json
    risk_report_index705.json
    safety_reports/
      online_risk_episode705.json
      online_risk_timeline705.json
    result_axes.json
  indexes/
    all_attempts.jsonl
    successful_tasks.jsonl
    positive_demonstrations.jsonl
    obstruction_confirmed.jsonl
  lerobot_v3/
```

目录和 result_axes 等名称是本次建议新增的输出契约。可以沿用现有 episode 路径实现相同语义，但必须明确 all_attempts 与仅成功数据的关系。中途失败也落盘；不得只把进入成功 quota 的轨迹交给风险汇总。

在线原始流兼容 online-iso-observation-stream-2 时，可在项目原 Python 环境中调用 write_episode_reports(episode_dir)，或 python -m safety_risk.online_report705 <episode_dir>。该模块校验 records、执行引用与摘要，仅汇总已记录判断；它不重新算风险。使用独立的小脚本通过 arena_python.sh 调用即可，脚本入口需要实施者添加。

```python
# 建议新增报告脚本的核心；不要假定此文件已在目标工程存在。
from safety_risk.online_report705 import write_episode_reports
result = write_episode_reports('/workspace/.../attempts/ATTEMPT_ID')
print(result)
```

如果 Arena 采用了不同时间协议，不要通过删除校验使其强行兼容 705。应实现有新 schema 的报告器，并保留相同的因果、摘要和覆盖检查。报告中的最高等级是已知观察槽最大值，不等于每个物理瞬间都已评估；全物理接触事件仍独立保留。

export_safety_lerobot_v3.py 应增加 risk.level[4]、risk.valid[4]、risk.data_complete[4]、来源 step/observation id、profile 标识和四轴结果。保持现有机器人 action、state、三视角和任务索引不变。物理命令块的长度由实际频率比例决定，维数由真实动作通道决定，不能复制旧 Franka 的 4×9。

导出后逐行核对动作、mask、风险、来源时间和图像帧；检查 episode 边界及随机中间片段。原始证据放 sidecar 并写 SHA256。策略输入只包含任务允许的可观测通道，仿真 GT 与标签用于监督和评价，避免把答案直接泄漏给策略。

## 十三 实际启动与断点续采规则

当前已使用的命令如下。它用于补齐默认人体场景成功数，现有批次已达到 20 条，再运行只会确认完成并退出；这不是 observer 新代码已经执行的证据。

```bash
cd /home/user/LinjingYou/Isaacsim_RoboDojo_A_H_Adaptation
ARENA_CONTAINER=arena-kitchen-collect \
bash scripts/arena_python.sh scripts/finish_default_human_collection.py \
  --target 20 \
  --max-attempts 100
```

实施时先查看各入口 help 与实际解析逻辑，确认新批次输出目录、run_id、场景和环境数如何传入。若当前没有 observer 配置参数，需要新增一个明确的 CLI 参数并向内层透传；不要写入一个未被读取的环境变量就当作已启用。本文不伪造现有入口的 --output 或 --observer-config 选项。

新的冒烟批次建议由现有统一调度入口或专用小入口启动：1 环境、固定 seed、1～3 个正常 attempt、独立输出；通过后扩展为原 4 环境。人体场景只选 human_default_mpfb，非人体场景从 augmentation_nonhuman10.json 原键读取。augmentation_human7.json 的历史文件名不能被解释为本批次必须执行七个人体场景。

续采兼容键至少包含 run_id、场景、机器人与资产身份、随机化配置摘要、observer 配置摘要、profile 摘要、代码版本、时钟协议和导出 schema。任何关键项变化都建立新 run，避免旧 20 条成功数据被错误计入新的 observer quota。

新批次可继续以成功数作为采集目标，但同时统计尝试数、失败数、数据完整率、各风险等级及 unknown 覆盖、独立距离审计结果、obstruction_confirmed 数。结束时先完成本 episode 数据归档；重新启动不能把同一 attempt 写两次。

## 十四 无介入验收与覆盖测试

最强的无介入证据来自同一次运行的最终发送边界：比较名义命令与实际提交命令，并审计全部 actuator 写入来源。两次视频看起来相同或 success 一样不够；跨次物理与规划非确定性可能影响轨迹。需要同时验证风险评估没有改变随机数消耗、规划频率、碰撞模型、drive 参数或模拟时间步。

| 测试 | 通过标准 |
| --- | --- |
| 配置与依赖 | 环境已固定；真实绑定齐全；profile 可读；mode=shadow |
| L0 到 L3 及 unknown | 模拟评估器输出各级别时，发送边界命令和任务推进保持原有逻辑 |
| 评估异常 | 保存 error/unknown；不产生新增 hold、replan、调力或等级终止 |
| 逐步动作审计 | 关节顺序、控制模式、位置／速度／effort／夹爪／mask 对应一致 |
| 四环境与 reset | 接触、时钟和相机没有跨 env 或 episode_generation 混入 |
| 三视角异步 | 乱序返回、延迟和缺帧可识别；不使用下一 episode 图像填旧帧 |
| 短接触与尾段 | 相机之间碰撞被保留；不足一观察槽的物理尾段有正确引用 |
| 场景四轴 | 任务、数据、安全和障碍约束分别汇总；保留 unconfirmed |
| 导出 | 原始与 LeRobot 逐行数值及帧索引一致；未知有效性保持 |
| 性能 | 统计采样／评价／写盘延迟、队列峰值与掉帧；没有无记录的背压 |

先用 mock 报告和模拟 controller sink 验证 L1/L2/L3/unknown/error 不改变动作；这项测试不必真的让机器人碰撞。然后在仿真中使用现有场景验证采样覆盖，不为了让测试变绿而修改阈值、隐藏失败或把 unknown 写成 L0。

任务数据检查继续运行 validate_collection.py，独立几何安全继续运行 audit_collection_clearance.py，并增加 observer 专项校验。它们的具体参数按目标 help 输出确认。新测试需检查命令值和调用次数，而不只是检查配置文件 enabled=false。

源端已运行 4 组共 18 项纯 Python 测试并通过，覆盖冻结输入回放、未知处理、因果顺序、执行引用、抓取绑定、短接触及陈旧相机。它们可作为移植回归基础，但不代表目标 Arena 的传感和四环境接入已验证。

```bash
python -m unittest \
  safety_risk.tests.test_online_iso \
  safety_risk.tests.test_online_observation \
  safety_risk.tests.test_grip_evidence_binding576 \
  safety_risk.tests.test_observation_cadence -v
# 上述 python 必须为 arena_python.sh 使用的同一容器解释器。
```

建议顺序：原环境基线 → 单环境只采集 → 单环境 shadow → L0至L3与异常 mock → 4环境正常任务 → MPFB人体场景 → 全部11场景 → 导出与断点续采。每步都保留证据，再扩大规模。

## 十五 排错与完成标准

| 现象 | 优先定位 |
| --- | --- |
| 再次启动马上退出 | 旧人体 run 已有 20 条成功；是否使用了新的 observer run 身份 |
| 一直得到 L0 | 是否把 intervention 值误作风险；是否以空数据假装覆盖；人体绑定是否漏掉 |
| 大量 unknown | 传感覆盖、单位、实体名、profile 与 runtime binding；不要用零填缺失 |
| shadow 仍改动作 | 外层风险 gate、旧控制组件、observer callback 写入、技能退出条件 |
| 并行环境滑移异常 | 世界／局部坐标、单位推断、对象和 EE 是否同环境同时间 |
| 第三视角缺失或串帧 | 异步任务冻结身份、相机命名、reset generation、导出双相机假设 |
| 距离通过但 PT 或 RS 异常 | 不同评价事实和几何口径；检查接触、力矩与关节证据 |
| 任务成功但 obstruction_unconfirmed | 没有独立证据证明障碍影响运动；保留当前状态 |
| LeRobot 与原始不一致 | 动作维数、物理频率、episode边界、mask和视频时间索引 |
| GPU 或渲染失败 | 先复现原容器基线，再检查版本与扩展；不要将失败当成安全结果 |

完成标准：现有 Arena CuRobo/FCL 采集可继续运行；新增四类记录和报告可追溯；所有尝试均有数据或明确失败记录；三视角与物理数据可关联；四个结果轴保持独立；逐步审计证明新增安全模块未改写动作；新 run 的续采与导出验证通过。仅配置文件生成或规则单元测试通过，不等于部署完成。

## 附录 源参考与已有数据位置

当前目标数据位置均相对于宿主工程，容器中加 /workspace 前缀：

```text
datasets/augmentation/nonhuman10_n4_20261019/episodes/
datasets/augmentation/human_default20_n4_20261019/
datasets/augmentation/human_default20_n4_20261019/lerobot_v3/
configs/augmentation_nonhuman10.json
configs/augmentation_human7.json
```

源端评估参考为 safebench:/root/autodl-tmp/SafeBench_gpu661；已核对文件字节摘要如下。摘要用于识别实现，未校准的实验规则不因迁移到 5090 自动变成标准认证。

```text
safety_risk/online_report705.py
99392ee219f8d39f9da4d0ec381b15229c179eccb611b1f8cd2e7bf456764c11
scripts/evaluate_iso_risk.py
37c249975968413b812af67ec9f054df414ac7e878d4d39036266b2357e6c13a
safety_risk/configs/iso_profile.franka_scene569_shared.json
5936ce0cec6ddb841cdc41813c77bf9823b6d3825d968becedf374fbfbc836ee
```

硬件参考查询于2026年10月19日。选择与目标已安装版本相符的官方要求，避免按 main 分支文档无意升级现有环境。

Isaac Sim 5.1 Requirements
https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/requirements.html

NVlabs CuRobo 官方安装说明
https://github.com/NVlabs/curobo/blob/main/docs/getting-started/installation.rst
