# 默认采集方式：必要证据在线，风险和媒体离线

这是**所有已配置厨房场景通用**的工作流；不固定场景 ID、人体网格数量或 PT 资产清单。当前人体芥末瓶场景用于验证。入口读取每批自己的场景规格、采集参数及运行时资产绑定。

默认策略在 `configs/risk_capture_workflow.json`，入口为 `scripts/risk_capture_workflow.py`。同一批次使用相同资产拓扑，多个不同场景分别指定输出目录；同一场景的多条随机布局可以同批采集。

## 在线与离线边界

| 在线采录或执行 | 离线处理 |
|---|---|
| 物理推进、实际控制器规划及决定下一动作的任务状态 | 完整 Warp CUDA 网格距离、自碰撞几何 |
| 120 Hz 关节位置/速度、力矩、限位、机器人位姿 | 接近速度、TTC、抓持连续性和掉落证据投影 |
| 120 Hz 已注册刚体的真实 PhysX 路径、位姿和速度 | 原始接触流的力、冲量和四步窗口汇总 |
| 原始接触点/法向/冲量/actor/collider、实际命令与故障事件 | HS/PT/RS 风险特征、分项规则和等级 |
| 30 Hz 动作前观测、厨房关节与部件状态、任务原始信号 | 完整原始记录校验、历史执行/间距审计 |
| 初始化冻结网格、对象/关节清单、坐标、相机、规则和豁免 | 按测量状态渲染 RGB、编码视频、风险字幕和数据集导出 |

相机目前不参与策略输入，故 RGB 全部离线渲染。未来接入视觉策略时，策略实际读取的图像必须在线采录。控制或在线干预依赖的计算也必须留在线上；纯报告计算移到离线。采集仍使用有界异步写进程，结束时排空并 fsync，禁止丢帧。

## 启动与分阶段处理

各场景生成器提供 `episode_specs.json`，可选提供 `collector_overrides.json`；不要把验证轨迹路径当默认输入。以下用 `datasets/my_scene_specs.json` 表示本次实际场景文件。

```bash
# 1. 只运行物理并保存完整原始证据；不计算风险、不渲染 RGB
bash scripts/arena_python.sh scripts/risk_capture_workflow.py capture \
  --episode-specs datasets/my_scene_specs.json \
  --collector-overrides datasets/my_collector_overrides.json \
  --output datasets/risk/my_run --run-id my_run

# 2. 采集完成后，校验原始数据，再离线提取特征和评价 HS/PT/RS
bash scripts/arena_python.sh scripts/risk_capture_workflow.py evaluate \
  --output datasets/risk/my_run

# 3. 完整原物理/间距审计，作为独立的离线报告
bash scripts/arena_python.sh scripts/risk_capture_workflow.py audit \
  --output datasets/risk/my_run

# 4. 按本条数据实际测量状态渲染三路 RGB
bash scripts/arena_python.sh scripts/risk_capture_workflow.py render \
  --output datasets/risk/my_run

# 5. 三路画面叠加本条离线风险标签，生成审阅视频
bash scripts/arena_python.sh scripts/risk_capture_workflow.py video \
  --output datasets/risk/my_run
```

场景没有显式采集覆盖参数时省略 `--collector-overrides`。每阶段只处理指定运行目录。已有结果不覆盖，已完成的阶段无需重复执行。采集完成不会自动启动后面的计算，因此可以先连续采多批，再调度离线任务。

输出结构：

```text
my_run/
  workflow_policy.json       # 本运行保存的在线/离线策略
  episode_specs.json        # 本批场景参数
  workflow_runs.json        # 每阶段命令、时间和完成状态
  capture/env_000/           # 原始流、冻结网格、轨迹、绑定
  risk/env_000/              # 完整特征、规则、等级、CSV 和耗时
  audit/env_000/             # 独立历史执行/间距审计
  rendered/env_000/          # 三路测量状态 RGB
  video/env_000/             # 精简 HS/PT/RS 风险视频
```

采集输出仍是 `pending_offline`。风险输出明确 `offline_replay`，保留源物理时间和实际处理时间。原始任务成功、某类 L0 或审计完成均不自动等于生产准入。

## 跨场景处理

- 人体适用性从当前绑定的人体清单读取；确认无人时 HS 为不适用，PT/RS 继续评价。
- RS 独立机器人几何保存在 `rs_authored_geometry`，不依赖人体场景的 HS 网格。
- PT 资产、目标物与允许关系从本条绑定读取；不沿用验证场景的 44 个对象或 87 对关系。
- 已注册动态刚体按实际 PhysX actor 路径保存，不把财产场景中的可动障碍当作静态模型。厨房关节和部件位姿按评价/观测频率保存。
- 有动态碰撞体却没有可保存位姿通道时，中止并报告缺失绑定；不静默冻结，不用 bbox 代替真实表面。
- 本入口适配本仓库现有刚体/静态网格场景。未来引入形变网格、换机器人或不同任务时，必须补齐对应状态和规则适配，不能仅凭目录存在就声称支持。
- 新归档保存完整无人体 RS 绑定；旧的无人体 raw 归档若没有该字段，离线工具明确拒绝，不补造数据。

完整历史间距审计保留原有几何库和原检查口径，属于独立执行诊断；HS/PT/RS 风险距离使用 Warp，不被该审计结果替换。IR 保持 disabled；损伤、稳定支撑裕量和权威故障状态缺测时继续未知，按已知分项判级。

## 当前验证证据

2026-10-22 默认人体场景：9,168 个物理步、2,292 组标签、76.4 秒仿真。原始采集程序约 321.122 秒；完整离线风险生成 585.079 秒（测距 492.558 秒），在线参考逐条比较另需 18.437 秒。数值特征、规则结果及等级一致；204 条支撑接触清单仅顺序不同，集合一致。

HS：L1 231 / L2 2061；PT：L0 1722 / L1 482 / L2 88；RS：L0 2292。它们是已知有效规则的等级，不能解释为全部证据完整。

该完整验证对应保存的原始轨迹和当时实现；跨场景新增分支的状态应查看各自运行报告。**通用流程不表示其他场景已完成本次端到端验证。** 视频不上传模型库，代码仓库也不保存媒体。

通用适配保存后的短验证：当前人体场景另录 4 个动作、16 个物理步，新增刚体路径/位姿通道与独立 RS 几何已落盘，原始校验通过，在线几何/判级计数均为 0；离线生成 4 组 HS/PT/RS 结果且几何完整。该短诊断在动作预算处主动结束，**不代表新的完整任务成功**。通用几何适配器还对原完整轨迹的 0、573、1146、1719、2291 时刻逐项比较，每个时刻 110 个数值特征差异均为 0。无人体几何分支仅做了构造绑定的组件检查，不能充当实际非人体场景验收。
