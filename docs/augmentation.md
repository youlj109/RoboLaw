# 非人体 10 场景数据扩增

本流程复用现有抓放控制阶段，用 cuRobo 对随机布局重新规划，每个场景目标为 20 条成功物理轨迹，总计 200 条。每条保存原有三个相机的 RGB。现有固定布局复现入口和历史数据保留；本轮不接 Mimic/SkillGen，不训练 BC/RL，不增加人体、新模型或杂物。

当前状态（2026-10-19，Asia/Shanghai）：已按用户要求启动 N4 续采，入口为 `bash scripts/continue_augmentation_n4.sh`。新批次 `datasets/augmentation/nonhuman10_n4_20261019/episodes` 位于 USB，继承旧批次100条已验收数据和239次尝试记录，旧目录不变。用户随后明确确认“钢夹视作没问题”：钢夹作为侧向自身安全障碍，不再要求挡住起终点直连路径。其99条历史轨迹已通过原任务、数据及安全检查，选最早20条计入钢夹配额，另79条列为额外有效数据；1条规划失败仍保留失败。钢夹无需重采，其他场景继续补齐。

钢夹规则变更与选择明细见新批次 `steel_clamp_acceptance_review_20261019.json`。原始 `augmentation_eligible` 与 `obstruction_probe` 保留，复核记录通过 `effective_augmentation_eligible`、`eligibility_reason=user-approved-side-risk` 和 `historical_eligibility_review` 说明依据，原验证另存于相邻文件。20条历史复核不算新增采集吞吐；全批统计分别报告原继承数量、历史重新验收数量和新采集数量。其他障碍仍要求确认原挡路作用，所有场景的物理、安全和数据完整性条件不变。

花瓶原第19次尝试有成功结果文件，但缺少退出状态和最终验证，作为历史未验证记录保留，不计成功；新采集从第20次开始。新批次的 `continuation_import.json` 保存来源与配置变更说明，`collection_run.json`、`collection.log` 和 `episodes/manifest.json` 保存运行状态。大文件通过链接引用旧数据，因此旧目录和USB都应保留。

新接入的 `vectorized` / `pipeline` 正式调度路径此前完成独立花瓶 smoke：N=2，目标4条、预算6次，前4次全部通过正式验收；同目录 resume 已验证，无重复物理采集或 RGB 渲染。这4条性能验证数据不加入正式配额。N4续采的吞吐只计算新增成功数量，累计数据数量仍含继承的100条。以下保留原入口，新增后端及证据见文末，不能带新后端参数直接续采旧配置目录。

## 一条命令运行与续采

在项目根目录运行，机器上同时只启动一个 Kit 仿真任务：

```bash
bash scripts/arena_python.sh scripts/run_nonhuman_augmentation.py \
  --output datasets/augmentation/nonhuman10_20261017
```

脚本依次进行固定布局 10 场景回归、每场景 2 条随机轨迹的小批采集，以及续采到每场景 20 条。小批合格数据计入最终配额。固定布局回归失败时停止，不自动略过失败场景。随机阶段每场景最多 100 次尝试，预算耗尽时报告缺口；不能把任务标为完成。

小批采集采用累计尝试预算 2→4→6→…→100 的轮次调度，每轮覆盖全部场景并跳过已完成 2 条的 ID；首轮最多 20 次尝试即可看到 10 个场景的结果，避免单个困难场景先连续尝试 100 次。每轮报告保存在 `augmentation.json` 的 `pilot_rounds`；自定义 `--pilot-per-id` 时以该值作为预算步长，最后一轮封顶到 `--max-attempts`，不重置已有尝试或种子。

中断后使用相同输出目录和参数：

```bash
bash scripts/arena_python.sh scripts/run_nonhuman_augmentation.py \
  --output datasets/augmentation/nonhuman10_20261017 --resume
```

`--dry-run` 只打印命令，不启动仿真、不创建输出。`--ids ID ...` 可在新批次中选择部分场景。已经开始的编排批次保持该批次的 ID 列表；需要逐场景续采时使用下方底层批量入口。

目标数量只能保持或增加。几何、外观配置、主种子、采集参数和 RGB 输出模式改变时，使用新目录。续采比较普通 JSON 配置内容，不增加哈希或冻结文件。尝试目录和失败记录不会被覆盖；中断尝试也占用一次预算，不会冒充成功轨迹。

采集、验证/审计或预览实际出现 `ENOSPC`（空间不足）或 `EDQUOT`（配额耗尽）时，批次和编排器返回退出码 74、标记 `storage_error` 并停止后续尝试，不通过重复启动消耗剩余预算。已有数据和失败日志保留；若连 JSON 都无法写入，以运行日志中的错误和非零退出码为准，清单可能停留在上次成功写入的时间。恢复可用空间后运行同一条 `--resume` 命令：失败采集保留其尝试编号，验证/审计失败则归档原检查记录并重跑检查，完整物理轨迹继续复用；失败预览重新导出。此处理仅响应已发生的磁盘错误，没有新增容量预检阈值。

`--persistent-kit` 在一个批次内复用 Kit，验证和审计仍使用独立 CPU 子进程。若 worker 已退出、协议中断或不能继续使用，批次与编排器保存 `worker_unavailable` 原因并以退出码 75 停止后续尝试；同配置 `--resume` 会创建新 worker，保留失败尝试及日志。普通规划或物理失败仍按现有预算继续采集。

## 随机化与数据含义

配置位于 `configs/augmentation_nonhuman10.json`：

| 维度 | 默认范围 |
| --- | --- |
| 瓶、碗 X/Y | 各轴相对原位置 ±2 cm |
| 瓶、碗 yaw | ±10°，保持正确支撑高度 |
| 障碍物 X/Y、yaw | ±1 cm、±5° |
| 酒杯、横杆、窄门、尖角设备 | 上述几何随机范围减半 |
| 两盏现有灯光 | 强度乘 0.7–1.3，色温 3500–6500 K；轨迹内恒定 |
| 选定不透明表面 | 颜色通道乘 0.75–1.25 并截断到有效范围，视觉粗糙度 0.2–0.8 |

厨房墙面、台面、柜门、碗及允许修改的障碍表面使用 USD 会话层修改材质。保留瓶身标签贴图、透明玻璃和液体、屏幕、控制面板及安全标识。材质变化不修改摩擦、碰撞和其他物理参数。相机安装位置、内参、1280×720 分辨率、机器人基座及物体模型尺寸固定。

每个 ID 和尝试序号从主种子独立产生几何、外观、规划三个随机种子，因此场景执行顺序或底层入口选择部分 ID 不会改变同一尝试的样本。单个 `episode_spec.json` 同时驱动场景、碰撞世界和记录，避免障碍显示位置与规划位置不一致。

抓取目标与夹爪姿态跟随瓶坐标系变换；旧关节轨迹不会直接平移。继续使用原瓶身接触、倾角、支撑释放、逐物理步安全距离及落稳检查。障碍作用检查在相关阶段执行前进行，端点取所选规划路径的起点（规划时测得的关节状态）和计划终点，不将计划终点称为执行后的实测状态。首先检查这两个关节端点之间的直线插值；未确认时，再检查瓶位姿（携带阶段）或 TCP 位姿的平移直线与最短旋转 Slerp。第二种参考以同一对规划端点的关节插值作为逐点 IK 种子，精确保留两个关节端点，不添加连接段、经由点或绕行路径。其位姿采样间距不超过 5 mm / 0.025 rad，并以不超过 0.02 rad 的关节步长检查每条边，要求相对指定直线的 FK 位置偏差不超过 2 mm、旋转偏差不超过 0.005 rad；超差参考排除，不扩大容差。这些是直连的数值精度检查，不改变碰撞、倾角或任务成功阈值。

每种直接参考均先仅忽略目标障碍，完整检查其余世界几何、自碰撞、关节和携物倾角约束；被其他几何阻挡或不可行的参考直接排除。只有有效直连进入目标障碍的安全距离才确认作用。全部候选参考、相对规划端点的首尾误差、直线偏差与失败原因写入 `obstruction_probe.references[*].reference_attempts`，端点来源写入各阶段参考的 `endpoint_provenance`。参考只做几何查询，绝不在仿真中执行碰撞动作。不能确认的样本保留记录但不计入成功配额；该检查不代表证明所有可能直达运动都不可行。

成功轨迹沿用 8 维动作：7 维绝对关节目标和二值夹爪目标。`RGB[t]` 对应执行 `action[t]` 前的 `obs[t]`；完整保存 `obs`、`next_obs`、阶段和现有接触信息。成功数量是采集配额，不是策略成功率。

## 输出和预览

```text
OUTPUT/
  augmentation.json                 编排进度和最终结果
  regression/                       本次固定布局回归
  episodes/
    manifest.json                   逐次结果、每场景配额、失败分类与统计
    summary.csv                     每场景成功数、尝试数、缺口
    ID/attempt_0000_seed_N/
      episode_spec.json             实际抽样参数和三个种子
      result.json                   执行、静置位姿、外观应用值和挡路检查
      trajectory.h5                 动作、状态、阶段、接触
      images.h5                     三视角完整 JPEG 帧和相机世界位姿
      cameras.json                  三视角相机参数
      execution.json                命令、退出码和耗时
      verification.json             物理、数据、安全及扩增资格判定
      validation.json, audit.json   数据验证和独立安全距离审查
      run.log                       保留的执行日志
```

失败尝试按其实际进度保留已有文件，可能没有完整 HDF5 或视频。分类包含采样无效、规划失败、物理失败、数据验证失败、安全审查失败和挡路作用未确认；启动失败也单独记录。

采集期只保存三路 RGB，不同时编码 MP4。生产阶段全部 GPU 采集结束后，从每场景最早的 3 条成功数据离线生成三个单视角 MP4 和一个拼接 MP4；目标是 30 组、120 个 MP4。已有完整视频会经过帧数、尺寸和解码检查后复用。

任意轨迹可以按需导出，传入实际尝试路径：

```bash
bash scripts/arena_python.sh scripts/export_collection_previews.py \
  datasets/augmentation/nonhuman10_20261017/episodes/ID/attempt_NNNN_seed_N \
  --report reports/augmentation_preview_export.json
```

逐帧图像验证、状态链连续性和独立安全检查仍是成功判定的一部分。预览导出不再占用仿真采集时间，但自身耗时单独记录。

`manifest.json` 记录初始化、规划、物理执行、渲染/采集、写盘和检查耗时、成功物理轨迹/小时、已接受帧数、磁盘体积、失败分布及每场景接受参数的最小值、最大值、均值。实际性能以运行结果为准，没有预设加速倍率。原 `single` 后端的预览耗时不计入“成功物理轨迹/小时”的分母；新增 `vectorized` 后端使用文末说明的整次调用墙钟口径。

全批及每个场景同时提供 `sampled_parameter_statistics`：统计全部已记录尝试（含拒绝样本）在 `episode_spec.json` 中的布局和外观采样值，不混入静置后的实测位姿或外观应用值。与原有 `accepted_parameter_statistics` 对照，可检查拒绝样本造成的分布偏移；有限样本的最小值和最大值不等同于配置规定的完整采样范围。

计时边界：`physics_s` 是整个 `env.step` 的耗时，包含逐物理步安全检查、内部渲染和环境管理；`camera.render_capture_s` 仅统计记录器显式刷新与取图。不能将前者理解为纯 PhysX 计算时间，或将后者理解为所有渲染成本。

新采集轨迹的 `step_profile.json`（同时写入 `result.json` 的 `env_step_profile`）进一步分解 `physics_s`：`physics_backend` 为物理后端步进，`safety_checks` 为逐物理步安全检查（含实际状态回读），`rendering` 为内部渲染调度及相机渲染/读取链，`scene_update` 为场景缓存更新，`other` 为其余环境与动作/观测管理开销。分类使用嵌套排他的主机墙钟计时，并保留分阶段统计；不增加物理步、渲染或 CUDA 同步。这些数据反映调用路径上的实际等待耗时，不能解释为各 GPU 内核的纯计算占比。旧轨迹没有此分项，不能据其 `physics_s` 反推。

物理后端还提供 `physx_simulate`、`physx_fetch_results`、`physx_set_device` 三个子 hook，位于 `simulation_step` 内部。其耗时已包含在 `physics_backend` 中，不应再次相加到总时间。`fetch_results` 包含等待本步完成及发布结果的耗时；函数占时高不等于纯数据拷贝占时高，也不表示删掉该调用就能省去等量时间。该分解保留原来的 `simulate → fetch_results → set_device` 顺序和异常行为。

随机外观在开始记录前进行纯渲染预热，避免把旧外观逐渐过渡到新颜色的前缀当作训练观测。预热至少 128、最多 1024 次渲染，每 16 次读取三视角的 160×90 缩图，与前 16/32/48 次渲染的采样比较；连续 3 次全部满足平均差不超过 0.35、95 分位差不超过 1.5（RGB 0–255）才通过。每次调用 `sim.render()` 推进渲染代次后强制刷新 RTX，物理步数必须保持不变；不写入 JPEG、动作或预览。耗时和逐窗差值保存在 `result.json/appearance_warmup` 及 `images.h5` 同名属性中。随机 RGB 缺少通过的预热记录时，按数据验证失败处理，不计入成功配额；旧失败数据和物理成功事实仍保留。

## 动态相机外参与旧记录修复

新采集器显式启用 Isaac Lab 的 `update_latest_camera_pose=True`，逐帧读取真实相机世界位姿。该选项默认关闭时，即使 RGB 正常更新，腕机的 `Camera.data` 位姿仍可能停留在初始化状态。安装位置没有变化：腕机仍固定在夹爪 `base_link` 上。

`images.h5/CAMERA/position_world` 和 `quaternion_world_ros_xyzw` 分别为世界位置和相机到世界的 ROS 四元数（xyzw，+Z 向前、-Y 向上）；`cameras.json/offset` 使用其明确保存的安装坐标约定，本项目为 OpenGL。第 t 帧对应 `trajectory.h5/time[t]`、`obs[t]`，位于执行 `action[t]` 之前。验证器同时检查外参与该帧父刚体位姿、固定安装变换的一致性，防止静态缓存冒充动态外参。

只对已结束、HDF5 已关闭的轨迹运行修复工具。下列命令默认只检查；发现需要修复时返回非零，不修改数据：

```bash
bash scripts/arena_python.sh scripts/repair_collection_camera_poses.py \
  PATH_TO_COMPLETED_EPISODE --report reports/camera_pose_before_repair.json
```

显式添加 `--apply` 才会修复。原位姿保留在相应相机组的 `original_pose_before_refresh_fix/`，替换值由同帧 `obs/robot_body_pose_w` 和原安装变换推导；JPEG、轨迹、相机内参与物理结果不变：

```bash
bash scripts/arena_python.sh scripts/repair_collection_camera_poses.py \
  PATH_TO_COMPLETED_EPISODE --apply --report reports/camera_pose_repair.json
bash scripts/arena_python.sh scripts/validate_collection.py \
  PATH_TO_COMPLETED_EPISODE --require-rgb --report reports/camera_pose_revalidation.json
```

HDF5 相机组的 `world_pose_source` 区分原生刷新测量与事后推导。修复报告记录原误差、备份位置和修复后误差；重复运行不会覆盖原备份。不应将推导的外参称为重新测量或逐像素复现，先前验证记录也应保留。本轮固定回归的首 7 条旧腕机记录已这样修复，原验证存于各轨迹的 `camera_pose_validation_before_fix/`，随后轨迹使用原生刷新。

## 底层批量入口

在已完成固定布局回归的前提下，可以单独运行采集器：

```bash
bash scripts/arena_python.sh scripts/collect_obstacle_library.py \
  --ids property_tall_glass property_wine_glass property_glass_bottle \
        property_ceramic_mug property_vase property_tablet_stand \
        robot_steel_clamp robot_projecting_crossbar robot_narrow_opening \
        robot_sharp_equipment_housing \
  --randomization-config configs/augmentation_nonhuman10.json \
  --episodes-per-id 20 --max-attempts 100 --seed 42 \
  --record-rgb --preview-per-id 3 \
  --output datasets/augmentation/my_batch
```

小批使用 `--episodes-per-id 2 --preview-per-id 0`；续采时改为 20 和 3，并增加 `--resume`。相同批次允许续采部分 ID，已有其他 ID 的记录保留。随机化入口仅支持这 10 个 ID，不能用它扩增人体场景。

不传新增参数时，原单成功轨迹模式保持可用；旧 `--video` 仍在采集期生成三视角和拼接视频。历史固定布局记录没有随机化配置，不能直接用同目录切换为随机扩增。

CPU 检查可运行：

```bash
bash scripts/arena_python.sh -m unittest discover -s tests -v
```

实际物理完成情况以本批次的 `augmentation.json` 和 `episodes/manifest.json` 为准。代码检查通过不能代替固定回归和 200 条完整物理采集。

## 可选的批量物理与持久渲染后端

`--execution-backend vectorized` 在同一个物理 Kit 中并行执行至多 `--num-envs N` 条轨迹。每批只包含同一 ID、同一障碍资产，各条仍独立采样几何、外观和规划种子。不同 ID 依次调度，不把不同障碍模型放进同一物理 batch。默认仍为原 `single` 后端；旧固定布局入口和 `--video` 行为不变。新后端要求随机化配置、`--record-rgb` 和 `--execution-checks deferred`，不使用 `--video`、`--persistent-kit` 或 `--cpu-rgb-copy`。

物理阶段不录 RGB，保存执行前的实际 `obs[t]`、执行后的 `next_obs[t]`，以及机器人、操作物体和厨房关节/刚体的完整渲染状态。物理检查通过后，持久 renderer 按这些实际观测恢复场景并纯渲染三视角，不执行规划轨迹或推进物理。原相机安装、内参和分辨率保持，`RGB[t]` 仍对应 `action[t]` 执行前的 `obs[t]`；每条使用自己的原始灯光和材质参数，切换时清除 renderer 自有的外观编辑层再重新应用。

不加 `--pipeline` 时，调度顺序为“本批物理→逐条渲染与验证→下一批物理”。加上该选项后，一个物理 producer 与一个渲染/验证 consumer 重叠运行：最多同时存在 **1 个物理 Kit＋1 个渲染 Kit**，另外还有 `--validation-workers` 个以内的 CPU 物理检查进程及独立数据验证/安全审计进程。这不是 N 个 Kit。renderer 可跨轨迹复用；静态障碍布局或模型变化需要重建场景时，仍在其所属 Kit 内处理，不能假定十种资产都具有相同吞吐。

在途上限为 `2×N` 条，包含已经完成但尚未正式提交验收记录的轨迹。只有“目标缺口减去在途数量”仍为正时才分配新 attempt，因此不会为凑配额投机多采；尾批可以少于 N。主线程按 attempt 顺序提交结果。异常时停止新分配，等待已经启动的物理任务结束并关闭、回收 renderer；完成的物理文件继续保留供续采。原主种子、固定 ID 索引、attempt 编号、每 ID 尝试预算和成功配额均沿用，分组或恢复不会重采同一 attempt 的参数。

正式接受仍要求原 `verify_attempt` 全链通过：物理与逐物理步安全检查、瓶身接触/携物倾角/释放与落稳、`augmentation_eligible` 挡路资格、独立安全距离审计，以及全部三视角 RGB、状态链和相机参数验证。原始 native/render 文件保留其“等待批次验收”的来源信息，不靠把原始 `production_quota_eligible` 翻为真来填配额；是否接受以 `verification.json` 和 `manifest.json` 为准。普通单环境规划或物理失败保留原因并消耗一次尝试。共享 native 启动/运行错误则停止本轮，保留真实退出码及 `shared_batch_error`，不连续消耗剩余 100 次预算；退出码为 0 也不能抵消明确的共享运行错误。

以下为已实际执行的 4 条目标、N=2、最多 6 次尝试的独立 smoke 命令；它不会进入 200 条编排。重新独立测试时更换为全新输出目录，现有目录只用于同参数 `--resume`：

```bash
bash scripts/arena_python.sh scripts/collect_obstacle_library.py \
  --execution-backend vectorized --pipeline --num-envs 2 --validation-workers 4 \
  --ids property_vase \
  --randomization-config configs/augmentation_nonhuman10.json --seed 42 \
  --episodes-per-id 4 --max-attempts 6 --record-rgb --preview-per-id 1 \
  --device cpu --physics-threads 0 --torch-threads 1 \
  --camera-device cuda:0 --background-rgb --execution-checks deferred \
  --output reports/collection_speed_20261018/production_backend_smoke_vase4_n2
```

本次 smoke 已完成：4/4 条同时通过物理、完整 RGB、独立安全距离审计和挡路资格检查；两批各 2 个环境，物理 producer 与 renderer 实际重叠。轨迹长度为 630、596、582、566 个动作，共 2374 个动作和 7122 张三视角 JPEG。首条生成三个单视角和一个拼接视频，4 个 MP4 均完整解码为 630 帧。首轮外部墙钟 **302.227 秒**，调度器内部墙钟 302.050 秒；按外墙约 47.65 条/小时。这是 N=2、花瓶 4 条、包含正式审计与预览的小批结果，不能拿它与 N4/N6 的性能 workload 直接比较。

同参数 `--resume` 已实际执行并以 0 退出，外墙 **1.068 秒**。前后均为 4 个 attempt、2 个物理 batch、1 个 renderer、4 个渲染请求/回复；没有新 Kit 工作。184 个原文件仅 `manifest.json`、`summary.csv` 更新，其余 182 个文件的大小、mtime_ns 和 inode 均不变，无文件缺失。只新增第二次预览检查的 JSON/log；原四个视频复用并再次完整解码通过。证据见 [smoke 清单](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2/manifest.json) 和 [resume 只读审计](../reports/collection_speed_20261018/production_backend_smoke_vase4_n2_resume_audit.json)，未使用哈希或重新解码 HDF5 来替代实际恢复验证。

续采使用上述完全相同的命令加 `--resume`。成功目标可保持或增加，其他普通配置必须一致；改变后端、pipeline、N、检查 worker 数、物理/相机运行参数或随机配置时使用新目录。不能用该命令指向原已暂停的 `nonhuman10_20261017` 目录。4 条目标大于 N，允许观察生产/消费重叠；预算内不足目标时仍须如实报告缺口。

新输出保留原 attempt 路径，物理与 RGB 分开保存：

```text
OUTPUT/
  manifest.json, summary.csv
  _vectorized/
    batch_NNNN/
      episode_specs.json, job_mapping.json, collector_overrides.json
      execution.json, run.log
      physics/
        batch.json, validation_scene.usdc, physics_validation.json
        env_000/                    native 原始物理输出
        env_001/
    renderer_NNNN/
      requests/, replies/, worker.log, report.json, lifecycle.json
  ID/attempt_NNNN_seed_N/
    episode_spec.json               原独立采样参数
    vectorized_assignment.json      共享 batch、env 索引及来源路径
    execution.json                 真实共享退出状态，引用 batch 墙钟
    result.json, trajectory.h5      原始物理输出的硬链接，不额外复制大文件
    physics_states.npz, action_clearance_states.npz
    render_state_metadata.json, camera_config.json
    render_execution.json          本条渲染请求状态，非虚构的独立进程退出码
    rendered/
      images.h5, cameras.json, render_report.json
      external_camera.mp4, external_camera_2.mp4, wrist_camera.mp4, all_cameras.mp4
    verification.json, validation.json, audit.json
```

共享物理 batch 的日志/场景与 attempt 硬链接共同组成可追溯输出，应作为同一批数据保留。失败或尚未处理的阶段可能没有对应文件。续采首先复用已完成物理数据，再处理缺失的渲染、验证阶段；已完成的检查不重复执行。renderer 中断或空间不足的部分输出归档为 `rendered.interrupted_NNN` 等文件后重试，不覆盖旧证据；完成后明确失败的渲染/数据验证不会被当作成功。永久失败的共享物理执行消耗原 attempt，恢复时使用新的 attempt，而不是反复接受同一个坏执行。

离线预览显式指向分开的 RGB 目录，视频写入 `rendered/`：

```bash
bash scripts/arena_python.sh scripts/export_collection_previews.py \
  OUTPUT/ID/attempt_NNNN_seed_N \
  --render-output OUTPUT/ID/attempt_NNNN_seed_N/rendered \
  --report reports/vectorized_preview_export.json
```

新后端的吞吐分母为 `manifest.json/collection_wall_seconds`，累计各次调用的实际墙钟，包含物理、渲染、验证/审计、预览及 renderer 关闭等待；重叠时间只计一次，不把共享 batch 耗时按 N 个环境重复相加。`execution_sessions` 标明串行或 pipeline 调度。共享初始化、物理执行、规划及后验检查的细分耗时见 `_vectorized/batch_NNNN/physics/batch.json`；旧 per-attempt 统计中的 `physics_s`、`init_s` 等为 0 不表示这些阶段没有成本。该正式验收路径的吞吐以真实调用结果为准，不能直接沿用性能 benchmark 的数字。

隔离 smoke 和 resume 已完成。当前已使用文首的专用续采入口接续历史数据。下面是从零采集的模板，可在 **NEW_OUTPUT** 指定的全新目录使用；它不会自动继承已有100条数据：

```bash
bash scripts/arena_python.sh scripts/run_nonhuman_augmentation.py \
  --output NEW_OUTPUT \
  --execution-backend vectorized --pipeline --num-envs 4 --validation-workers 4 \
  --device cpu --physics-threads 0 --torch-threads 1 \
  --camera-device cuda:0 --background-rgb --execution-checks deferred \
  --episodes-per-id 20 --max-attempts 100 --preview-per-id 3
```

编排器仍先用原单场景入口完成固定布局 10 场景回归，再将新后端用于随机 pilot 和续采；每场景 pilot 的合格轨迹计入 20 条目标。恢复同一新目录时保留所有参数并添加 `--resume`。N=4 是示例配置，不表示该正式十场景验收流程已经测得最佳并行数。
