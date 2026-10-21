# 厨房安全抓放数据输出标准：LeRobot Dataset v3.0

本文规定远程采集后的**目标交付格式**：11 个场景各 200 条成功轨迹，共 2200 条，统一交付一个 LeRobot Dataset v3.0 数据集。采集范围和每个场景的随机化见 [数据采集文档](data_collection.md)。本文是实施规范，不代表已完成这批数据或格式转换，也不启动本地采集。

## 1. 数据范围与计数

所有场景共用一个基础任务：机械臂抓住芥末酱瓶的瓶身，绕开安全障碍，将瓶子放入目标碗，松爪、撤离并确认落稳。场景分为 6 个财产安全、4 个机械臂自身安全、1 个人身安全。

| scene_index | obstacle_id | 场景 | 成功 episode 目标 |
| --- | --- | --- | --- |
| 0 | `property_tall_glass` | 高水杯 | 200 |
| 1 | `property_wine_glass` | 红酒杯 | 200 |
| 2 | `property_glass_bottle` | 玻璃瓶 | 200 |
| 3 | `property_ceramic_mug` | 陶瓷杯 | 200 |
| 4 | `property_vase` | 花瓶 | 200 |
| 5 | `property_tablet_stand` | 平板支架 | 200 |
| 6 | `robot_steel_clamp` | 钢夹 | 200 |
| 7 | `robot_projecting_crossbar` | 横杆 | 200 |
| 8 | `robot_narrow_opening` | 窄门／窄开口 | 200 |
| 9 | `robot_sharp_equipment_housing` | 尖角设备 | 200 |
| 10 | `human_default_mpfb` | 原默认 MPFB 人体伸手 | 200 |
| 合计 | 11 个场景 | 同一个基础抓放任务 | **2200** |

人体只包含原默认 MPFB 模型和伸手姿态，手臂伸入瓶子到碗的操作区域；其他人体模型、体型或姿态变体不在本次范围内。

一条完整物理抓放轨迹计为一个 episode，包含三个相机的视频。三视角不计为三条轨迹，同一物理轨迹的重复渲染也不计为新增轨迹。完整数据有 2200 个 episode、6600 段相机观测；由于 v3 支持多个 episode 共用视频文件，实际 MP4 文件数不一定为 6600。

- `episode_index`：整个数据集唯一、从 0 连续编号；完整交付为 0–2199。失败尝试不占用成功 episode 编号。
- `scene_index`：本项目增加的场景编号，按上表解释，不能从 episode 编号区间推断场景。
- `task_index`：LeRobot 的自然语言任务编号。本批只有一个基础任务，统一为 0；11 个场景不等于 11 种操作技能。
- 统一任务文本：`Grasp the mustard bottle by its body, place it into the bowl while avoiding the safety obstacle, then release and retreat.`
- 原始 `attempt` 和随机种子保存在附加记录中，不能用它们替代全局 `episode_index`。

## 2. LeRobot v3 目录与元数据

目标格式版本为 `v3.0`。下面的路径和写入接口以官方 LeRobot **v0.4.3** 实现为参考；库版本号与数据格式版本号是两件事。该实现将表格和视频按文件分片，并在元数据中保存 episode 的位置和视频时间偏移。[官方格式说明](https://huggingface.co/docs/lerobot/lerobot-dataset-v3)、[v0.4.3 路径定义](https://github.com/huggingface/lerobot/blob/v0.4.3/src/lerobot/datasets/utils.py)

```text
kitchen_safety_11_lerobot_v3/
  meta/
    info.json
    stats.json
    tasks.parquet
    episodes/
      chunk-000/
        file-000.parquet
        ...
  data/
    chunk-000/
      file-000.parquet
      ...
  videos/
    observation.images.external_camera/
      chunk-000/file-000.mp4
      ...
    observation.images.external_camera_2/
      chunk-000/file-000.mp4
      ...
    observation.images.wrist_camera/
      chunk-000/file-000.mp4
      ...
  extras/
    scene_catalog.json
    field_conventions.json
    cameras.json
    episodes.parquet
    collection_summary.csv
    episodes/
      episode-000000/
        randomization.json
        validation.json
        grasp_contacts.json
        clearances.json
        safety_steps.parquet
        replay_states.npz
      ...
```

`meta/`、`data/`、`videos/` 使用 LeRobot 原生组织。`extras/` 是本项目的附加信息，不是 LeRobot 标准必需目录；普通 LeRobot 读取器应能在不读取 extras 的情况下加载图像、动作和状态。

| 文件 | 要求 |
| --- | --- |
| `meta/info.json` | `codebase_version` 为 `v3.0`，`fps` 为 15；保存真实 episode／帧数、机器人标识、所有 feature 的 dtype／shape／维度名称、数据与视频路径模板。`total_episodes` 只有完整交付时才为 2200，不写入尚未完成的目标数。 |
| `meta/tasks.parquet` | 保存任务文本和 `task_index`；本批使用上文同一任务文本。不要用旧格式的 `tasks.jsonl` 替代。 |
| `meta/episodes/chunk-*/file-*.parquet` | 使用原生 writer 生成 episode 长度、任务、表格位置、各视频文件位置与起止时间等元数据。不要用旧格式的 `episodes.jsonl` 替代。 |
| `meta/stats.json` | 由实际交付的数据计算统计信息；不能拷贝其他批次的统计值。 |
| `extras/scene_catalog.json` | 保存上表的 `scene_index`、`obstacle_id`、中文名称、类别和每场景目标 200。 |
| `extras/episodes.parquet` | 每个 episode 一行，保存场景、源批次／attempt、三个种子、成功与安全结果、随机参数和附加文件路径的对应关系。 |
| `extras/collection_summary.csv` | 逐场景保存目标数、实际成功数、尝试数、各类失败数和缺口；总数达标不能抵扣某个场景缺口。 |

分片大小交给原生 writer 管理，不要求一个 episode 对应一个 Parquet 或 MP4。每个相机分别通过原生 episode 元数据定位其共享视频文件和时间偏移，不能假设三个视角或相邻 episode 使用相同文件编号。[官方 writer 与视频索引实现](https://github.com/huggingface/lerobot/blob/v0.4.3/src/lerobot/datasets/lerobot_dataset.py)

## 3. 逐帧训练数据

每一行表示一个动作时刻。以下 shape 是**单帧** shape，不包含整条轨迹长度 T；标量索引和时间在 LeRobot feature 中声明为 `(1,)`。表格数据与视频通过原生 episode／帧索引关联。

### 3.1 基础字段

| feature | dtype / 单帧 shape | 语义 |
| --- | --- | --- |
| `observation.state` | float32 `(13,)` | 动作前的 13 维实测关节位置，单位 rad，顺序见下文。 |
| `action` | float32 `(8,)` | 7 维绝对机械臂关节目标，单位 rad；最后 1 维为二值夹爪目标。 |
| `observation.images.external_camera` | video `(720,1280,3)` | 原第一外相机 RGB。 |
| `observation.images.external_camera_2` | video `(720,1280,3)` | 原第二外相机 RGB。 |
| `observation.images.wrist_camera` | video `(720,1280,3)` | 原腕相机 RGB。 |
| `timestamp` | float32 `(1,)` | episode 内相对时间，秒，`frame_index / 15`。 |
| `frame_index` | int64 `(1,)` | episode 内从 0 连续递增的帧索引。 |
| `episode_index` | int64 `(1,)` | 全数据集 episode 编号。 |
| `index` | int64 `(1,)` | 全数据集连续行索引，由原生 writer 生成。 |
| `task_index` | int64 `(1,)` | 自然语言任务索引，本批为 0。 |
| `scene_index` | int64 `(1,)` | 本项目扩展，0–10 的场景索引。 |
| `phase_index` | int64 `(1,)` | 本项目扩展，当前执行阶段。 |
| `next_observation.state` | float32 `(13,)` | 本项目扩展，执行本行动作后的关节位置，包括末行动作的最终状态。 |

13 维关节状态的顺序固定为：

```text
panda_joint1, panda_joint2, panda_joint3, panda_joint4,
panda_joint5, panda_joint6, panda_joint7,
finger_joint, right_outer_knuckle_joint,
left_inner_finger_joint, right_inner_finger_joint,
left_inner_finger_knuckle_joint, right_inner_finger_knuckle_joint
```

`action[0:7]` 按 `panda_joint1` 至 `panda_joint7` 排列，是**绝对关节目标**，不是增量、速度、力矩或 TCP 位姿。`action[7]` 只能是 `0`（打开）或 `1`（闭合），不是开口宽度。观测 13 维、动作 8 维是有意保留的不同语义。

`phase_index` 映射写入 `extras/field_conventions.json`：0 settle、1 pregrasp、2 approach、3 close、4 lift、5 transfer_front、6 transfer、7 above_bowl、8 lower、9 release、10 retreat。阶段可缺省或重复，例如末尾再次 settle；按逐帧记录读取，不假设每段帧数固定。

### 3.2 状态、接触力与相机扩展字段

为了保留原采集结果，下列项目字段也声明为 LeRobot features。除相机位姿外，同时保存 `observation.*` 和对应 `next_observation.*`；以下以动作前字段举例。静态障碍也要保存真实位姿，不能因没有动态传感器字段就填零。

| feature 后缀 | dtype / 单帧 shape | 内容 |
| --- | --- | --- |
| `joint_velocity` | float32 `(13,)` | 与关节位置同序的关节速度，rad/s。 |
| `tcp_pose` | float64 `(7,)` | TCP 的 `[x,y,z,qx,qy,qz,qw]`，位置米。保留原 TCP 位置精度。 |
| `mustard`、`bowl` | float32 `(13,)` | 瓶和碗根状态：位置、四元数、线速度、角速度。 |
| `robot_body_pose` | float32 `(126,)` | 18 个机器人刚体的 7 维位姿，按刚体优先展平。 |
| `robot_contact_forces` | float32 `(54,)` | 18 个接触刚体的 3 维净接触力，按刚体优先展平，单位 N。 |
| `hazard_pose` | float32 `(7,)` | 障碍物根位姿；动态障碍取实际状态，静态人体／机械障碍取实际应用的场景根变换。 |
| `camera_pose.external_camera` | float64 `(7,)` | 第一外相机逐帧位置和 ROS 相机四元数。 |
| `camera_pose.external_camera_2` | float64 `(7,)` | 第二外相机逐帧位置和 ROS 相机四元数。 |
| `camera_pose.wrist_camera` | float64 `(7,)` | 腕相机逐帧位置和 ROS 相机四元数。 |

例如完整名称是 `observation.tcp_pose`、`next_observation.tcp_pose`、`observation.camera_pose.wrist_camera`。相机只要求保存与 RGB 同步的动作前位姿，不要求补造动作后的图像或相机记录。

刚体根状态的 13 维顺序为 `[x,y,z,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz]`。机器人位姿的刚体顺序与接触力的刚体顺序可能不同，应分别保存 `robot_body_names` 和 `contact_body_names`；不能因当前二者都是 18 个就混用索引。

这些扩展是仿真状态记录，不自动成为策略输入。训练配置可以只选择三路 RGB 与 `observation.state`，但导出不能默默丢掉已采集的其他状态。厨房刚体／关节／连杆状态、机器人刚体速度等回放信息存入每个 episode 的 `extras/.../replay_states.npz`，保留动作前后两份数组、原字段名和名称映射，第一维与 T 一致；其坐标约定同第 5 节。

## 4. 三视角视频与时间对齐

三个相机的安装关系、内参和分辨率固定：两个外相机和一个腕相机，均为 1280×720 RGB。腕相机相对夹爪固定，但在厨房坐标系中的位置随动作改变；必须保存逐帧位姿。

- 动作和 RGB 频率：15 Hz；物理步进：120 Hz，每个动作对应 8 个物理步。
- 每个 episode 有 T 行动作，每个视角恰好 T 帧。`RGB[t]` 对应执行 `action[t]` **之前**的 `obs[t]`。
- `next_observation[t]` 对应动作执行后状态；除末帧外，应与 `observation[t+1]` 连续。末帧的 next state 需要保留，但不额外插入一帧 RGB。
- 时间从 0 开始，最后一行时间为 `(T-1)/15`，该段视频时长约为 `T/15`。共享 MP4 的文件时间偏移只写在原生视频元数据中，不加到逐帧 `timestamp` 上。
- 使用原三路图像分别编码，不缩放，不叠加文字，不使用拼接预览替代训练视频；默认 MP4/H.264、15 fps，编码配置记录在交付元数据中。
- 编码前图像为 HWC、uint8 RGB；LeRobot 读取器返回的图像通常为 CHW、float32、范围 `[0,1]`，不要将其误认为磁盘视频布局。

关系如下：

```text
observation[t] + 三相机 RGB[t]
                ↓ action[t]：7 个绝对关节目标 + 二值夹爪目标
next_observation[t] ≈ observation[t+1]
```

在当前源数据上检查状态链时，使用绝对误差 `1e-6`、相对误差 0。导出后仍检查关节、TCP、物体和已保存扩展状态的连续性。视频压缩可能改变像素值，因此视频验收侧重帧对应、分辨率、可解码性和内容完整性，不宣称与 JPEG 字节或像素完全相同。

`extras/cameras.json` 保存每路相机的内参矩阵、图像尺寸、安装父节点、安装偏移和轴约定；逐帧位姿存入第 3 节的 feature，不能只保留一次静态外参来代替腕相机轨迹。

## 5. 坐标、单位与接触记录

统一使用厨房环境局部坐标 `environment_local`，轴方向与原厨房世界坐标一致。位置米，关节角弧度，线速度 m/s，角速度 rad/s，力 N；所有导出四元数顺序为 **xyzw**。

并行采集产生的环境平移必须正确消除：已经转换为环境局部坐标的轨迹和相机记录直接使用，不能再次减去环境原点。仍位于原物理大场景的接触点，在导出时仅减一次该物理环境的原点；瓶子局部接触点保持瓶子局部坐标。原环境原点保留在附加元数据中以便追溯。

相机位姿表示 `T_environment_camera`，即将相机坐标转换到厨房环境坐标；光学轴为 ROS 约定：x 向右、y 向下、z 向前。原安装配置可能使用 OpenGL 轴约定，应在 `cameras.json` 中明确区分，不能把安装四元数原样当作 ROS 光学姿态。

`grasp_contacts.json` 保留稀疏的瓶身抓持接触记录，包括 `frame_index`／原 `action_index`、手指名称、接触点、法向力和分离距离等原来实际记录的字段。源记录的 `normal_forces_n` 是法向力，不包含接触法向量。稀疏接触记录条数不要求等于 T，也不能用它代表所有物体之间的完整接触信息。它与每帧 `robot_contact_forces` 的净接触力是两类数据。

逐物理步安全记录放在 `safety_steps.parquet`，至少包含 episode、动作帧索引、物理子步或检查序号、阶段，以及实际记录的状态、距离和检查结果。保存检查覆盖区间及被跳过的初始化区间；原实现的几何检查从初始 30 个 settle 动作后开始，不能把没有检查的步骤写成“检查通过”。

## 6. 随机化和结果追溯

每个 episode 的 `randomization.json` 保存：

- 场景 ID、场景类别、主种子、原 attempt 编号，以及独立的 geometry／appearance／planner 种子。
- 使用的普通配置内容、几何范围缩放系数、瓶／碗／障碍采样位姿及静置后的实际位姿。
- 灯光、颜色、粗糙度采样值；实际应用结果和未修改表面的原因。不能把采样参数当作所有材质都已修改的证据。
- 人体模型和姿态标识、实际根变换、局部和变换后的关键点；默认人体是静态障碍，不补造逐帧人体关节运动。
- 规划与执行设置、来源记录路径；路径在最终交付中使用相对路径，不依赖原机器的 USB 挂载点。

`validation.json` 保存完整任务结果、物理与安全检查、风险作用判定、图像／状态对齐和格式验证结果。钢夹采用已确认的侧向自身安全场景规则，不额外要求其阻断起终点直连；所有场景仍保留既有物理和安全要求。失败尝试另行保留诊断和汇总，不混入这 2200 条成功 episode。

## 7. 从现有采集结果导出

Collector 原生输出 HDF5 等中间文件；现已提供 [LeRobot v3 导出入口](../scripts/export_safety_lerobot_v3.py) 和 [独立 CPU 环境安装脚本](../scripts/setup_lerobot_export.sh)。入口读取成功清单，或读取 collector `manifest.json` 中各 ID 的 `accepted_paths`，实际数量写入原生元数据。单独的默认人体成功清单会保留其原始 benchmark 资格标记，不能据“导出成功”把未确认挡路作用的轨迹改标为正式 2200 条配额合格数据。

| 当前中间数据 | 目标 |
| --- | --- |
| `trajectory.h5/action` | `action` |
| `trajectory.h5/obs/q`、`next_obs/q` | `observation.state`、`next_observation.state` |
| `obs`／`next_obs` 其他状态 | 第 3 节对应 feature 及 replay 附加记录 |
| `phase` | 按映射写入 `phase_index` |
| `images.h5` 或 `rendered/images.h5` 的三路 JPEG | 解码原帧，经原生视频 writer 写入三路 `observation.images.*` |
| 相机 H5 位姿及 `cameras.json` | 逐帧 camera_pose features 和 `extras/cameras.json` |
| `episode_spec.json`、`appearance.json`、实际静置结果 | `randomization.json` |
| `verification.json`、`validation.json`、`audit.json`、任务结果 | 汇总到附加验证记录，保留各项检查含义 |
| `grasp_contacts.json`、`clearances.json`、`physics_states.npz` | 规范化坐标和索引后的接触／距离／逐物理步记录 |

实现使用官方 `LeRobotDataset.create`、批量 `save_episode(episode_data=...)` 和 `finalize`，由原生 writer 维护索引、分片、统计和视频偏移。视频输入优先复用已完整导出的三个无叠字原视角 MP4；只有 RGB 时，从原始 JPEG 编码三个训练视频。只替换临时编码步骤，不手填原生 LeRobot 元数据，也不把拼接预览用于训练。[v0.4.3 原生写入接口](https://github.com/huggingface/lerobot/blob/v0.4.3/src/lerobot/datasets/lerobot_dataset.py)

远端在采集结束后运行，路径替换为实际数据位置：

```bash
bash scripts/setup_lerobot_export.sh
env -u PYTHONPATH HF_HOME=/data/huggingface_cache \
  .venv-lerobot-export/bin/python scripts/export_safety_lerobot_v3.py \
  --selection /data/episodes/manifest.json \
  --output /data/kitchen_safety_11_lerobot_v3
```

安装脚本优先使用 Arena 的 Python，推荐 Python 3.12；CPU PyTorch 与采集环境分开。若需指定 PyPI 镜像，可设置 `LEROBOT_PYPI_INDEX`。导出目标目录应是新目录；先用 `--limit 2` 和独立测试目录检查本机环境，再导出全量数据。默认 MPFB 的 20 条真实成功轨迹已完成原生读回、全部状态／相机字段逐帧比对、跨 episode 视频偏移及所有视频完整解码；另有原始 JPEG 编码路径的实际验证。远端其他场景仍须完成自身采集与验收。

`export_report.json` 记录本次格式导出的实际数量、是否为子集和源采集是否达标；`validation_report.json` 记录逐条字段比对、视频起中末帧对应和所有视频文件完整解码。格式转换完成不代表 11 个场景各 200 条配额已经达到。`extras/collection_summary.csv` 同时列出源采集数量和本次导出数量，源报告中的原机器路径仅作溯源，原生读取不依赖这些路径。

## 8. 交付检查

1. 使用选定版本的原生 LeRobot 读取器成功打开数据集，能读取三视角、action、state；记录使用的库版本和视频后端。
2. 按 `scene_index` 和 episode 元数据核对 11 个场景各 200 条，共 2200 条；每条源轨迹只映射一次。未满时写真实数量与逐场景缺口。
3. 所有 episode 的三路视频均可完整解码，帧数均为 T，分辨率 1280×720；共享文件中的 episode 边界和起止帧定位正确。
4. 动作、观测、next state、阶段、时间、相机位姿逐帧对齐；关节名称、单位、四元数及环境局部坐标约定一致。
5. 元数据实际帧数、索引、统计与文件内容一致；writer 已正常结束并关闭文件，附加记录可通过 episode_index 对应回核心数据。
6. 每条成功数据都能追溯到源采样、完整执行和既有安全验收；失败尝试、预览和重复渲染不计入配额。
7. 跨机器交付包含所需真实文件，不能仅交付指向原采集机器的软链接。

最终交付物是上述 LeRobot v3 数据集、逐场景汇总和附加记录。预览视频可以单独提供，不影响训练数据的三路原视角组织。
