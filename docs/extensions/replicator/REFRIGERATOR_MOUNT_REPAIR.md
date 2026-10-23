# G 型厨房背景冰箱的安装约束修复

此修复仅对应当前 Replicator G 型 mustard→bowl 场景。完整数值、原始报告路径和哈希见 [refrigerator_mount_repair.json](refrigerator_mount_repair.json)。背景物理修复和可动性已通过：`neutral_mount_03` 的 40 秒中性以及两门真实外力可动性均通过独立复核。第一轮 `neutral_mount_02` 的拒绝原因和下一轮预先声明的初始化协议完整保留。新任务数据仍须另外重采与验收。

原场景中，未接触的冰箱右门在 16 秒对照里转动约 110°。它不是 articulation，原 USD 中两个转动关节及两个内部固定关节均保留，104 个背景刚体的抽取没有丢失这些约束。源关节轴接近竖直，两个门的两端锚点差约 4 μm；初始化后冰箱壳体在地面碰撞支撑上倾斜约 0.398°，门的转轴随之倾斜。

使用实际质量、质心、惯量和运行时转轴计算，在第 2 秒右门及其固定面板产生约 −0.1905 N·m 的重力矩，轴向惯量约 2.6157 kg·m²，预测角加速度 −4.172°/s²，与记录约 −3.9°/s² 相符。源面板质量 22 kg、内部固定件 110 kg 都保留；本修复不声称这些值经过实物标定。惯量均正定，完整特征值和条件数在 JSON 中。启用原生 contact report 后，五个冰箱刚体之间没有测到内部接触负载，因此没有添加排除碰撞对。

4 秒隔离实验（以每个实验 reset 后首帧为参考，采样到 3.933 秒）：

| 条件 | 右门最大角变化 | 说明 |
| --- | ---: | --- |
| 原始源资产 | 28.961° | 自主开门复现 |
| 仅冰箱五个刚体关闭重力 | 0.000318° | 诊断用，未采用 |
| 位置/速度 solver iterations 提升至 32/8 | 28.973° | 无改善，未采用 |
| 仅安装固定壳体 | 0.000157° | 保留重力及门的自由度 |

实现 [background_physics.py](../../../kitchen_safety/extensions/replicator/background_physics.py) 生成一个可追溯的 USD sublayer，仅新增 `/World/Fridge/RoboLawHousingInstallation` 固定关节，把冰箱壳体固定到原始 authored 世界姿态。这是本扩展采用的“厨房冰箱已经固定安装”假设，上游并未提供这个安装约束。旧 reset 已包含初始微小倾斜，因此安装锚点使用源 authored 姿态。逐项比较 composed USD，所有原 prim、属性和 metadata 均未改变；只新增这一关节。源碰撞、质量、门的 ±110° 行程、门的原驱动设置、厨房其他物体及机器人/瓶/碗关系保持一致。运行期间没有位置、速度或关节状态回写。

可动性实验先中性保持 2 秒，再向左/右门分别施加 +0.3/−0.3 N·m 的世界 Z 轴真实力矩 2 秒，然后撤去力矩观察 2 秒。720 个物理子步都记录了 wrench 应用，门在脉冲末分别打开 16.824°/12.779°，壳体全程零位移和转动。初始中性阶段两门变化均低于 0.001°。这是外力诊断，不是机器人开门成功样本；自由门在外力撤去后继续滑行符合未添加阻尼的模型。原三路视频及补充冰箱视角全部 90 帧解码通过，并目视核对初始静止、脉冲后逐渐开门。

复现可动性：

```bash
# 在 arena-kitchen-dev 容器 /workspace 下运行；每次使用新的 output。
flock /workspace/reports/skill_trials/gpu.lock env PYTHONPATH=/workspace HEADLESS=1 \
  /workspace/.venv-arena/bin/python scripts/extensions/replicator/probe_refrigerator_mount.py \
  --device cpu --layout configs/extensions/replicator/avoidance/human_repaired03.json \
  --output reports/replicator_physics/mobility_new
```

长中性使用 `scripts/extensions/replicator/neutral_scene_probe.py --duration-seconds 40`。预先门槛为所有 104 个背景体在 2 秒沉降后不超过 2 mm/1°，所有有关节连接的动态刚体还必须从 reset 首帧起不超过 2 mm/1°；瓶/碗不超过 1 mm/1°，机器人关节不超过 0.001 rad。静态柜架锚点须明确标记为静态端点，动态关联体漏采则拒绝。初次 `neutral_mount_01` 在采集前将合法静态端点错误拒绝，日志保留；该失败不能作为物理通过或不通过的证据。

原始资产和失败尝试保留。新数据的 result/audit 会比较源、recipe 与 overlay 哈希；修改后的变体必须匹配同一背景物理版本的新基线。该修复仅检查当前单环境场景，未验证其他厨房或多环境并行复制。

第一轮 40 秒结果：右门最大变化 0.0000436°，左门 0.0001889°，壳体无位移/转动，所有已观测关节体满足原门槛。整场景仍失败：(1) Range 源炉体由 PhysX articulation root 路径报告，需要可追溯的动态 alias 解析，不能伪称静态；(2) 自由梨在 2 秒后还有 0.346 mm/4.141° 的初始摇摆。梨最后一次非零速度在第 3.000 秒，3.067 秒到 39.933 秒原生 pose 逐 bit 相同、线/角速度均为零。这是有限初始沉降，而非持续无接触机构开门。原失败 JSON、源脚本快照、原始速度与姿态完整保留。

下一轮在运行前明确采用：无关自由背景物体先自然沉降 **5 秒**，随后观察 **35 秒**，仍使用 **2 mm/1°** 门槛；所有关节关联体仍从 reset 首帧使用原门槛。新两类无障碍基线和三个变体也统一记录 **5 秒无动作等待**，其间所有视频和状态保留。没有改变梨的质量、惯量、碰撞、摩擦或状态，没有改变机器人实际执行的安全距离。这一协议改变的是自由物体的初始化观察窗，不能把失败的旧结果改写为通过。

最终 `neutral_mount_03` 按上述预先声明的协议复跑，通过独立数组复算：104 个背景刚体在第 5–40 秒全部零位移、零角变化；89 个关节关联刚体从 reset 起最大 1.22344 mm/0.322587°，低于原 2 mm/1° 门槛。66 个静态锚点无动画、原源/运行时矩阵一致且运行中不漂移。Range 炉体的唯一关节图根对应 native articulation root，源质量 126 kg 与原生 125.999992 kg 一致，初始位姿误差 4.055 μm/0.00001149°，因此没有遗漏动态关联体。瓶、碗和机器人中性保持全部通过。相机 capture 不推进物理，每帧间隔确为 8 个 120 Hz 物理步；原三路、拼接以及额外冰箱视角各 600 帧、15 fps 全部解码，并目视检查 0、2、5、10、20、39.933 秒冰箱保持静止。

最终复核文件：`reports/replicator_physics/neutral_mount_03/independent_review.json`；实际开门可动性：`reports/replicator_physics/mobility_mount_01/mobility_probe.json`。这些结论针对背景质量和门自由度，不代替新采集任务成功、连续安全距离和配对避障必要性的验证。
