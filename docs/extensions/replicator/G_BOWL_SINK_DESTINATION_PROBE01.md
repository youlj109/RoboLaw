# G bowl/sink：首次有界目标接触诊断

该次运行是诊断，`accepted=false`、`physical_trial_passed=false`、`task_success=false`。不能计作交付成功样本。正式采集器和旧失败证据未改变。

## 实际观测及独立复查

- 3649 个连续真实状态，456 个完整动作；释放从 step 3593 到 3648，共 56 子步。未补造状态。
- 全部 384 个下放子步保持原 2 mm，独立 FCL 最小间距为 **3.710993507 mm**。合法释放边界的实际落点误差为 **1.61093 mm**。
- 独立 CPU 重放全部实际 7 臂关节、6 指关节、碗和背景位姿：所有其他原碰撞对通过。目标底面原 strict 检查仍有 **51 次源网格相交**，完整记录在 ledger 中。
- 原 source/固定台架、背景/关节/静态锚点、reset 起全部子步、原相机参数与时间对齐、匹配 neutral 均经独立 CPU 检查通过。6 个视频均完整解码为 456 帧。
- 20 格原视频帧已实际查看，包含原三相机和 overview。可见碗以倾斜姿态进入水槽，开爪后倾倒并摇摆。最后 RGB 对应 step 3640，不能以图像宣称最终 step 3648 稳定。

## 原任务判定为何仍失败

末态源 footprint 和向上支撑均通过；支撑力为 **3.38910079 N**。失败项是原速度门槛：末态线速 **0.062471904 m/s**，高于原 **0.003 m/s**。

56 个释放子步中，线速范围为 **0.033968162–0.390912533 m/s**，没有一次达到原稳定阈值。碗的局部 +Z 轴倾角从释放前约 37.13°，先增到约 41.23°，再倾倒、反弹，末态约 6.01°；角速度末态仍为 1.64318 rad/s。原成功谓词逐物理步和逐完整动作复算一致，整个运行没有成功样本。

这说明有限观察窗捕获了落地和摇摆过程，尚未捕获稳定放置。不能仅延长时长或把接触力为正解释为成功。后续需要首先解释接触几何和开爪后的转动机制。

## 源网格与 native 接触不可混为一谈

原源网格最大相交深度为 **0.388393 mm**，末态约 **0.0302525 mm**。这些值只描述本次观测，不是可接受深度。

3649 个 native 原始接触样本均完整，所有实际点落在原底面区域，正加载法向位于原 +Z 支撑锥。逐步重算 `Σ(normal_force × normal)` 与原过滤传感器力，最大差仅 **2.05636×10⁻⁶ N**；保存值重算误差为 0。差值没有被用作新验收容差。

所有正加载接触点的最小 native separation 为 **+1.255423 μm**。全部点中仅一个负 separation：step 3621 的 **−27.627684 μm**，该点 normal force 为 0。原 strict 源相交与正加载 native separation 不同，不能直接称成相同物理形状的穿透。

可能需要区分的因素包括：原视觉/碰撞源三角形与 PhysX cooked 凸分解或 boundingCube 的几何差异，以及 native contact patch 的生成时刻与 post-step 物体位姿是否相同。当前报告不把其中任一个因素预设为唯一原因。

## Offset 与 shape 证据边界

实际 bowl offset getter 保存了形状为 `(1,256)` 的数组：rest 全为 0，contact 全为 0.0013625002466 m。安装 API 的契约是 `(body_count,max_shapes)`；本次没有另存逐 shape 标识、有效槽映射或 cooked 凸体清单，因此不将 256 个数组槽直接认证为 256 个实际碰撞体。原静态 BasinBottom 的 native offsets 未测，明确标为 unavailable。

Contact offset 是接触生成距离，不是允许源网格穿透深度。原台面 6 mm 接触带也不能移用到目标底面。

## Provenance 分类问题如实保留

原始开始/结束 hash 与归档文本完整；独立复核了全部 93 项预声明输入与 freeze02 一致。实际 controller 与诊断声明分别由两个确切路径绑定，内容分别与本次 result 完全一致。

但两个文件都被记录为 `controller_input` 且含相同 `task_id`。通用审查器要求唯一匹配 controller，因而报告 `Actual task controller profile input omitted/ambiguous`。该拒绝已保留，没有修改旧 raw 或给旧记录补字段。独立报告仅对本次两个固定路径及不同 schema 作精确核对；未来入口可为诊断声明使用独立角色分类。

## 下一步生产接触规则的必要依据

暂不改变生产规则。应先用安装版本的只读 cooking 表示 API 获取原 bowl 凸分解与原 BasinBottom 的表示，绑定 source SHA、composed stage、collision prim、scale、坐标变换和 cooking 参数；保存逐凸体顶点、面、plane，以及能实际证明的 shape 数量/映射。离线比较时使用每步真实位姿，分别计算原源网格与 cooked 表示的有向间距，并核对 contact patch 的时间语义。API 返回的表示未完成上述绑定前，不能叫作本次实际仿真碰撞形状。

依据齐备后，生产规则的候选结构应是：

1. `lower` 仍保持原源几何至少 2 mm；所有 robot/floor、payload/其他物体始终使用原检查。
2. 只有 `release` 后的原 bowl/原 BasinBottom 指定支撑对，才可进入显式接触判定；限定真实过滤目标、原底面区域、+Z 支撑和完整 native 记录。
3. 源网格与 cooked 表示的差异只能由已绑定几何推导解释，不能从本次最大深度加余量生成容差。任何求解器接触容差还需独立、预声明的数值或物理依据。
4. 接触通过与任务成功分开。仍须满足原位置、支撑、速度和完整任务序列，实际撤离后不得重新触碰碗；没有稳定证据就继续判失败。

缺项包括 cooked/runtime 表示绑定、静态 floor offset/有效 shape 映射、contact patch 时序核对、稳定释放后的真实基线，以及这次诊断的独立 actual-q CUDA self 审查。上述 CPU 与视频证据不能替代它们。

## 证据

- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_native_source_review.json`
- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_full_fcl.json`
- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_generic_review.json`
- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_fixed.json`
- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_raw_montage01.jpg` 及原 JPEG SHA manifest
- `reports/replicator_execution_20261023/g_bowl_sink/destination_probe01_visual_review01.json`
