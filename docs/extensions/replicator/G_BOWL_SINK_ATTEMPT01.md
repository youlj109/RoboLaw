# G bowl/sink 首次真实抓取失败与候选修正

`datasets/replicator/g_shape_bowl_sink/base/attempt_01` 是保留的真实失败，不能计入成功样本。机器人实际执行了 pregrasp、approach 和 close，未执行 lift、transport、lower 或 release。完整记录为 2457 个物理状态、307 个动作、6 个可解码视频。

## 实际失败原因

末 10 个物理子步同时有双指接触力。左指约 2.72–2.78 N，加载点约为碗局部 `[-0.061878, -0.008996, -0.004659]` 米，比本轮预声明接触区下界低 2.144 mm；右指约 0.388–0.422 N，加载点位于区内。因此，失败是左指在指定上壁/沿部以外受力，不是完全未接触。

这里的数值接触区来自本地 `bowl_rim_geometry` 的源几何抓取策略。上游 YAML 规定的是将原碗放入原水槽，没有规定这个数值 box。该区已用于本轮事前检查，保持不变；不能在看到失败后扩大区来使 attempt_01 通过。

左指在主指角约 0.530 rad 时先接触，54 个物理子步后右指才在约 0.646 rad 时加载。闭合期间 TCP 向内移动约 4.773 mm，碗反向移动约 1.523 mm，相对变化约 6.296 mm。最终六指为各自实际测量值，左内指与简单耦合值存在约 0.017 rad 差异。

实际查看了原相机帧 276、284、306。腕部图像显示一指进入碗内、另一指接近外沿，碗在这些帧中一直留在台面；没有抬升画面。小样图及观察记录：

- `reports/replicator_preparation/g_bowl_attempt01_contact_cpu01/raw_contact_frames.jpg`
- 同目录 `close_visual_observation.json`

这些图像只支持失败阶段的语义观察。毫米级区域结论来自原始接触数据，不是像素估测，也不是完整成功轨迹的视觉验收。

## 源几何与物理接触差异

独立 CPU 逐指检查实际 `robot_body_pose_w` 与六指 FK：三个检查时刻最大平移误差 0.377 μm，旋转误差小于 0.71 μrad，没有发现明显关节映射或 body frame 错误。

实际加载点距原碗三角面约左 0.408 mm、右 0.654 mm；距按实际 body 变换的源指面约左 0.456 mm、右 0.339 mm。因此，源面不相交不能覆盖或否认实际受力证据。

原碗和指碰撞均指定 `convexDecomposition`。碗源属性包含 `errorPercentage=1`、`maxConvexHulls=256`、`shrinkWrap=true`；左指 tip 具有 `shrinkWrap=false`。未发现这些 collider 上显式 authored contact/restOffset，不能据此认为运行时 offset 为零。attempt_01 没有保存 cooked hull 或运行时 offset，本报告不能把差异唯一归因于某一种 cooking/offset 机制，更没有修改源资产。

诊断：`reports/replicator_preparation/g_bowl_attempt01_contact_geometry_cpu01.json`。

## 独立验收结果

`reports/replicator_execution_20261023/g_bowl_sink/attempt_01_review_02.json` 正确拒绝整条任务。完整物理/RGB 对齐、匹配 neutral、源关系、固定台架、实际 q 自碰撞、live FCL、完整 provenance 均通过；合法双指 close、lift/release、原成功没有通过。固定台架和自碰撞 sidecar 均检查了全部 2457 个状态。

## 候选范围与尚待验证项

原区、对象位姿、布局、源 yaw、2 mm 非接触间隙和原保守球保持不变。首先保留了 0/2.5/4/6/8/10 mm 纯抬高候选。对实际全过程 near/loaded 点作刚体敏感性分析，纯 Z 可行窗口仅约 3.095–3.686 mm，不能仅凭源面相交通过就宣称物理抓取修好。

随后预声明正负 world-X 倾斜候选，并保留全部结果。负方向 −5…−30° 无法让完整闭合历史接触点保持在原区；正方向候选结果如下。每个表中候选的 46 点 approach、源面双指原区检查、闭合过程 FCL 和原夹爪球均通过。

| world-X 调整 | TCP Z 调整 | 历史接触点变换后的最小区域余量 |
| --- | --- | --- |
| 0° | 3.390 mm | 0.295 mm |
| +5° | 3.870 mm | 0.327 mm |
| +10° | 4.358 mm | 0.342 mm |
| +15° | 4.717 mm | 0.473 mm |
| +20° | 4.945 mm | 0.717 mm |
| +25° | 5.040 mm | 1.074 mm |
| +30° | 5.016 mm | 1.444 mm |

这只是将已有接触点按候选 TCP 变换的敏感性分析；改变姿态会改变接触机制，所以这些值不能预测将来的接触力、稳定性或实际余量。全部结果保存在 `g_bowl_observed_grasp_cpu02`、`g_bowl_observed_grasp_cpu03`，最初脚本接口失败也单独保留在 `g_bowl_observed_grasp_cpu01`。

主代理选定先离线核对 +25°：它是此次候选顺序中历史接触点估计余量首个超过 1 mm 的候选，并非新的安全门槛。该候选另外通过了 46 点 approach、1 个 nominal closed、240 个逐步实际六指角/物体位姿下的变换压力检查，保留所有原源 FCL 与原夹爪球门禁。报告：`reports/replicator_preparation/g_bowl_grasp_candidate25_cpu01/diagnostic.json`。

GPU 诊断由主代理排队执行，命令如下。它不启动 Kit 或物理执行；CUDA 每步显式使用六个独立指角，保留原 joint/self/world，并保存失败分项。重复碗的候选表示遮罩仍需完整源 FCL 审计，不能扩展为删除其他对象。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/diagnose_bowl_grasp_candidate.py \
  --episode datasets/replicator/g_shape_bowl_sink/base/attempt_01 \
  --search reports/replicator_preparation/g_bowl_observed_grasp_cpu03 \
  --candidate 4 \
  --output reports/replicator_preparation/g_bowl_grasp_candidate25_gpu01 \
  --gpu
```

## 固定候选接入更新

主代理已实际运行上述 GPU 诊断，`g_bowl_grasp_candidate25_gpu01/diagnostic.json` 的 46 个 approach、1 个 nominal closed、240 个实际六指状态压力样本全部通过；逐行原 joint/self/world 代价均为 0，源 FCL 和原夹爪球也全部通过。独立复核确认 240 行六指角与 attempt_01 原始记录完全一致。

随后将这一明确候选写入 schema v2 控制配置：`bowl_controller_profile.grasp_target` 每次用当前碗姿态重算原源几何抓点，再左乘固定 world-X +25° 旋转、Z 增加 0.005039543828685633 m。它不读取历史录制位姿，也不根据新增障碍改变目标。原本地接触区、源任务对象、源布局和水槽落点保持不变，未来 baseline/variants 使用同一规则。旧配置及源码快照位于 `g_bowl_grasp_runtime_promotion01/`；旧 attempt_01 不变。

CPU 已通过 28 项目标/支撑配对/下放测试和新版 `bowl_collect.py --dry-run`。旧 `diagnose_bowl_plan.py` 路径档案只接受明确提供的旧 v1 profile，避免把旧抓姿证据误标为 v2。

新实际采集仍须重新验证真实双指加载、测量 attachment、抬升、原成功和完整独立验收，不能复用假设 attachment 或将 CPU/neutral/GPU 诊断计为成功。
