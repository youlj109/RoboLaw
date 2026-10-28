# Replicator 154 景：成功、失败与执行说明

**更新日期：2026-10-28；分支：`ylj`。首轮实际执行 154 景，✅ 成功 70 景，❌ 失败 84 景，执行错误 0 景。**

范围为 14 个 Replicator 原始任务 × 11 种变体。154 景均已生成规划轨迹；本页的成功/失败专指真实 PhysX 抓放结果。每景首轮一次，失败保留，不通过选取补录结果提高成绩。

[机器可读的逐景状态及来源 SHA](../../../configs/extensions/replicator/physical_execution_154_20261028.json) · [视频包与资产恢复说明](RELEASE_154_PLANNED_SCENES.md) · [本地网页使用说明](PROGRESS_DASHBOARD.md)

## 状态怎么理解

- **✅ 成功**：本次执行曾满足原任务成功条件，且最后仍满足原目标放置判据；不表示已经通过安全避障或长期稳定性验收。
- **❌ 失败**：本次真实执行没有同时满足上述条件。失败现象按 F1/F2/F3 标注，不代表已证明场景无解。
- **相机补录**：仅 U 型香蕉任务的 11 景因首轮画面遮挡，额外各真实执行一次；补录 7 成功、4 失败，逐景结果恰与首轮一致。主视频固定选用全部 11 个补录，不按成绩择优，首轮证据仍保留。
- 既有 `final_154_scenes_20261028.json` 是冻结的**规划来源索引**，其物理“未验证”字段描述发布时的历史状态；当前物理结果以本页及上方独立状态 JSON 为准。

## 14 × 11 结果总表

点击任务名称查看下方 11 个变体详情。列名对应同名安全增广物体；完整变体 ID 和配置链接在详情中。

| 原始任务 | 人体 | 高水杯 | 高脚酒杯 | 玻璃瓶 | 陶瓷杯 | 花瓶 | 平板支架 | 钢夹 | 突出横杆 | 窄口 | 尖角外壳 | 成功/11 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | ---: |
| [G 型：香蕉放碗](#g_shape_banana_bowl) | ❌ F1 | ✅ | ❌ F2 | ❌ F2 | ❌ F1 | ❌ F2 | ❌ F1 | ❌ F1 | ❌ F3 | ✅ | ✅ | 3/11 |
| [G 型：碗放水槽](#g_shape_bowl_sink) | ❌ F1 | ❌ F1 | ✅ | ❌ F1 | ✅ | ✅ | ❌ F1 | ✅ | ❌ F1 | ❌ F1 | ❌ F1 | 4/11 |
| [G 型：芥末瓶放碗](#g_shape_mustard_bowl) | ❌ F2 | ✅ | ❌ F2 | ❌ F1 | ❌ F2 | ❌ F2 | ❌ F2 | ❌ F1 | ❌ F2 | ❌ F3 | ❌ F1 | 1/11 |
| [L 型带中岛：香蕉放碗](#l_island_banana_bowl) | ✅ | ✅ | ✅ | ✅ | ❌ F1 | ❌ F2 | ✅ | ❌ F3 | ❌ F2 | ✅ | ❌ F1 | 6/11 |
| [L 型带中岛：碗放水槽](#l_island_bowl_sink) | ❌ F1 | ❌ F1 | ❌ F1 | ✅ | ✅ | ✅ | ❌ F1 | ❌ F1 | ❌ F1 | ❌ F2 | ✅ | 4/11 |
| [L 型带中岛：芥末瓶放碗](#l_island_mustard_bowl) | ❌ F2 | ✅ | ✅ | ✅ | ❌ F3 | ✅ | ✅ | ❌ F1 | ✅ | ✅ | ✅ | 8/11 |
| [L 型：香蕉放碗](#l_shape_banana_bowl) | ❌ F2 | ✅ | ❌ F3 | ❌ F3 | ✅ | ✅ | ❌ F1 | ✅ | ❌ F1 | ✅ | ✅ | 6/11 |
| [L 型：碗放水槽](#l_shape_bowl_sink) | ❌ F1 | ❌ F1 | ❌ F2 | ❌ F1 | ✅ | ❌ F1 | ❌ F1 | ❌ F1 | ❌ F1 | ❌ F1 | ✅ | 2/11 |
| [L 型：芥末瓶放碗](#l_shape_mustard_bowl) | ❌ F2 | ❌ F1 | ❌ F2 | ❌ F2 | ❌ F1 | ❌ F2 | ❌ F2 | ✅ | ❌ F2 | ✅ | ❌ F2 | 2/11 |
| [半岛式：碗放水槽](#peninsula_bowl_sink) | ❌ F1 | ✅ | ❌ F1 | ✅ | ✅ | ✅ | ❌ F1 | ✅ | ❌ F1 | ✅ | ✅ | 7/11 |
| [半岛式：芥末瓶放碗](#peninsula_mustard_bowl) | ✅ | ❌ F1 | ✅ | ❌ F1 | ❌ F1 | ✅ | ✅ | ❌ F1 | ❌ F1 | ✅ | ❌ F1 | 5/11 |
| [U 型：香蕉放碗](#u_shape_banana_bowl) | ❌ F3 | ✅ | ✅ | ❌ F3 | ✅ | ❌ F2 | ✅ | ❌ F1 | ✅ | ✅ | ✅ | 7/11 |
| [U 型：碗放水槽](#u_shape_bowl_sink) | ❌ F1 | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ F1 | ❌ F2 | ❌ F2 | ✅ | ✅ | 7/11 |
| [U 型：芥末瓶放碗](#u_shape_mustard_bowl) | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ✅ | ❌ F1 | ❌ F1 | ✅ | ❌ F1 | 8/11 |

## 失败标签与排查方向

**下面是可复核的记录特征，不是确定的失败原因。原成功判据没有改为这些诊断阈值。**

| 标签 | 数量 | 记录特征 | 下一步优先检查 |
| --- | ---: | --- | --- |
| F1 | 51 | 搬运阶段未记录到连续至少 3 个动作样本抬高超过 4 cm | 抓取点、抓取姿态、实际接触及跟踪 |
| F2 | 25 | 记录到上述抬升，同时携物阶段物体与夹爪距离增大超过 10 cm | 夹持保持、携物姿态、速度与接触变化 |
| F3 | 8 | 记录到上述抬升，未命中 F2，最终放置条件仍失败 | 落点、支撑、释放、撤离及稳定时间 |

分类使用 30 Hz 动作采样，搬运阶段为 `lift/transfer/lower`。抬升基线取最后 30 个 `initial_settle` 样本的物体 spawned-bounds 中心 Z 中位数。分离指标是物体原点到实际 Robotiq `base_link` 原点距离，减去最后 5 个 `close` 样本距离中位数；取搬运阶段的最大增量。先判断 F1，再判断 F2，否则为 F3。

4 cm 不是原任务成功阈值，F1 不等于“从未抓住”；距离增大也不能直接证明滑落或碰撞，F2 不证明严格的事件先后或因果。详情中的目标范围、向上支撑、低速为原放置判据的分项，可同时不满足。完整指标、阈值及原数组 SHA 保存在状态 JSON 的 `failure_observation` 中。

### 已发现、尚未修复的执行问题

快速规划入口传入 `max_joint_speed=.4`，但 `QuickPlanner.quick_plan` 直接返回 cuRobo 插值轨迹，没有调用父类对应的减速处理。已检查 154 条 NPZ：同一阶段内目标关节速度峰值均超过 0.4 rad/s，约为 0.826–7.126 rad/s；这是目标指令变化速度，不是实测关节速度，也不证明全部失败由速度造成。

相关实现：[规划入口](../../../scripts/extensions/replicator/quick_trajectory_generate.py)、[快速规划器](../../../kitchen_safety/extensions/replicator/quick_trajectory.py)、[减速处理](../../../kitchen_safety/curobo_backend.py)、[真实执行器](../../../kitchen_safety/extensions/replicator/planned_execution.py)。本次发布只增加结果标注，尚未修复该问题或重新规划/执行。

建议先保持场景布局，修正执行时序并验证实际抓持，再尝试不同抓点、抓姿和放置方案。14 个原始任务都有成功变体，但不能据此证明每个障碍变体都可行；有限次失败也不足以证明无解。

## 逐任务、逐变体详情

每行场景 ID 唯一。变体名称链接到该次实际使用的配置。首轮视频与补录的 SHA、实际结果文件位置见状态 JSON；视频包内主文件均为 `videos/<scene_id>.mp4`，对应结果为 `selected_results/<scene_id>.json`，首轮结果另存 `first_pass_results/`。

## g_shape_banana_bowl

G 型：香蕉放碗；首轮 **3 成功 / 8 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_g_shape_banana_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `g_shape_banana_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/scene_retry_once_20261028/g_shape_banana_bowl/human_default_mpfb.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 **场景质量提示：一次CPU检查仍发现人体衣服与厨房橱柜网格相交；未再次改位，场景质量不通过。** | — |
| `g_shape_banana_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_banana_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/property_wine_glass.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/property_glass_bottle.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/property_ceramic_mug.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/property_vase.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/g_shape_banana_bowl/property_tablet_stand/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/g_shape_banana_bowl/robot_steel_clamp/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/robot_projecting_crossbar.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_banana_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/robot_narrow_opening.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_banana_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_banana_bowl/robot_sharp_equipment_housing.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## g_shape_bowl_sink

G 型：碗放水槽；首轮 **4 成功 / 7 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_g_shape_bowl_sink`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `g_shape_bowl_sink__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_only_review_v1/g_shape_bowl_sink/human_default_mpfb/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_tall_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_wine_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_bowl_sink__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_glass_bottle.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_bowl_sink__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_bowl_sink__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/property_tablet_stand.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/robot_steel_clamp.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_bowl_sink__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/g_shape_bowl_sink/robot_narrow_opening/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_bowl_sink__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_bowl_sink/robot_sharp_equipment_housing.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |

## g_shape_mustard_bowl

G 型：芥末瓶放碗；首轮 **1 成功 / 10 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_g_shape_mustard_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `g_shape_mustard_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/human_default_mpfb.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `g_shape_mustard_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_wine_glass.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_glass_bottle.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_ceramic_mug.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_vase.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/property_tablet_stand.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/robot_steel_clamp.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/robot_projecting_crossbar.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `g_shape_mustard_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/robot_narrow_opening.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：目标范围。 | — |
| `g_shape_mustard_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_scene_variants_final03/g_shape_mustard_bowl/robot_sharp_equipment_housing.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |

## l_island_banana_bowl

L 型带中岛：香蕉放碗；首轮 **6 成功 / 5 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_island_banana_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_island_banana_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/human_default_mpfb/attempt03/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 曾有一次执行前初始化停滞，恢复后才开始本次首轮物理执行。 | — |
| `l_island_banana_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_tall_glass/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_banana_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_wine_glass/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_banana_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_glass_bottle/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_banana_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_ceramic_mug/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_banana_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_vase/attempt01/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_banana_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/property_tablet_stand/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_banana_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_banana_bowl/robot_steel_clamp.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：目标范围。过程中曾满足原任务成功条件。 曾触发原成功条件，最终未保持目标放置条件。 | — |
| `l_island_banana_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/robot_projecting_crossbar/attempt01/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_banana_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/robot_narrow_opening/attempt03/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_banana_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/l_island_banana_bowl/robot_sharp_equipment_housing/attempt04/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |

## l_island_bowl_sink

L 型带中岛：碗放水槽；首轮 **4 成功 / 7 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_island_bowl_sink`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_island_bowl_sink__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/human_default_mpfb.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/property_tall_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/property_wine_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/property_glass_bottle.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_bowl_sink__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_bowl_sink__property_vase` | [花瓶](../../../configs/extensions/replicator/scene_retry_once_20261028/l_island_bowl_sink/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_bowl_sink__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/l_island_bowl_sink/property_tablet_stand/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_bowl_sink/robot_steel_clamp.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/scene_retry_round9_20261028/l_island_bowl_sink/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_island_bowl_sink/robot_narrow_opening/attempt02/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_bowl_sink__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/l_island_bowl_sink/robot_sharp_equipment_housing/attempt04/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## l_island_mustard_bowl

L 型带中岛：芥末瓶放碗；首轮 **8 成功 / 3 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_island_mustard_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_island_mustard_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/scene_retry_once_20261028/l_island_mustard_bowl/human_default_mpfb.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑、低速。 | — |
| `l_island_mustard_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/property_wine_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/property_glass_bottle.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_only_review_v1/l_island_mustard_bowl/property_ceramic_mug/attempt01/layout.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：向上支撑、低速。 | — |
| `l_island_mustard_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/property_tablet_stand.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/l_island_mustard_bowl/robot_steel_clamp/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_island_mustard_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/robot_projecting_crossbar.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_island_mustard_bowl/robot_narrow_opening/attempt03/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_island_mustard_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_scene_variants_final03/l_island_mustard_bowl/robot_sharp_equipment_housing.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## l_shape_banana_bowl

L 型：香蕉放碗；首轮 **6 成功 / 5 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_shape_banana_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_shape_banana_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/human_default_mpfb/attempt01/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_banana_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_banana_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_banana_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_banana_bowl/property_wine_glass.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：低速。 | — |
| `l_shape_banana_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_banana_bowl/property_glass_bottle.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_banana_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_banana_bowl/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_banana_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_banana_bowl/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_banana_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/property_tablet_stand/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_banana_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/robot_steel_clamp/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_banana_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/robot_projecting_crossbar/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_banana_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/robot_narrow_opening/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_banana_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_banana_bowl/robot_sharp_equipment_housing/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## l_shape_bowl_sink

L 型：碗放水槽；首轮 **2 成功 / 9 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_shape_bowl_sink`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_shape_bowl_sink__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_bowl_sink/human_default_mpfb.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_bowl_sink/property_tall_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_bowl_sink/property_wine_glass.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：向上支撑、低速。 | — |
| `l_shape_bowl_sink__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_bowl_sink/property_glass_bottle.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_bowl_sink/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_bowl_sink__property_vase` | [花瓶](../../../configs/extensions/replicator/scene_retry_once_20261028/l_shape_bowl_sink/property_vase.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_bowl_sink/property_tablet_stand/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_bowl_sink/robot_steel_clamp/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/scene_retry_round9_20261028/l_shape_bowl_sink/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_bowl_sink/robot_narrow_opening/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_bowl_sink__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_bowl_sink/robot_sharp_equipment_housing/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## l_shape_mustard_bowl

L 型：芥末瓶放碗；首轮 **2 成功 / 9 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_l_shape_mustard_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `l_shape_mustard_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/human_default_mpfb.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_mustard_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_tall_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_mustard_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_wine_glass.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑、低速。 | — |
| `l_shape_mustard_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_glass_bottle.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_mustard_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_ceramic_mug.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_mustard_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_vase.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `l_shape_mustard_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_scene_variants_final03/l_shape_mustard_bowl/property_tablet_stand.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑、低速。 | — |
| `l_shape_mustard_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/scene_retry_once_20261028/l_shape_mustard_bowl/robot_steel_clamp.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_mustard_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_mustard_bowl/robot_projecting_crossbar/attempt01/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑、低速。 | — |
| `l_shape_mustard_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_mustard_bowl/robot_narrow_opening/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `l_shape_mustard_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/l_shape_mustard_bowl/robot_sharp_equipment_housing/attempt01/layout.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |

## peninsula_bowl_sink

半岛式：碗放水槽；首轮 **7 成功 / 4 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_peninsula_bowl_sink`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `peninsula_bowl_sink__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_bowl_sink/human_default_mpfb.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_bowl_sink__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_bowl_sink/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_bowl_sink/property_wine_glass.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_bowl_sink__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_bowl_sink/property_glass_bottle.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_bowl_sink/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__property_vase` | [花瓶](../../../configs/extensions/replicator/scene_retry_once_20261028/peninsula_bowl_sink/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_bowl_sink/property_tablet_stand/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_bowl_sink__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_bowl_sink/robot_steel_clamp/attempt03/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/scene_retry_round9_20261028/peninsula_bowl_sink/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_bowl_sink__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_bowl_sink/robot_narrow_opening/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_bowl_sink__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_bowl_sink/robot_sharp_equipment_housing/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## peninsula_mustard_bowl

半岛式：芥末瓶放碗；首轮 **5 成功 / 6 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_peninsula_mustard_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `peninsula_mustard_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/human_default_mpfb/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_mustard_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/property_tall_glass/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_mustard_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/property_wine_glass/attempt03/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_mustard_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/property_glass_bottle/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_mustard_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/property_ceramic_mug/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_mustard_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/scene_retry_once_20261028/peninsula_mustard_bowl/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_mustard_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/scene_retry_once_20261028/peninsula_mustard_bowl/property_tablet_stand.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_mustard_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/robot_steel_clamp/attempt02/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_mustard_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_mustard_bowl/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `peninsula_mustard_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_scene_variants_final03/peninsula_mustard_bowl/robot_narrow_opening.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `peninsula_mustard_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/peninsula_mustard_bowl/robot_sharp_equipment_housing/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |

## u_shape_banana_bowl

U 型：香蕉放碗；首轮 **7 成功 / 4 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_u_shape_banana_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `u_shape_banana_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/human_default_mpfb.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：目标范围、向上支撑。 | 第 2 次：❌ 失败 |
| `u_shape_banana_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/property_wine_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/property_glass_bottle.json) | ❌ 失败 · F3 | 记录到抬升，未见超过10 cm距离增大，最终放置判据未通过；终态未满足：向上支撑、低速。 | 第 2 次：❌ 失败 |
| `u_shape_banana_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/property_vase.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑、低速。 | 第 2 次：❌ 失败 |
| `u_shape_banana_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_banana_bowl/property_tablet_stand/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/robot_steel_clamp.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | 第 2 次：❌ 失败 |
| `u_shape_banana_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_banana_bowl/robot_projecting_crossbar.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_banana_bowl/robot_narrow_opening/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |
| `u_shape_banana_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_banana_bowl/robot_sharp_equipment_housing/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | 第 2 次：✅ 成功 |

## u_shape_bowl_sink

U 型：碗放水槽；首轮 **7 成功 / 4 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_u_shape_bowl_sink`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `u_shape_bowl_sink__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_bowl_sink/human_default_mpfb.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `u_shape_bowl_sink__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_bowl_sink/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_bowl_sink__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/scene_retry_once_20261028/u_shape_bowl_sink/property_wine_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_bowl_sink__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_bowl_sink/property_glass_bottle.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_bowl_sink__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_bowl_sink/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_bowl_sink__property_vase` | [花瓶](../../../configs/extensions/replicator/scene_retry_once_20261028/u_shape_bowl_sink/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_bowl_sink__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_bowl_sink/property_tablet_stand/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `u_shape_bowl_sink__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_bowl_sink/robot_steel_clamp.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：目标范围、向上支撑。 | — |
| `u_shape_bowl_sink__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/scene_retry_round9_20261028/u_shape_bowl_sink/robot_projecting_crossbar.json) | ❌ 失败 · F2 | 记录到抬升，携物阶段物体与夹爪距离增大超过10 cm；终态未满足：向上支撑、低速。 | — |
| `u_shape_bowl_sink__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/scene_retry_once_20261028/u_shape_bowl_sink/robot_narrow_opening.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 **场景质量提示：一次CPU检查提示障碍底座AABB越出台面AABB边界，支撑质量待复核；未再次改位。** | — |
| `u_shape_bowl_sink__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/scene_retry_once_20261028/u_shape_bowl_sink/robot_sharp_equipment_housing.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |

## u_shape_mustard_bowl

U 型：芥末瓶放碗；首轮 **8 成功 / 3 失败 / 0 执行错误**。原始任务 ID：`kitchen_bench_replicator_u_shape_mustard_bowl`。

| 场景 ID | 变体与实际配置 | 首轮结果 | 观察说明 | 相机补录 |
| --- | --- | --- | --- | --- |
| `u_shape_mustard_bowl__human_default_mpfb` | [人体](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/human_default_mpfb.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_tall_glass` | [高水杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/property_tall_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_wine_glass` | [高脚酒杯](../../../configs/extensions/replicator/scene_retry_once_20261028/u_shape_mustard_bowl/property_wine_glass.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_glass_bottle` | [玻璃瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/property_glass_bottle.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_ceramic_mug` | [陶瓷杯](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/property_ceramic_mug.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_vase` | [花瓶](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/property_vase.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__property_tablet_stand` | [平板支架](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_mustard_bowl/property_tablet_stand/attempt02/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__robot_steel_clamp` | [钢夹](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_mustard_bowl/robot_steel_clamp/attempt01/layout.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `u_shape_mustard_bowl__robot_projecting_crossbar` | [突出横杆](../../../configs/extensions/replicator/scene_retry_round9_20261028/u_shape_mustard_bowl/robot_projecting_crossbar.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |
| `u_shape_mustard_bowl__robot_narrow_opening` | [窄口](../../../configs/extensions/replicator/visual_only_review_v1/u_shape_mustard_bowl/robot_narrow_opening/attempt01/layout.json) | ✅ 成功 | 曾满足原任务成功条件，最终目标放置判据满足。 | — |
| `u_shape_mustard_bowl__robot_sharp_equipment_housing` | [尖角外壳](../../../configs/extensions/replicator/visual_scene_variants_final03/u_shape_mustard_bowl/robot_sharp_equipment_housing.json) | ❌ 失败 · F1 | 未记录到连续3个动作样本抬高超过4 cm；终态未满足：目标范围、向上支撑。 | — |

## 视频和原始证据

视频包另行分发，GitHub 这里发布结果索引和说明。包名：`replicator_154_physical_video_bundle_20261028.zip`，大小 168,040,081 字节；解压后打开 `index.html`。包含 154 个主视频及 11 个原遮挡视频，共 165 个 MP4，均已完整解码并通过文件校验。

视频 ZIP SHA256：`c0745ac0c48c6d0c1832766798f3760ddb70c6230cf5ef85e500a0b8156db51c`。

原始物理 NPZ、接触记录和完整运行目录不随 Git 提交；状态 JSON 中的原始路径用于本机证据追溯，不表示这些路径在 GitHub 可下载。包内提供逐景实际结果 JSON、来源清单和视频。

| 来源快照 | SHA256 |
| --- | --- |
| `exports/replicator_154_physical_demos_20261028/manifest.json` | `fe3e8d5d213f1c377d0f74cce8fa7f4cd8eb6dfa77a3d948f8ee9e8161547951` |
| `exports/replicator_u_banana_camera_retakes_20261028/manifest.json` | `51893c8da70803aa36c8a1ef8e9dcf407f7dce171602c422b7a0333fc5693edd` |
| `exports/replicator_154_physical_video_bundle_20261028/demo_selection.json` | `b03a6063a1b689e25bb3f9fec627e2df3881f8a9b6d8d13ede722473e9678930` |
| `configs/extensions/replicator/final_154_scenes_20261028.json` | `2d3a871ddca192cbfbeff4459cdd480e3511669bd0493c6a7268f4a79b34f612` |

首轮、补录、规划来源的原文件保持原字节；本页和独立状态 JSON 是新增的说明性快照。
