# Arena PT v4（2026-10-20）

默认配置指向 `configs/safety/profile.robot.pick_place.v4.json`。
`rules.pt_mode` 和 `pt_hs_inspired.version` 均为 `pt-hs-inspired-v4`，
适配器版本为 `arena-risk-observer-4-pt-projected`。旧 v1 文件留作历史记录。
源参考评价器未修改。本次升级保留全场景资产清单与有证据的支撑距离豁免。

## 测距公式

新模块 `kitchen_safety/pt_geometry.py` 按压缩包
`sampling_reference/safety_risk/physx_collector.py` 的 authored 几何支持区间实现：

1. Mesh 使用全部 authored 顶点；Cube/Sphere/Capsule/Cylinder 使用解析支持区间；
   Xform/Scope 使用子几何组合。缩放烘焙到刚体局部几何，
   每步用实测世界刚体姿态更新；固定几何缓存。
2. 每对碰撞体计算紧致世界 AABB 间距，再沿形状中心连线计算投影区间间距。
   Mesh 中心为顶点均值，解析形状用其中心，组合形状用当前世界包围盒中心。
3. 两者取较大值，多碰撞体对取最小值。机器人按实际刚体完整路径分组。
   AABB 只用于可证明的最小值剪枝，不替代投影计算。

目标物来源为 `link_target_projected_clearance_gt`，
环境资产来源为 `pt_environment_projected_clearance_gt`，
metric 为 `collider_world_mesh_aabb_centroid_projection_lower_bound`。
这仍是保守下界，不是精确三角面距离或 PhysX cooked 距离。

不支持的几何、未加载引用、缺失动态姿态和对象产生逐对缺测，
不回退到旧 AABB 并标成 v4。几何空声明也保守保留未知。
绑定的 `pt_geometry` 保存几何指纹、路径和错误；
物理步的 `distances.pt_v4` 保存新距离；
评价对保存 `distance_geometry_evidence`、`distance_coverage_complete`
和独立的 `legacy_aabb_*` 审计字段。

## Robotiq 抓持适配

原参考 profile 的 Franka 手、两指及末端手腕映射为 Arena：

| 作用 | 实际链接 |
|---|---|
| 末端手腕 | `panda_link7` |
| 手爪适配器 | `panda_link8` |
| Robotiq 基座 | `base_link` |
| 手指机构 | 左右 `outer_knuckle`、`outer_finger`、`inner_finger`、`inner_knuckle` |

每个名字必须解析为唯一碰撞体刚体完整路径，写入 `allowed_grip_link_paths`。
缺失或歧义会在初始化时失败。这是显式任务关系适配，并非两款夹爪的校准等价证明。

源阶段 `pregrasp/grasp/gripper_close/attach/lift` 映射到 Arena 的
`approach/grasp/lift`（Arena 没有独立 attach 阶段）。
运输、放置和释放不再获得这项正常抓持距离豁免。
仅距离项排除指定链接；其余机械臂距离、碰撞力和冲量继续评价。
双指真实抓持判定仍只使用两个内指，区别于距离豁免部件清单。

## 保留的 Arena 语义

- 碗、台面、冰箱、橱柜及其他已启用碰撞体的资产组继续生成机器人/搬运物评价对。
- 有证据的碗/台面支撑只豁免对应距离项，保留原等级与适配原因；力与冲量不豁免。
- 损伤缺测仍为未知；掉落候选继续使用当前实测速度、下降、抓持及支撑协议。
- HS、RS、IR 的规则和来源独立记录，没有随 PT 切换。
- 各资产沿用已有的未校准实验代理阈值。厨房资产不能因为切到 v4 就自动套用
  源包中另一件托盘或杯架的材料数值。

因此“PT v4”表示版本路由、几何口径与规则接入升级，不表示当前不同机器人、
不同资产和 Arena 的所有适配行为都与原包场景完全相同。

## 运行与历史数据

本次完成代码和默认配置部署，未运行测试或仿真采集。
旧轨迹、报告、视频没有重算，新采集必须使用新的 run/output。
旧 AABB 数据不能仅通过修改版本标签宣称为 v4 数据。
参考包内文件保留原样，部署备份包含所有被替换文件及哈希清单。
