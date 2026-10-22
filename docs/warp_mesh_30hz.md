> 当前通用默认采集策略见 [offline_risk_workflow.md](offline_risk_workflow.md)：源状态和接触在线保存，本文的 Warp 几何与风险计算在离线阶段执行；源时刻与实际处理时刻分别记录。

# Warp GPU 完整网格测距与 30 Hz 采集

当前人体芥末瓶场景使用 Warp 1.16.0、CUDA GPU 查询。物理仍为 120 Hz；动作/轨迹、几何采样、HS/PT/RS 标签及离线 RGB 来源状态为 30 Hz。每四个物理步进行一次几何采样与风险评价，物理接触和关节状态仍逐物理步记录。RGB 按实测状态离线渲染，不作为实时视觉感知输入。IR 保持 disabled。

## 几何算法

每份 authored 碰撞网格构建一次 GPU BVH。所有源三角形均进入完整搜索；点到网格查询只提供上界和保守剪枝依据，不作为最终表面距离。候选三角形由扩张 AABB 查询得到，最终计算六个顶点到面、九条边对边以及逐边穿面相交。距离为非负分离距离，表面相交为 0，不输出穿透深度。网格中的退化三角形以其线段/点参与距离计算。

GPU 查询采用 float32，几何规模和误差预算以米记录。点查询可能排除细长/退化面，因此将这些面的联合包围盒加入距离下界，避免因忽略这些面而错误剪枝。查询边界使用数值 padding。该实现是完整三角网格数值查询，不是符号意义上的无限精度计算。

固定缩放烘焙进顶点，逐时刻更新刚体旋转和平移。姿态完全相同时可以复用几何对结果；任一姿态改变使缓存失效。GPU 归约后仅回读距离和最近点。禁止没有 CUDA 时退回 CPU/FCL。

`fcl_distance.py` 保留历史文件名供拓扑/查询适配使用；运行时 HS/PT/RS 几何对象来自 `warp_mesh_backend.py`，其中 FCL 风格的类型只是兼容接口，没有调用 FCL 库。规划及历史离线诊断仍可使用其原有几何库，不能把风险后端的纯 Warp 声明扩展到整个规划系统。

## 规则与记录

HS/PT/RS 阈值及用户指定豁免沿用先前配置；PT v4 显式验证新的 Warp metric/source 字段，旧 FCL source 仍支持历史回放。每个决策从最近已完成的物理状态计算，在动作提交之前产生。逐物理步记录中的 `geometry_fresh` 和 `geometry_sample_physics_step` 区分本步新计算与保留的上次几何。

## 验证与运行

```bash
bash scripts/arena_python.sh scripts/check_risk_perception.py
bash scripts/arena_python.sh -m unittest discover -s tests -p test_warp_mesh.py -v
bash scripts/arena_python.sh scripts/collect_safety_benchmark.py --ids human_default_mpfb --num-envs 1 --episodes-per-id 1 --max-attempts 1 --seed 42 --output datasets/risk/warp30_run01 --run-id warp30_run01
```

测试覆盖已知间隙、边对边最近距离、穿面相交、共面接触、退化几何和姿态缓存失效。可选 FCL 仅用于开发对照，不参与正式风险采集。完整采集耗时应以运行记录实测；使用 GPU 不自动保证比 CPU FCL 更快。

## 本次人体场景采集实测

2026-10-22 的 `human_warp30_review_20261022_f` 完成任务，记录 2,292 组 30 Hz 动作/标签，物理 9,168 步；三路离线实测状态画面各 2,292 帧，视频 76.4 秒。完整轨迹因果回放、几何来源、时间对齐和 RGB 验证通过。采集循环实际耗时 4,200.34 秒（约 70 分钟），其中物理执行 4,070.03 秒、规划 126.70 秒；尚未达到实时，不代表优于 FCL。HS 为 L1–L2，PT 为 L0–L2，RS 已知分项为 L0；损伤和权威故障状态仍缺测，不能视为完整安全证明。该次复现属于 review 数据，未通过生产配额准入。详见 `validation_warp30_episode_20261022.json`。

相机写入器原有 15 Hz 硬限制已扩展到 15/30 Hz。原物理与同步标签保持不变，修复后独立渲染成功，旧失败记录归档保留。视频仅导出本地，不上传模型库。

整帧并行执行与性能测量见 [warp_frame_batch.md](warp_frame_batch.md)。当前正式采集共享 HS/PT/RS 查询队列，采用 CUDA Graph 和候选前沿宽度 8。
