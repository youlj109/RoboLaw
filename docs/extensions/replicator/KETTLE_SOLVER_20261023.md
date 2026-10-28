# G 型厨房水壶启动漂移：局部求解设置诊断

完整启动记录 `base/a20261023_base02` 暴露了旧动作率记录未覆盖的峰值：水壶按钮、壶盖世界位移约 2.038、2.031 mm，超过原 2 mm 门限，采集在任务动作前被拒绝。父子相对运动只有微米量级，主要是整个水壶轻微摇摆；这项诊断不能替代世界坐标门禁。

先检查了源质量、质心、惯量、三条关节、drive、支撑几何及求解设置，再预声明并运行以下三组局部求解迭代。没有更改质量、几何、碰撞、初始位姿、关节约束或验收阈值，没有运行时写物体/关节状态。

| 水壶位置/速度迭代 | 10 秒记录 | 按钮世界位移峰值 | 原世界稳定性门禁 |
| --- | ---: | ---: | --- |
| 16 / 1 | 1201 个状态 | 3.286556 mm | 失败 |
| 32 / 2 | 1201 个状态 | 3.030578 mm | 失败 |
| 64 / 4 | 1201 个状态 | 1.299490 mm | 通过 |

三组均为原 G 型芥末瓶无新增风险布局、seed 42，从 reset 返回初态开始逐物理子步记录。结果保存在 `reports/replicator_execution_20261023/g_mustard/kettle_solver_{16_1,32_2,64_4}_01/`，失败报告和日志全部保留。`kettle_solver_comparison01.json` 绑定三份报告、原始 NPZ 和曲线图的 SHA256。

这些 10 秒诊断使用探针的默认线程设置，而正式采集使用 2 个 PhysX 工作线程和 2 个 torch 线程。因此它们仅用于筛选设置，不能直接认证正式采集。运行时已读回四个刚体的 typed USD 求解属性；当前原生 tensor 接口没有内部求解迭代 getter，不能声称已读回引擎内部数值。

## 新物理配置的范围

`background_physics.py` 的 v2 覆盖保留已有冰箱机身安装，并且仅给源水壶四个刚体添加 `PhysxRigidBodyAPI` 及八个求解属性（64 / 4）。源质量、COM、惯量、碰撞及全部源关节仍由原资产继承。强层作者字段检查拒绝额外编辑；元数据绑定原资产、实现和覆盖层哈希，以及全部三组诊断证据。

2 mm / 1° 世界门限保持不变，关节连接体从 reset 起检查；5 秒自由物品沉降窗也保持不变。任何旧记录都不能因设置更新而追认通过。

`neutral_v2_01` 已在正式采集的 CPU、2 个 PhysX 工作线程、2 个 torch 线程和 CPU RGB 复制设置下完成 45 秒观察：5401 个物理状态、675 帧原三相机及总览、15 项检查通过。水壶按钮/盖子世界位移峰值为 1.682625 / 1.681901 mm；全部六个预览视频完整解码。另实际查看了外视第二相机、手腕和冰箱总览的第 0 / 337 / 674 帧，抽样范围及观察保存在 `visual_review.json`，不声称逐帧语义审查。

`mobility_v2_01` 使用相同 source / recipe / overlay 哈希和 2 / 2 线程设置，通过冰箱门可动性检查：2 秒已知外力矩结束时，左右门分别转动 16.8236° / 12.7796°，机身没有记录到位移或转动。外力矩只用于独立机构诊断，不作为机器人操作成功证据。

这两项通过后，仍需完成两种匹配基线和三个变体的重新采集及独立审查，才能交付 G 型首任务。

```bash
docker exec -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 arena-kitchen-dev \
  flock /workspace/reports/skill_trials/gpu.lock \
  env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  /workspace/.venv-arena/bin/python scripts/extensions/replicator/neutral_task_probe.py \
  --task-id kitchen_bench_replicator_g_shape_mustard_bowl --device cpu \
  --duration-seconds 45 --episode-length-seconds 180 \
  --physics-threads 2 --torch-threads 2 --cpu-rgb-copy \
  --output reports/replicator_execution_20261023/g_mustard/neutral_v2_new
```

输出路径必须不存在。JSON 的完整逐项结果、原始状态和视频检查决定该次观察是否合格；示例命令不是已完成的实验。其他布局也须分别实测，不能由这一个 G 型诊断推断全部 14 个任务已通过。
