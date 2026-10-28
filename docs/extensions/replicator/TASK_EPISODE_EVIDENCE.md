# Task-aware G banana/bowl 与 bowl/sink 独立证据入口

这是新 schema 的 CPU 审查工具，不修改已完成的 Gmustard。`review_task_episode.py` 不启动模拟器或 CUDA，不把接触窗口当作任务成功，也不生成目视通过。支持真实任务对象名；sink 必须是原 `ObjectReference`，不能套用刚体 bowl 的状态。

当前 attempt03 是失败样本：2441 个 reset 起连续物理状态、305 个动作；2431–2440 子步双指加载通过，但 lift/release/final_settle 均没有实际执行。原任务失败，末态目标支撑为 0 N。完整 JPEG/六路视频解码、中性匹配、原源关系与全采样 fixed-base CPU 审计通过。缺失 self-collision GPU 审计、完整执行 provenance、实际运行时谓词仪器记录及操作视觉验收，不能追补成成功。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/review_task_episode.py \
  --episode datasets/replicator/g_shape_banana_bowl/base/attempt_03 \
  --neutral reports/replicator_execution_20261023/g_banana/neutral_v2_02 \
  --fixed-audit reports/replicator_execution_20261023/g_banana/attempt03_fixed_audit01.json \
  --output reports/replicator_execution_20261023/g_banana/attempt03_independent_review_fresh.json
```

失败退出码为 2，报告逐项保存失败/缺失；已有输出拒绝覆盖。未来完整 episode 可额外传 `--self-audit` 和 `--visual-review`。视觉报告必须由实际查看形成，给出 reviewer、scope、observations、frames、cameras、visual_sample_passed 与原 result/physics/images/overview images 的 `sha256`；成功验收的抽样至少覆盖 close/lift/release/final_settle。机器解码不等于目视验收。

## 下一次真实采集的 helper 接线

新文件 `kitchen_safety/extensions/replicator/task_collection_evidence.py` 已通过 CPU schema/负例测试；Kit 实际使用仍待首次运行验证。不要给历史 attempt 补生成 runtime evidence。collector 的修改由执行器所有者完成。

1. **SimulationAppContext 前**创建 `TaskRunProvenance(layout_path, collector_path=__file__, input_paths=[controller_profile_path])`。输入必须是本次真实读取的配置路径；64+ 项指定执行源码会保存初始文本及 SHA。新执行依赖须加入 `TASK_CODE`，不应只捕获 collector 本身。
2. `envdef = AssignedTaskEnvironment().build(...)` 后，**builder.make_registered/reset 前**调用 `provenance.bind_environment(envdef)`。它绑定真实 source/recipe/overlay、组合台架资产与原 YAML。
3. 唯一一次 `env.reset()` 返回后立即创建 `TaskPredicateRecorder(env, envdef)`；在任何 warmup 动作前调用 `predicate.sample_physics("reset", -1)`。它读取实际 spawned bounds、原任务阈值、native sensed body 和解析后的 filter 目标；不会调用会修改原成功状态的 `objects_settled`。
4. 在现有 recorder 的每子步 callback 中，保留原真实状态碰撞检查，并调用 `predicate.sample_physics(row["phase"], row["action_index"])`。不得替换已有 callback 或漏掉 warmup/initial_settle。
5. 每次 `env.step(action)` 完成、原 termination 已评估后，调用 `predicate.sample_action(phase, action_index)`；应先保存该真实动作终态，再处理 done/失败。physics 与 action 序列分开保存，不能把重复 action 末态伪装成新 physics step。
6. 保存原始物理/动作后且 env 仍存活时调用 `predicate.finish(out)`，最后 `provenance.finish_task(out, env.sim.get_physics_step_count())`。失败也保留原始数据和错误；缺失阶段不能生成通过记录。

输出包括 `task_collection_provenance.json`、`task_predicate_metadata.json`、`task_predicate_physics.npz`、`task_predicate_actions.npz`。原始 .1 N 支撑、速度严格小于 .003 m/s、向上支撑锥和原 consecutive steps 不变。CPU 复算确认原 rest → rise → support/success 顺序、实际传感器目标、所有原物理样本与已保存谓词的一致性。该 helper 不施力、不移动物体、不改 mass/碰撞/关节，也不宣布视觉成功。

## 原相机四元数说明

`attempt03_independent_review01.json` 保留旧审计的 external_camera_2 0.023329° 失败。原源相机 offset quaternion 范数为 0.9998499887；USD orient op 经 `GetLocalTransformation().ExtractRotationQuat()` 的旋转与将四分量直接欧氏归一化不同。新 `task_camera_audit.py` 使用原 USD authoring 语义，同时保留旧算法结果和输入/源码哈希。

真实 305 帧复算的最大角误差为 external 0.00000271°、external2 0.00000549°、wrist 0.00002424°。原 0.02°/0.1 mm 门槛保持；人工制造 0.021° 真实偏差或改分辨率的测试仍失败。没有改原相机配置或 raw 视频。

CPU 检查：

```bash
bash scripts/arena_python.sh -m pytest -q \
  tests/extensions/replicator/test_review_task_episode.py \
  tests/extensions/replicator/test_task_collection_evidence.py
```

Bowl 的抓点来自真实 rim 源几何；静态 sink 的最终原判据必须使用新采集实际记录的 reference transforms/bounds 和其专属支撑 filter。安全变体的 task-aware hazard schema 目前尚未实现，reviewer 明确拒绝将 base 改名作为变体验收。
