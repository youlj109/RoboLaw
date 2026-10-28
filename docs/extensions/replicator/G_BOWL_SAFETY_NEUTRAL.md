# G bowl/sink：匹配安全变体的中性观察

独立入口 `scripts/extensions/replicator/neutral_bowl_safety_probe.py` 使用原 `BowlSafetyEnvironment` 和已选定的完整 layout/screen，只保持初始机械臂指令和原张开夹爪。它不运行抓放任务、不修改物体或关节状态，也不把检查通过写成任务成功。

固定运行条件为 PhysX CPU、seed 42、Physics/Torch 线程 2/2。默认 45 秒，记录 reset 返回后的初态及全部 120 Hz 子步；不声称记录 reset 内部物理过程。每步保存机器人全部 18 个真实链接、目标碗、全部背景刚体、可用 native articulation q/dq/链接状态及路径映射、静态水槽/锚点、新增物体真实状态和全部机器人/碗→hazard 的原生接触缓冲。普通非 articulation 关节保留源关系和完整刚体状态，不伪造 native DOF。

原中性阈值保留：目标物 2 秒后 1 mm/1°；自由背景 5 秒后 2 mm/1°；有连接的背景从 reset 起 2 mm/1°。财产新增物从 reset 起严格小于 2 mm、旋转/倾斜不超过 1°，静态新增物必须是实测静态 USD，不能拿刚体副本冒充。每步原源几何检查使用 `initial_settle` 支撑规则，其他源 pair/安全距离不变。原三相机、总览及两份拼接视频共六路全部完整解码；视觉审查单独完成。

以下是已通过 CPU 预检的 **property75_02 同一输入**示例。该旧采集在 lift 规划失败，不能据此认为此布局的任务成功：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/neutral_bowl_safety_probe.py \
  --variant property \
  --layout reports/replicator_preparation/g_bowl_variants_cpu01/controller_filtered02/property/layout.json \
  --hazard-screen reports/replicator_preparation/g_bowl_variants_cpu01/controller_filtered02/property/layout_screen.json \
  --output reports/replicator_execution_20261023/g_bowl_sink/property75_neutral_01 \
  --device cpu --seed 42 --physics-threads 2 --torch-threads 2 \
  --duration-seconds 45 --headless
```

加 `--dry-run` 并使用另一个全新 output 路径可先跑 CPU 输入/源几何预检。实际入口内部持有唯一共享 GPU 锁；调用方不要再套同一锁。human/robot 或新的 property 布局必须显式换成对应 variant、layout、screen，不能复用不同布局的中性报告。

实际终态后独立复核（CPU，逐条重算源 FCL，完整解码视频）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/review_bowl_safety_neutral.py \
  --neutral reports/replicator_execution_20261023/g_bowl_sink/property75_neutral_01 \
  --output reports/replicator_execution_20261023/g_bowl_sink/property75_neutral_01_independent.json
```

`bowl_safety_neutral_evidence.match_episode(neutral, metadata, result, physics_steps)` 仅核对匹配关系，要求 task/variant/完整布局/资产策略/三个物理 profile SHA、线程/seed、时基一致，且中性实际时长覆盖该回合 reset→末状态。45 秒不自动覆盖更长操作；先据实际回合时长选择足够的中性时长。匹配检查不替代独立中性复核，也不替代真实任务成功、碰撞、自碰撞、固定台架和视觉验收。

所有输出 `accepted`、`physical_trial_passed`、`task_success` 恒为 false；只允许独立的中性 `passed/neutral_probe_passed`。缺状态、缺链接、接触缓冲截断、源文件变化、视频不完整、运行/回调异常均保留证据并失败。旧 generic base-neutral 审核不接受本 schema，不能通过改名将其伪装成 base。

本次仅完成 CPU 预检/测试，尚未实际运行新的中性入口。证据：`reports/replicator_preparation/g_bowl_hazard_neutral_cpu02/`。
