# G bowl → sink：第二次真实执行及运输修复

`datasets/replicator/g_shape_bowl_sink/base/attempt_02` 是保留的失败样本，不能计入任务成功。真实执行了预抓取、接近、闭合、抬升和运输；原 5 mm 落点门槛拒绝继续下放，因此没有释放或原任务成功。

独立证据位于 `reports/replicator_execution_20261023/g_bowl_sink/`：

- `attempt02_diagnostic_bundle01.json` 汇总完整证据；3569 个真实物理状态源 FCL 全部通过，原相机、固定底座、记录完整性与 provenance 通过。
- `attempt02_transport_explanation01.json`：闭合末到抬升末，实测 TCP→碗关系变化 36.446° / 50.614 mm。两指持续受力且加载点位于预声明局部 rim 区，仍不代表刚性夹持。旧运输规划继续使用闭合末附件，额外要求 TCP 旋转约 36.446°。实际落点误差为 49.400 mm。
- `attempt02_actual_gripper_fk01.json`：全部实际七臂关节、六指关节重建与 native body 对照；TCP 最大位置误差约 0.806 μm，九个夹爪 body 最大约 1.018 μm，没有发现 FK/body 映射错误。
- `attempt02_endpoint_tracking01.json`：末目标实际连续保持 0.6 s（包含规划尾段与显式 hold），TCP 跟踪误差仍 6.630 mm，q5 末速度约 0.04296 rad/s。额外等待未被证明能收敛；此次修复不延长 hold，不增加未审查的细调，不放宽落点门槛。

新的 `bowl_transport_binding.py` 在抬升末验证最新连续十个真实物理子步的双指受力、间距与原局部接触区，并核对实际七臂/六指关节、源 FK、原碗身份及完整源 FCL。它仅用 `inv(实际 TCP) @ 实际碗` 更新规划附件，不移动物体、不建立物理约束。全部闭合末至抬升末的附件变化另存 `lift_attachment_motion.json`；`rigid_grasp_verified` 始终为 false。

`bowl_transport_curobo.py` 仅在运输查询中锁定实际六个独立手指角，随后重建原源模型的携物球。原父类其余 phase 保持原行为。每次运输重新生成 MotionGen 路径，并对全部原稠密样本检查 CUDA joint/self/world 与独立完整源 FCL；任一失败均拒绝执行。源台面和目标 sink 的成对检查不变，原落点 `[0.05, -0.1]`、2 mm 非接触距离和 5 mm 落点门槛不变。

CPU 正常/负例与原 bowl 回归合计 49 项通过，CLI 与 CPU preflight 通过。当前只证明绑定和实现可检查，尚未证明新路径 GPU 可用，更未证明新真实任务成功。

由根代理安排 GPU 诊断（不启动 Kit、不执行物理动作）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/diagnose_bowl_transport.py \
  --episode datasets/replicator/g_shape_bowl_sink/base/attempt_02 \
  --output reports/replicator_preparation/g_bowl_transport_rebind_gpu01 \
  --seed 42 --gpu
```

诊断通过且根代理确认后，才能启动新的真实尝试。旧 attempt_02 与所有源报告保留：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/bowl_collect.py \
  --output datasets/replicator/g_shape_bowl_sink/base/attempt_03 \
  --seed 42 --device cpu --physics-threads 2 --torch-threads 2 --headless
```

后续仍需检查真实运输跟踪、持续抓持、下放/释放、原成功谓词，以及实际 q 自碰撞、源几何、固定底座、相机、背景与完整 provenance；任何 CPU 或 GPU 候选通过都不替代这些物理验收。
