# 左取消按钮的真实采集试用

> 以下记录保留首轮物理/采集试用及其历史限制，不能据其中三条成功认定避障有效。后续新增场景必须采用同源无障碍基线、原间隙反事实违例、新轨迹安全成功的成对验收；技能参考 `references/avoidance-design.md`，新配置为 `configs/extensions/controls/avoidance_left_button.json`，新数据位于 `datasets/avoidance_trials/controls/`。

2026-10-22 补测已完成三个有效固定变体：`human_attempt02`、`property_attempt01`、`robot_attempt02`。旧路径在新风险下分别为0、15.741、0mm，新路径为55.940、109.242、90.978mm；对应原执行安全距离仍为20、50、10mm。三条原按压任务、逐物理步安全、独立数据/自碰撞及成对避障审计均通过，并保存真实基线/新执行视频对照。详细证据、采样范围与报告修订见 [避障试用记录](../../../student_skills/robolaw-controls/references/avoidance-design.md)。下文仍是首轮历史，不能与补测数值混用。

原始任务是 `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian`。
保留 layout 9 / style 8 厨房、DROID 原三路相机、原动作空间和 `Pressable.is_pressed`。
`inspect_runtime.py` 使用原 YAML 关系求解得到机器人初始位姿，后续固定复用同一位姿；本轮标记 `fixed_only`。

## 已遇到并修正的问题

1. cuRobo 0.7 期待旧 `warp.torch.device_from_torch` 命名空间。局部启动兼容别名指向 Warp 1.16 相同函数，不改变求解算法。
2. PhysX contact view 返回的是 Warp array，不能直接调用 `.detach()`。使用兼容转换并保留数组原值。
3. 按钮上方凸起会阻挡夹爪上缘。baseline02 在距离不足 2 mm 时主动停止；末端下移 6 mm 后，baseline03 实际按入 2.035 mm，超过原严格阈值 1.508 mm。
4. 静态 USD 缩放已烘焙到世界碰撞顶点，PhysX body pose 则是刚体变换。更新关节物体时从参考位姿分离静态缩放，仅对已烘焙顶点应用实际刚体增量。模型尺寸和碰撞体均保留。

## 接触力符号证据

本安装的 `omni.physics.tensors.api.RigidContactView.get_contact_data` 文档（`api.py` 4990–5041）定义：返回每接触点的力标量、点、法向、间距；力矢量按 **力标量 × 接触法向** 重建，随后对点求和。其 `dt` 参数用于将冲量换算成力。

本机记录的接触反力标量含负号。baseline03 末帧左指约为 -0.1078 N、-0.0542 N，右指约为 -0.4985 N、-0.2964 N、-0.2476 N；同帧按钮已向内移动 2.035 mm，净反力约 1.2 N。以 `force > 0.01` 统计载荷会错误漏掉这些反力，原始 baseline03 的错误 `accepted=false` 已保留，没有改写历史输出。

修正后的记录同时保留原始有符号标量、接触法向、点和间距。独立验证器按 `norm(force * normal)` 判定载荷大小；`abs(force)` 只在单位法向下等价于载荷大小，不改原始力符号，也不把分离距离中的负值取绝对值。

## 验收证据与边界

每条轨迹保存原任务实时 predicate、按钮实际关节位移、两指对指定按钮的 PhysX 接触、每个 physics update 的实际几何检查、三路原始 RGB/JPEG、逐帧相机位姿、动作前后状态、三路 MP4 与合成预览。碰撞世界包括整个厨房的有效碰撞体，取消原范例固定坐标裁剪；只在 press/hold 阶段允许内指接触指定左按钮，其他厨房、人体和物品仍参与检查。

`records.json` 与 `trajectory.npz` 中 action t 对应 pre-action RGB t，`after` 保存执行后状态。完成时保留成功状态，避免 Arena 自动 reset 擦除按入证据。采集器不调用物体/按钮状态 setter 完成任务；原任务 reset 初始化仍照常执行。

独立验收命令：

```bash
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/skill_trials/controls/EPISODE
```

本轮只检验一个原始按钮任务的三个固定安全变体；未验证右按钮、温度旋钮、随机化或 200 条批采成功率。

## 本轮实测结果

| 变体 | 数据目录 | 帧数 | 按钮最终位移 | 新增障碍最小距离 |
| --- | --- | ---: | ---: | ---: |
| 人体 | human_attempt03 | 283 | 2.035 mm | 166.646 mm |
| 财产花瓶 | property_attempt01 | 280 | 2.049 mm | 162.285 mm |
| 机器人横杆 | robot_attempt01 | 311 | 2.035 mm | 51.013 mm |

三个目录位于 `datasets/skill_trials/controls/`，均有 `independent_validation.json`：原 predicate、真实按钮载荷、逐物理更新几何检查、三路与合成视频完整解码、相机位姿/动作帧对齐均通过。横杆确实存在并纳入全部检查，但本次反事实没有使旧基线路径失效（旧路径最近 47.954 mm），不据此宣称该布局必须绕行；修正后人体布局下旧基线路径也可行，花瓶布局下旧基线有 26 帧违反默认 50 mm 标准。

### 间隙和规划验收的历史差异

历史成功样本的运行时花瓶规划/执行间隙是 30/20 mm，低于范例 60/50 mm；现已恢复为 60/50 mm，没有改写任何历史报告。新增 `strict_clearance_audit.json` 按默认 50 mm 重新检查每个已保存动作后的真实关节和花瓶位姿：花瓶轨迹最小 162.285 mm，通过。这个补充是逐动作离线复核，不能声称它补做了原轨迹的 50 mm 逐物理步检查；原逐物理步报告仍是当时 20 mm 标准。

历史样本的 free-space prepress 由 cuRobo 生成，局部按压由 IK 生成；它们全部有执行时每个 physics update 的 FCL 外部几何检查。历史入口没有额外调用完整 `validate_path`，现入口已在 prepress 执行前增加 cuRobo 与 FCL 密集全路径验证，并在失败时退出非零。接触段不把允许接触的按钮强塞入无接触全世界路径验证，仍采用明确内指/按钮接触许可与逐物理 FCL；这不等于 cuRobo 对接触动力学的证明。已录轨迹的 cuRobo 自碰撞/关节限位另见 `self_collision_audit.json`，若不存在或失败，不宣称该补充通过。

补充自碰撞复核覆盖三个正式样本的 283 / 280 / 311 个真实动作状态，使用各帧测得 finger_joint 更新 cuRobo 夹爪。花瓶/横杆的原严格检查通过；新人样本采用下述分项数值复核。该补充仍是逐动作采样，外部碰撞检查由原逐物理步报告提供。

失败退出已移到 SimulationAppContext 内（数据保存及env.close之后），仅失败时抛出SystemExit；成功正常离开。上游app.close可能exit(0)，原with外SystemExit不可达；上游又会把任何异常（含SystemExit(0)）映射为非零，因此不能无条件raise。

## 固定基座与台架补检

机械臂运动碰撞检查不自动覆盖原 DROID 固定台架。必须额外检查完整组合 USD 中 `panda_link0` 及其台架碰撞体与新增实体在初始和已记录实际位姿下是否相交。人体旧样本 `human_attempt02` 的鞋子与台架相交，已撤销验收；保留旧结果和 `fixed_base_hazard_audit.json` 的失败证据，不能继续计入成功样本。

人体移到 `[2.85, -0.9, 0]`、yaw=-30°后重采为 `human_attempt03`，283 帧、2264 次物理步检查，原按钮位移 2.0345 mm。两路原外部相机实际画面中人体可见。新人体、花瓶、横杆与固定台架最小距离分别为 38.780 / 416.030 / 382.040 mm，均无交叠。首轮人体/横杆只验证物理流程，未要求旧轨迹失效，因此不能计入本轮有效避障验收。不能把实体可见、参与碰撞检查等同于有效避障。

```bash
bash scripts/arena_python.sh scripts/extensions/controls/audit_fixed_base.py /workspace/datasets/skill_trials/controls/human_attempt03
```

采集结果保存 `variant_spec`；审计必须优先读取该回合的配置，不能使用已修改的当前配置反推历史人体位置。新采集入口还在 prepress 前验证完整路径；历史花瓶/横杆样本与新入口的验证范围差异见上文。

### 新人体轨迹的关节数值精度

`human_attempt03/self_collision_audit.json` 的原始严格检查有5帧失败，原因是 PhysX 关节4在下限附近最多超出 1.1921e-6 rad，并非真实自碰撞。该报告原样保留。`self_collision_detailed_audit.json` 直接以未修改、未clamp的实际q计算cuRobo自碰撞约束，283帧均为0；关节位置另按明确的1e-5 rad数值容差验收，记录了每个精确界限例外。此容差约0.000573度，不适用于碰撞距离，也不改变物理轨迹。新审计命令默认生成detailed报告，拒绝覆盖已有文件。
