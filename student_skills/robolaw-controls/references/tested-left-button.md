# 左取消按钮：真实试用所得的实现约束

**修复进展：** 新的左按钮 `human_repaired01`、`property_repaired01`、`robot_repaired01` 与 `base_repaired02` 已另行验证；按 [修复后的实测与复现](repaired-left-button.md) 使用新配置。下述历史撤回及严格验收要求保留，不能据新样本复活旧回合。

> 以下记录保留首轮物理/采集试用及其历史限制，不能据其中三条成功认定避障有效。后续新增场景必须采用同源无障碍基线、原间隙反事实违例、新轨迹安全成功的成对验收。`avoidance-design.md` 保存此前单基线试验，`configs/extensions/controls/avoidance_left_button.json` 是该轮历史配置；当前启动入口、修复配置与完整验收步骤见 [repaired-left-button.md](repaired-left-button.md)。

该轮用户要求每个技能选择一个原任务，构建人体、财产、机器人周边障碍三个变体，每变体一条真实采集并保存视频；不构成随机化或批采成功率结论。当前配额以 workflow 为准。

选用 `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian`，保留原 PressButtonTask、原 YAML 关系、DROID 相机/动作空间。Lightwheel 官方厨房为 layout 9 / style 8，恢复版本 UUID 为 `1a892eee-c6c7-4077-9791-bfa3686b3cf7`。

## 历史试用入口（仅作溯源）

以下命令解释首轮记录，不是当前新交付的启动流程。技能压缩包本身不携带大型厨房、数据或完整 RoboLaw 仓库；先取得 `origin/ylj` 基线，再使用 [当前修复入口](repaired-left-button.md)，不能直接运行本段默认旧配置后宣称完成新验收。

```bash
bash scripts/arena_python.sh scripts/extensions/controls/restore_assets.py
bash scripts/extensions/controls/run.sh inspect_runtime.py --device cpu
bash scripts/extensions/controls/run.sh collect_trial.py --device cpu --variant base --output datasets/controls/left_button_base_01 --max-steps 480
bash scripts/extensions/controls/run.sh collect_trial.py --device cpu --variant human --output datasets/controls/left_button_human_01 --max-steps 480
bash scripts/arena_python.sh scripts/extensions/controls/validate_trial.py datasets/controls/left_button_human_01
bash scripts/arena_python.sh scripts/extensions/controls/audit_augmentation.py datasets/controls/left_button_human_01 --baseline datasets/controls/left_button_base_01/records.json
bash scripts/arena_python.sh scripts/extensions/controls/audit_fixed_base.py /workspace/datasets/controls/left_button_human_01
bash scripts/extensions/controls/run.sh audit_self_collision.py datasets/controls/left_button_human_01
```

`--variant` 支持 base/human/property/robot；每次选择新的输出目录，脚本拒绝覆盖既有轨迹。复现财产与机器人障碍时改 variant 和输出目录。`run.sh` 显式传入容器内 `HEADLESS=1` 并以共享 flock 串行使用 GPU；先按 workflow 核实 `ARENA_CONTAINER`、`/workspace` 挂载、解释器及锁文件父目录。宿主机 export 不会自动传入 `arena_python.sh` 容器；该固定 Arena CLI 也不接受 `--headless`。

`audit_augmentation.py` 比较原无障碍 baseline 与新增障碍下的几何可行性，需要先有本轮 `inspection/stage.usda` 和 `base_attempt03/records.json`。在新机器上改为本机实际生成的基线文件；缺证据时不得把反事实测试标为通过。这是离线几何检查，不把旧路径真实执行到新增障碍物中。

## 避免重复踩坑

- **按压区域**：按钮行程 3.016605 mm，原谓词严格要求大于 50%，即超过 1.508303 mm。不能降低阈值。对本厨房左按钮，按压 TCP 比按钮中心低 6 mm 可避开上方突起；末端目标、机器人基座及厨房坐标必须从实际资产/关系求解确认，不能把这个偏移机械套到右按钮或其他厨房。
- **空夹爪**：共享瓶子规划适配器的闭合默认是携瓶宽度 0.38 rad；按按钮时空夹爪实际接近 0.785 rad，应使用测得的 finger_joint 更新 cuRobo 锁定夹爪几何。
- **动态碰撞**：不沿用原厨房固定裁剪范围；保留所有有效厨房碰撞体。烤面包机其他旋钮在物理仿真中也可能运动，按测得的 body pose 更新各部件碰撞。源 USD 的静态缩放已烘焙到顶点，更新刚体位姿时不要再次缩放，也不要直接把带缩放矩阵当刚体旋转。
- **允许接触**：仅 press/hold 阶段允许指定内指接触左取消按钮。其他机身、夹爪底座、机械臂、人体、花瓶、横杆仍检查；不忽略整个烤面包机。记录每个 physics update 的实际几何结果。
- **接触 API**：`RigidContactView.get_contact_data(dt)` 返回 Warp/torch/numpy 类型之一，按实际类型转换。SDK 的力矢量是 `force_scalar * normal`；本机标量包含负号，保留原始标量和法向，用矢量范数或单位法向下的绝对值统计载荷大小。不能因 `force > 0` 为假就认定没有接触，也不能只用接触点数量认定已加载。保留接触点、间距、法向和同帧按钮位移作交叉证据。
- **成功终态**：按钮释放后可能返回未按状态。记录原 predicate 首次成功时刻与保持阶段，阻止成功后的自动 reset 擦掉终态；不通过关节 setter 完成任务。原任务本身没有 hold 时间阈值，额外保持只作独立记录。
- **数据**：一帧 RGB 对应一个动作前状态；独立验收完整解码三路 MP4 和合成预览，并从机器人 body pose 与原相机固定外参重建相机位姿检查对齐。

试用记录和详细限制在仓库 `docs/extensions/controls/TRIAL_NOTES.md`；真实视频与验证报告在 `datasets/skill_trials/controls/`。右取消按钮、温度旋钮、随机化与 200 条批采仍属于未验证范围。

## 实测覆盖与复核边界

| 任务 | 人体 | 财产花瓶 | 机器人横杆 |
| --- | --- | --- | --- |
| 左取消按钮 | human_attempt03，283 帧 | property_attempt01，280 帧 | robot_attempt01，311 帧 |
| 右取消按钮 | 未验证 | 未验证 | 未验证 |
| 烤箱温度旋钮 | 未验证 | 未验证 | 未验证 |

本轮三个左按钮样本均通过原 predicate、接触载荷、逐物理外部几何与视频独立验证。对应新增障碍的逐动作最小距离分别为 166.646 / 162.285 / 51.013 mm。横杆反事实未使旧路径失效，不把这一配置说成强制绕行场景；修正后人体布局下旧路径也可行；花瓶反事实改变旧路径可行性。

保持默认花瓶规划 60 mm / 执行 50 mm。历史成功样本曾使用 30/20 mm，修订后保持历史文件原样，补充 `strict_clearance_audit.json` 按 50 mm 逐动作复核通过；这不回溯宣称按 50 mm 做过逐物理步审计。现入口在 free-space prepress 前调用完整 `validate_path`，历史样本则有执行时每 physics update FCL；接触段是局部 IK 加受限接触对及逐物理几何检查，不代表 cuRobo 求解了接触动力学。可用 `run.sh audit_self_collision.py EPISODE...` 独立检查实际动作状态的自碰撞/限位，检查输出内容而不只看进程结束。

严格间隙复核（保留旧文件）：

```bash
bash scripts/arena_python.sh scripts/extensions/controls/audit_augmentation.py datasets/skill_trials/controls/property_attempt01 --report-name strict_clearance_audit.json
```

反事实脚本也接受 `--stage` 和 `--baseline`，应指向本机实际生成的初始 stage 与无障碍基线记录。新版本拒绝覆盖已有报告；默认 augmentation_audit 已用当前默认执行间隙，strict_clearance_audit 仅用于历史样本补审。

补充自碰撞复核覆盖三个正式样本的 283 / 280 / 311 个真实动作状态，使用各帧测得 finger_joint 更新 cuRobo 夹爪。花瓶/横杆的原严格检查通过；新人样本采用下述分项数值复核。该补充仍是逐动作采样，外部碰撞检查由原逐物理步报告提供。

执行退出码：失败报告和数据保存、`env.close()`完成后，必须仍在`SimulationAppContext`内部仅当失败时抛出异常；with外的失败退出可能被`app.close()`的exit(0)跳过。成功时正常离开，不能抛出SystemExit(0)，上游对任何异常均使用非零退出。验收始终读取result及独立报告，不能只看退出码。

## 固定基座与台架补检

机械臂运动碰撞检查不自动覆盖原 DROID 固定台架。必须额外检查完整组合 USD 中 `panda_link0` 及其台架碰撞体与新增实体在初始和已记录实际位姿下是否相交。人体旧样本 `human_attempt02` 的鞋子与台架相交，已撤销验收；保留旧结果和 `fixed_base_hazard_audit.json` 的失败证据，不能继续计入成功样本。

人体移到 `[2.85, -0.9, 0]`、yaw=-30°后重采为 `human_attempt03`，283 帧、2264 次物理步检查，原按钮位移 2.0345 mm。两路原外部相机实际画面中人体可见。新人体、花瓶、横杆与固定台架最小距离分别为 38.780 / 416.030 / 382.040 mm，均无交叠。首轮人体/横杆只验证物理流程，未要求旧轨迹失效，因此不能计入本轮有效避障验收。不能把实体可见、参与碰撞检查等同于有效避障。

```bash
bash scripts/arena_python.sh scripts/extensions/controls/audit_fixed_base.py /workspace/datasets/skill_trials/controls/human_attempt03
```

采集结果保存 `variant_spec`；审计必须优先读取该回合的配置，不能使用已修改的当前配置反推历史人体位置。新采集入口还在 prepress 前验证完整路径；历史花瓶/横杆样本与新入口的验证范围差异见上文。

### 新人体轨迹的关节数值精度

`human_attempt03/self_collision_audit.json` 的原始严格检查有5帧失败，原因是 PhysX 关节4在下限附近最多超出 1.1921e-6 rad，并非真实自碰撞。该报告原样保留。`self_collision_detailed_audit.json` 直接以未修改、未clamp的实际q计算cuRobo自碰撞约束，283帧均为0；关节位置另按明确的1e-5 rad数值容差验收，记录了每个精确界限例外。此容差约0.000573度，不适用于碰撞距离，也不改变物理轨迹。新审计命令默认生成detailed报告，拒绝覆盖已有文件。
