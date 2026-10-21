# 首任务真实试用：one-wall open_fridge

2026-10-21在RoboLaw工作树`ylj`中测试，任务ID为`kitchen_bench_lightwheel_open_fridge`。本次用户要求每类1条，因此跑了无新增障碍基线，以及human/property/robot各1条；未跑200条。四组同时试用时共享已有`ylj`是本次明确安排，学生独立开展时仍使用分配的`ylj-doors`。源码应包含下述本组目录；技能ZIP是工作指引，不附带执行器、USD或数据。

## 已验证范围

| 变体 | 本机成功目录（`datasets/skill_trials/doors/`下） | RGB/动作帧 | 双指持续接触的拉门帧 | 120Hz外部几何审计数 | 最终门角rad |
| --- | --- | ---: | ---: | ---: | ---: |
| 无新增障碍 | `base_attempt_08` | 330 | 159 | 2970 | 0.31568924 |
| 人体MPFB | `human_attempt_02` | 292 | 131 | 2628 | 0.31629521 |
| 财产花瓶 | `property_attempt_01` | 327 | 156 | 2943 | 0.31509203 |
| 投射横杆 | `robot_attempt_01` | 327 | 158 | 2943 | 0.31457409 |

四条的原任务谓词、120Hz外部碰撞审计、门体37个碰撞体覆盖、三路RGB及合成视频逐帧解码、相机/机器人状态对齐均通过。每个目录包含`result.json`、`validation_full.json`、`audit.json`、`collision_coverage.json`、`metadata.json`、`trajectory.json/.h5`、`images.h5`、`cameras.json`和四个MP4。合成预览是`all_cameras.mp4`，原视频为`external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`。
补充的`self_collision_audit.json`也已通过：四条分别330/292/327/327个实际15Hz动作状态，cuRobo自碰撞和关节限位均无违规，夹爪使用测得开度；它不替代120Hz外部碰撞审计。

本次仅固定布局/seed42验证，标记`fixed_only`；不是随机化或批量成功率证明。三种新增物体均有真实碰撞，花瓶为动态刚体，记录中位移0、最大倾斜0.000021°。实际轨迹15Hz采样的最小机器人表面距离：人体0.13758m、花瓶0.28178m、横杆0.13598m，执行门限分别0.02/0.05/0.01m。人体限制冰箱左侧接近空间；花瓶保护把手旁台面；横杆占据接近通道左缘。无障碍基线的离线反事实在这些摆放下也可行，因此**未证明必须绕行**；这些是可运行的安全扩展代表，进一步训练增广还需验证障碍导致的约束变化。其余7个原任务及其变体全部未验证。

## 可复制的真实入口

先确认项目包含`kitchen_safety/extensions/doors/{environment,collision}.py`、`scripts/extensions/doors/collect.py`与对应配置；旧的上游commit并不自动包含本次实现。运行环境实测为IsaacSim6.1.0.0、IsaacLab17.0.2、Arena0.3.0、cuRobo0.7.7.post1.dev5、Torch2.11.0+cu128、Warp1.16.0。恢复资产仍按项目README，首任务复用了本地one-wall厨房、DROID、人体、花瓶和横杆，无新来源的大资产。

以下为本机实际命令。其他机器替换容器/挂载/解释器；`--device cpu`是物理仿真设置，cuRobo仍使用GPU。`HEADLESS=1`通过容器环境传入，CLI没有`--headless`。共享GPU使用同一锁，单次执行有420秒时限。

```bash
docker exec -i -w /workspace -e PYTHONPATH=/workspace -e HEADLESS=1 \
  arena-kitchen-dev env -u ISAACSIM_ROOT_PATH -u ISAACLAB_PATH -u OMNI_SERVER \
  flock /workspace/reports/skill_trials/gpu.lock timeout 420 \
  /workspace/.venv-arena/bin/python scripts/extensions/doors/collect.py \
  --device cpu --variant human --output datasets/skill_trials/doors/human_new
bash scripts/arena_python.sh scripts/extensions/doors/validate_trial.py \
  datasets/skill_trials/doors/human_new
bash scripts/arena_python.sh scripts/extensions/doors/audit_hazard_clearance_cpu.py \
  datasets/skill_trials/doors/human_new
bash scripts/arena_python.sh scripts/extensions/doors/audit_fixed_base_hazards.py \
  datasets/skill_trials/doors/human_new
```

`--variant`现支持`base/human/property/robot`；每次用新的输出目录，已有数据会拒绝覆盖。失败退出码非0且保留报告/已录视频。CPU预检可运行`check_initial_geometry_cpu.py`检查厨房与新增物体初始几何；机器人初始放置另由原生graph solver处理。实际记录关节的cuRobo自碰撞/限位审计入口为`audit_self_collision.py`，也需GPU与同一锁。该脚本检查15Hz实际动作状态，不是120Hz自碰撞审计。采集时的120Hz审计负责外部几何。

## 本次排障得出的约束

- 原关节范围0..90°，`openness_threshold=0.2`且源predicate严格大于，所以门角须**超过18°**，并先满足离开reset位置超过5%。17.09°的`base_attempt_07`保留为失败；没有放宽阈值。其他任务从各自上下限和源比例函数读取，不能通用写成`q/upper_limit`。
- 中部把手抓点在约17°时逼近手臂关节限位。当前抓点为主把手中心上方0.12m，z约1.25318m，仍在原主把手碰撞体z约0.9933..1.2731m内。此偏移只对该冰箱实测有效；CPU候选扫描后仍要真实接触验证。拉门使用测得门角生成下一段，单步臂关节增量上限0.025rad，不写门角、不施加外部附着力。
- 原CollisionWorld坐标裁剪漏掉该门全部37个碰撞体；本组局部适配器取消裁剪，并把实测门刚体变换同步到FCL和cuRobo。不是将固定工作区上限略放大；其他门不能硬编码37。
- 原整夹爪聚合凸包填满两指间隙，会把正确抓柄判碰撞。局部适配器仅去掉这个重复包络，仍保留所有手臂以及按实际关节角更新的夹爪基座/逐指几何。允许接触仅是指定内侧手指与把手，门板和其他机器人部件照常检查。
- cuRobo的忽略匹配在当前适配器使用末级组件名，完整USD路径不会命中。接触阶段只传精确把手组件名；另由FCL限制为手指接触，不允许全门豁免。按实际`finger_joint`更新夹爪，不能沿用瓶任务默认开合量。
- PhysX filtered contact view要求传感器pattern与filter列表一一对应；Warp数组使用Warp兼容转换。保存实际双指接触点、计数和带符号法向力（N）；带符号是传感器法向约定，不把未使用缓冲区计为接触。
- 外部安全检查挂在每次`scene.update`后，dt=1/120s；RGB/动作15Hz，每动作8物理步，另有一次动作终态检查。只做15Hz事后检查不足以宣称120Hz物理审计。自动成功reset局部关闭以保留成功时刻，但调用的仍是原predicate。
- 当前原生graph solver对固定新增物体的边界框只支持绕Z轴90°的整数倍旋转。人体yaw=-30°曾在放置前明确失败，改为0°；这不是所有物体或控制器普遍的旋转限制。
- 花瓶/横杆要先检查与冰箱静态侧板的交叠。跨界x位置曾侵入源侧板C009，已移至相邻台面内。横杆资产高约0.3125m，是台面夹具，不能当成人体高度的落地障碍摆放。
- `/Robot` 子树从外部世界读取时被排除，而运动包络不包含stand；必须补查新增障碍与真实静态台架/固定基座几何，不能由原graph通过推断。`audit_fixed_base_hazards.py` 使用实际源compose资产与记录基座位姿，检查初态及所有记录的财产物体位姿。本组三个成功变体补审均通过：人体到stand最小86.77mm、花瓶731.68mm、横杆579.93mm；各目录保存 `fixed_base_hazard_audit.json`。该检查发现其他组旧人体穿台架并促成重采，不放宽或跳过碰撞。
- 该任务graph的reachability检查找到0个门把手目标，布局报告的通过不能证明开门可达性。Warp1.16缺少旧cuRobo期望的`warp.torch.device_from_torch`入口时，入口使用项目已有同等兼容别名，不改固定上游源码。

完整本机复现笔记为`docs/extensions/doors/physical_trial_notes.md`，机器可读汇总为`reports/skill_trials/doors/summary.json`。下载技能后本机数据目录可能不存在；不得据此声称在学生机器已完成物理验证。
