# G 型 mustard → red bowl：第一轮物理场景记录

后续用户指出“物体摆放”不等于有效避障。本页三条保留为物理执行证据，不作为有效避障验收。新增强制因果对照与修正实测见 [avoidance-trial.md](avoidance-trial.md)。

2026-10-21 完成 `kitchen_bench_replicator_g_shape_mustard_bowl` 的人体、财产、机器人周边障碍三个固定变体，每类选取 1 条真实成功轨迹及视频。本次按用户要求只做小样本；覆盖 **1/14 个 Replicator 原始任务，其余 13 个未验证**，没有随机化、200 条或多环境速度结论。

从包含 `scripts/extensions/replicator/`、`kitchen_safety/extensions/replicator/` 的 `ylj` 统一提交分出 `ylj-replicator`，优先复用已运行的入口。`assignment.json.base_commit` 仅作来源追溯。用户指定分支与当前工作方式优先。

## 正式样本

| 变体 | datasets/skill_trials/replicator 下的目录 | 动作/RGB 帧 | 完整厨房物理子步复查 | 动作率最小障碍间距 |
| --- | --- | ---: | ---: | ---: |
| human_default_mpfb | human/attempt_04 | 512 | 3856 | 0.173601 m |
| property_vase | property/attempt_05 | 522 | 3936 | 0.110868 m |
| robot_projecting_crossbar | robot/attempt_03 | 512 | 3856 | 0.045664 m |

间距按所有动作状态及最后 next_obs 独立审计；完整物理子步安全另按原执行阈值复查。三条均满足真实 `result.success`、原 `PickAndPlaceTask` 谓词、终态碗支撑、双指瓶身接触/携带检查、完整厨房安全及以下独立报告：`source_relation_audit.json`、`fixed_base_hazard_audit.json`、`self_collision_audit.json`。对应 reports 目录中的 summary/validation/audit/videos 均通过；人体 8 个阶段、其余各 7 个阶段退出码全部为 0。失败或缺报告不能通过 runner。

每条都有原三路 1280×720 的 `external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`，以及 1920×360 的 `all_cameras.mp4`。人体另有 1280×720 `overview_camera.mp4`：脸、躯干、双臂、机器人、红碗和黄瓶可辨，真实台架部分遮住下腿。原三路内外参不变；新增视角单独保存 `overview/images.h5`、`overview/cameras.json` 和 `overview/frame_alignment.json`，全部 512 帧与原相机/动作同一物理时刻，`overview_validation.json` 通过。所有 JPEG/MP4 逐帧解码，并目视抽查起始、转运、终态；图像证据不代替物理检查。

总体报告是 `reports/skill_trials/replicator/summary.json`，覆盖表是 `docs/extensions/replicator/coverage.json`。运行 `python3 scripts/extensions/replicator/summarize_trials.py` 会重新读取上述正式名单与硬性验收报告。大数据和运行报告被 Git 忽略，需另行交接或在接收机器重采。

## 合法的共同初始布局

三变体保留同一原厨房、原杂物、DROID、原默认支座足迹、0.8 m 支座、yaw=-1.57、芥末瓶、红碗和原三路相机。共同 reset 坐标：robot `[0.61,0,0]`（基座再加支座高度），mustard `[0.77,-0.43,0.976899501]`，bowl `[0.45,-0.475,0.936899501]`。仅在初始 reset 设置已验证布局，执行中不传送、焊接对象或写物体状态制造成功。

`solve_relations=False` 采用合法固定解；每条独立调用未经修改的上游 `OnRelationValidator` / `NextToValidator` 及原 USD 包围盒变换，逐项核验源 YAML 全部 7 个关系，并核对录制基座姿态。真实支座原点不在足迹中心：当前支座到岛台相向边距 0.27961 m，在原 0.15±0.15 m 内；碗前侧边距 0.05737 m，高于原 On 的 0.05 m。不得用“机器人根坐标距离”替代真实验证器。

人体位置 `[-0.14,0.2,0]`、yaw 150°；花瓶位置 `[0.61,-0.61,台面+0.002]`；横杆位置 `[0.61,-0.84,台面+0.002]`、yaw 180°。runner 传入真实 obstacle_id，最终位置由本组 `layout.py` 解析；基础 JSON 内继承的 property/robot 无 ID 回退坐标不属于本次验证范围。

通用 CollisionWorld 排除 `/Robot`，因此另从实际组合 USD 提取固定台架/基座的碰撞凸包和可见 mesh，独立验证新增障碍初始无交叠。自碰撞报告直接检查真实未 clamp 的关节/夹爪，cuRobo 自碰撞代价必须为 0；关节限位仅单列 1e-5 rad 数值容差，不更改轨迹或碰撞间隙。

## 复现命令

先确认 `scripts/arena_python.sh` 对应自己的容器、挂载和解释器；不要把本机容器名当作他人机器的默认设置。使用尚不存在的 attempt 名，失败记录不覆盖。

```bash
python3 student_skills/robolaw-replicator/scripts/preflight.py --repo .
python3 scripts/extensions/replicator/restore_assets.py
python3 scripts/extensions/replicator/verify_assets.py
bash scripts/arena_python.sh scripts/extensions/replicator/audit_source_relation.py --layout configs/extensions/replicator/g_shape_mustard.json --report /tmp/replicator-relations.json
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --variant human --attempt repro_01 --overview-camera --dry-run
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --variant human --attempt repro_01 --overview-camera
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --variant property --attempt repro_01
bash scripts/arena_python.sh scripts/extensions/replicator/run_trial.py --variant robot --attempt repro_01
bash scripts/arena_python.sh -m unittest discover -s tests/extensions/replicator -v
```

入口自动设置 `HEADLESS=1`；当前 Arena CLI 不接受 `--headless`。所有 Kit/cuRobo 工作使用 `reports/skill_trials/gpu.lock`，每次运行有界 episode，CPU 导出检查在采集释放 GPU 后执行。其他 GPU 任务必须遵循同一锁；不终止他人训练。支持 `--seed`、`--max-steps` 诊断和 `--dry-run`，尚无多任务队列、成功配额、resume、随机化规格或多环境采集接口。

恢复脚本需要能读 USD/USDC 的 CPU `pxr`；本次使用宿主机 Python，不能因为某个未初始化 Kit 的解释器打不开 USDC 就判定源文件损坏。`external_assets.json` 固定记录 248 个真实源文件的 URL/字节数/SHA256，验证脚本检查哈希和源组合错误。依赖提取排除 provenance 字符串 `assetInfo.identifier`、`blender_asset_path/usd_source_file`，跟踪真实引用及 MDL 的 `using Module import *;`。原始资产、纹理不改写，大资产不放入技能 ZIP，遵守仓库第三方许可。

源 YAML SHA256：`cc781d2eadbfa2a3d3b884938f92a453ce09cf76db73590700eafcf15c9a69c1`。本机版本：torch 2.11.0+cu128、isaacsim 6.1.0.0、isaaclab 17.0.2、isaaclab-arena 0.3.0；CPU 物理、单环境、GPU cuRobo。静态预检不证明其他 13 个任务资产或运行时可用。

## 实测修正与失败保留

- 全厨房碰撞读取不沿用原一字型场景的固定裁剪。原厨房 104 个动态背景体使用 PhysX 递归路径 `/World/envs/env_0/kitchen/**`；单 `*` 漏嵌套对象。保存动作率/物理率姿态，规划前同步、逐物理子步复查，保留原非均匀 scale。
- 瓶底必要支持接触只适配源精确台面 `/World/envs/env_0/kitchen/CapSurface_03` 的原 6 mm 支持带，不因此豁免机械臂、其他面或深穿透。
- Kit 关闭可能直接 exit(0)，不能只看进程状态。runner 同时读真实物理结果、全部必要 sidecar、RGB 报告与各退出码；第一轮验收门槛有 3 个 CPU 回归测试；当前扩展至 6 项，新增几何检查需 arena 解释器的 FCL/Usd，不启动 GPU。
- 旧 human/attempt_01、property/attempt_03、robot/attempt_02 和无障碍 base/diagnostic_05 虽有物理完成记录，但上游 NextTo 和碗 On 边距不合格。原数据保留，各 `source_relation_audit.json` 标为 false，正式汇总排除。只修改 reset 的 robot y=0.183→0、bowl y=-0.43→-0.475，经过 CPU IK/FCL 预检后重采。旧无障碍反事实不适用于当前合法布局，不能沿用旧“19 帧违规”等结论。本轮没有新合法布局的无障碍实采，不能声称完成新的基线轨迹对照。
- 合法 property/attempt_04 在已释放后，额外退手规划被 cuRobo 拒绝；没有执行拒绝轨迹，样本计失败。CPU 复核定位为覆盖球对膨胀花瓶 AABB 的约 0.849 mm 侵入，实际 FCL 间距约 102 mm。只把花瓶 y=-0.59→-0.61，保留全部间隙，property/attempt_05 正常完成退手；未使用退手跳过策略。诊断源代码是 `diagnose_property_retreat.py` / `diagnose_property_spheres.py`。
- human/attempt_02 已物理/RGB通过，保留为早期合法样本。为增强人体可辨认度加独立 overview；human/attempt_03 因初版外观收敛硬判定在 0 动作时退出，保留失败。最终 human/attempt_04 记录 1024 帧有界纯渲染 warmup，物理步 64→64；外观低幅像素差诊断仍为 false，未伪称收敛。该诊断不属于物理成功门槛，实际图像完整解码、可见性、位姿/时间同步和所有安全门槛均通过。
- 其他早期资产/CLI/刚体匹配、夹持和路径规划失败均保留，未改写成成功。旧错误摘要有独立校正记录，正式样本只读当前 summary.json 的完整 gates。

## 安全证据边界

当前三种实体均进入规划/执行的障碍约束，并有完整实际轨迹和间距证据；没有对当前合法布局执行危险反事实，也没有新无障碍轨迹来证明每个障碍必然改变路径。横杆轨迹实际抬高携物，仍只作该次执行事实。花瓶是可动物理刚体，稳定后最大记录位移 5.53e-7 m；没有破碎模型，不宣称验证了损伤或人体风险等级。

其余 13 个源任务、其他厨房资产、更多障碍组合、随机化、批采成功率和最大速度全部保持未验证。后续逐任务实现原谓词、实际几何与物理/RGB验收，再增量提交学生分支。
