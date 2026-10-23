# lwx：C01 前方摆放修订

本轮完成 `kitchen_bench_lightwheel_open_fridge`（C01）的 human/property 前方摆放修订，保留首轮 robot 组。依据本仓库 README、C 组 prompt、`student_skills/robolaw-doors/SKILL.md` 及新版验收要求执行。飞书总清单不作为本轮全部完成的声明。个人分支 `lwx`，同步基线 `origin/ylj` 的 `035fc7c`，作者 `WinshareLi <22632139@zju.edu.cn>`。

正式报告：[gate_summary.json](lwx_front_20261023_final/gate_summary.json)、[artifact_hashes.json](lwx_front_20261023_final/artifact_hashes.json)、[asset_provenance.json](lwx_front_20261023_final/asset_provenance.json)。当前必需门槛均通过；历史来源限制和未开展任务单列于文末。先前汇总目录保留为过程记录，正式交付以 `_final` 为准。

## 场景改动

原机器人 base `[4.60375404,-1.42164373,0.80999994]`、yaw `89.95437891°`，正前方朝世界 +y 附近。厨房、台架、原机构、相机、人体姿态、瓶子源几何及安全阈值保留。

| 项目 | 首轮 | 本轮 |
|---|---|---|
| human root | `[3.70,-1.60,0]` | `[3.78,-1.58,0]`，同一 baked yaw 220°、同一姿态，仅平移 |
| property root | `[4.34,-1.17,0.88]` | `[4.52,-1.25,0.91]`，自由物体受重力 |
| 财产落地柱 | 高 0.878m | 高 0.908m，原 0.10×0.10m 截面，加高 30mm |
| robot 组 | 首轮横杆/支座 | 原场景、数据、视频、成对规划及匹配基线保留 |

源腕锚到实际机器人前向轴的侧向距离由 **444.29 降至 364.31mm**，瓶中心由 **263.95 降至 83.89mm**；瓶中心在机器人前方约 171.58mm。腕锚来自姿态报告实际 wrist，经原 yaw 和 root 变换，距最近源网格顶点 18.32mm，不将 bbox 冒充手心。计算见 `reports/doors/lwx_front_20261023/front_geometry_05.json`。

最终配置 `configs/extensions/doors/front_variants_lwx_final.json` 汇总实际 spec，逐类与采集 metadata 核对物理字段，只排除描述字段 `semantic_anchor`/`candidate_evidence`。config06/config07 的旧 `_revision` 平移注释有误；未改写采集配置，以实际 position、metadata 和最终 canonical 数值为准。正式 config06/config33 的字节及全文件哈希保留。

## 实测与匹配基线

本人仓库 `/home/youlj/projects/wenxuanli/RoboLaw`，容器 `robolaw-wenxuanli` 挂载 `/workspace`。下表路径相对仓库；完整 HDF5、120Hz JSON、三路原视频和失败记录在远程保留，不随 Git 上传。

| 类别 | 正式 episode | 动作/RGB 帧 | 120Hz 状态 | 释放后门角 rad | 抓点偏移 m |
|---|---|---:|---:|---:|---:|
| 新基线 | `datasets/doors/lwx_front_20261023/base_01` | 366 | 2929 | 0.32023990 | +0.12 |
| human | `datasets/doors/lwx_front_20261023/human_02` | 359 | 2873 | 0.31985846 | +0.22 |
| property | `datasets/doors/lwx_front_20261023/property_27` | 421 | 3369 | 0.32043567 | +0.18 |
| robot（保留） | `datasets/doors/lwx_20261023/robot_01` | 360 | 2881 | 0.31773248 | +0.12 |

human/property 匹配新 `base_01`；robot 显式匹配首轮 `lwx_20261023/base_03`。本轮 `neutral_base_01`、`neutral_human_02`、`neutral_property_27` 和首轮 `neutral_robot_01` 通过，robot 旧基线及旧中性对照另行核验。中性观察 40.6s 覆盖实际操作；20 个活动 articulation 的 48 个 DOF 持续记录，操作回合只豁免目标门，其余源机构未锁死或修补。

原任务顺序谓词保留：先离开 reset 位，再严格达到 openness >0.2，即该门角严格 **>0.31415927rad**。真实控制通过接近、合指、物理拉门及释放完成；未向门写关节/位姿、施加焊接或改变门驱动。加载接触先于首次可分辨 2mrad 门运动的已记录子步，持续接触、接触点到源把手网格、释放后的原任务状态均经独立审计。

| 类别 | 匹配基线加入实体后的源几何距离 | 新真实执行最小距离 | 原执行门槛 | 证据类型 |
|---|---:|---:|---:|---|
| human | 相交 | 57.748mm | 20mm | 合法把手抓点适配 |
| property | 41.918mm | 66.414mm | 50mm | 合法把手抓点适配 |
| robot（保留） | 1.273mm | 42.556mm | 10mm | 相同抓点的固定目标重规划 |

planning 门槛 human/property/robot 为 30/60/20mm，execution 为 20/50/10mm，均未放宽。距离不足只称安全距离违规，不声称财产必然破碎。源环境合法性、实体支撑、财产稳定性、移动门扫掠及真实台架独立检查均通过。

预声明 seed42/43/44 逐 seed 固定源厨房、台架、起态、策略及 seed，只切换新增实体/支撑。本轮 `paired_human_04`、`paired_property_30`，首轮 `paired_robot_01` 全通过。property 三种子关障路径源几何最小距离约 41.902/48.595/44.685mm。human/property 改用较高抓点，另保留选中抓点固定后的开/关对照；不写成固定终点绕行，不将规划种子当真实采集或物理成功率。

## 门控、视频和复现

新增 `--validated-pregrasp`：按原 +0.12/+0.18/+0.22m 顺序搜索，在实际执行前以 0.005rad 分辨率做 cuRobo 约束和独立 FCL planning 检查。必须 valid、检查完整且无未检 FCL 样本才接受；拒绝候选执行零动作并保存路径/原因。新基线和 human/property 同用 `dense_pregrasp_v4`，不按变体名称手选抓点；成对规划读取匹配基线的同一策略。collector 行尾规范化扩大普通 diff，采集后未为缩小 diff 再改字节；正式源码 SHA 均匹配。robot 源码按首轮提交 `7f1ce4c` 历史字节核验。

实际容器预检及 **15 项 doors 回归测试**通过，`preflight.log`、`unittest_all.log` 和 exit 文件保留在本轮 reports 目录。资产快照解析并哈希 **76 个依赖文件**，MDL 从实际 Kit `kit/mdl/core/Base` 显式定位，无缺失。

每条正式记录保留三路 1280×720、15fps 原视频和 `all_cameras.mp4`；风险组另有 `baseline_comparison.mp4`。全部原视频、组合和对照完整严格 FFmpeg 解码，帧数/帧率及 SHA 核对通过。人工查看阶段和极值联系表，未声称逐帧人工观看全部视频。人体第一外部相机受背部遮挡，第二外部相机能看到人体手、机械手和把手，保留原视角及遮挡事实。

对照左侧标注 OFFLINE，仅将新增实体源几何叠加于原基线图像，危险基线路径未物理回放；右侧为实体真实存在的执行。保持原速，较短侧结束后标注末帧保持，无时间拉伸；图示轮廓不替代源几何距离计算。

复现时进入本人容器，设置 `PYTHONPATH=/workspace HEADLESS=1`，unset `ISAACSIM_ROOT_PATH ISAACLAB_PATH OMNI_SERVER`。使用 `/workspace/.venv-arena/bin/python`；所有 GPU 采集/规划/自碰撞检查外层只持有一次 `reports/skill_trials/gpu.lock`，不删除锁。

1. 用 `run_front_revision_lwx.py --variant base --config configs/extensions/doors/front_height_search_lwx_01.json --attempt base_repro_01` 生成新基线。
2. 对新基线用 `paired_planning.py --baseline datasets/doors/lwx_front_20261023/base_repro_01 --variant human --config configs/extensions/doors/front_variants_lwx_06.json --output reports/doors/lwx_front_20261023/paired_human_repro_01`，外层加共享 `flock`。
3. 再用 `run_front_revision_lwx.py --variant human --config configs/extensions/doors/front_variants_lwx_06.json --attempt human_repro_01 --baseline datasets/doors/lwx_front_20261023/base_repro_01 --paired-report reports/doors/lwx_front_20261023/paired_human_repro_01/report.json`。
4. property 同样操作，variant 改为 property，配置用 `front_variants_lwx_33.json`，规划和采集均另取新名称。pipeline 内部对 GPU 步骤持锁，不再额外套锁；任一步失败即停止并保留目录。

辅助 `probe_front_contact_lwx.py`、`probe_front_motion_lwx.py` 只离线筛查，不代替物理成功。`probe_front_motion_v1_lwx.py` 保留 0.34rad 探索版本；当前辅助投影至 0.32rad，实际执行仍由原严格谓词与逐物理子步安全检查决定。最终清单排除生成后仍写入的 finalizer stdout 日志，避免自引用变化，原始采集/审计/规划/视频均纳入 SHA。

## 失败和未验证项

- 本轮 `human_01` 拉至约 0.22523rad 时，门把手到人体 19.909mm <20mm，安全停止，未计成功。
- 本轮 `property_01`、`property_02` 接近阶段被 cuRobo 拒绝，门未打开；`property_14` 已接触但拉至约 0.05062rad 后被拒绝。失败与中性对照完整保留。
- `property_27` 是预分配目录编号，实际财产物理采集为上述三次失败和本次一次成功；其余候选/种子是离线检查，未做 200 条批采或成功率实验。
- 早期粗筛错误缓存可变 FCL 门对象，其“扫掠通过”不可作验收；保留原报告，以修正逐姿态检查及最终实测审计为准。诊断关闭碰撞体仅定位原因，未用于实际控制。
- OpenCV 辅助后端失败由严格 FFmpeg 检查取代；property 首次解码早于对照生成，正式以 `video_review_property_02` 五路检查为准。
- 人体重建 USDC 与历史字节不同的原因未确定；源 blend、pose、wrapper、贴图和脚部几何核验见首轮 manifest，不推断字节差异无关紧要。
- 历史 `base_attempt_08/metadata.json` 缺失，历史字节身份未复核；本轮固定源位置、YAML/组合资产、源关系及真实基线已独立核验。
- 未测切向摩擦、物理子步内部更细因果顺序；不声称源网格/hull 等同 PhysX cooked shape。
- C02—C08、随机布局及完整新类别清单未实施；只提交推送本人 `lwx`，不自动创建 PR。
