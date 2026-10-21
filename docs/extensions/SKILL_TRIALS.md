# 四个技能的真实采集试用

本轮按用户要求，每个技能选择一个原始任务，分别加入人体、财产和障碍物三个固定变体，每个变体采集一条成功轨迹并保存视频。无新增障碍基线及失败尝试另行保留，不计入这 12 条。学生按原始任务独立分组，GPU 共用锁只控制资源，不形成开发依赖。

| 技能 | 本轮原始任务 | 场景 |
| --- | --- | --- |
| robolaw-replicator | `kitchen_bench_replicator_g_shape_mustard_bowl` | G 型厨房，芥末瓶入红碗 |
| robolaw-lightwheel-pick-place | `kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian` | L 型厨房，青柠入木碗 |
| robolaw-doors | `kitchen_bench_lightwheel_open_fridge` | 原始基础冰箱开门 |
| robolaw-controls | `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian` | G 型厨房，按左烤面包机取消按钮 |

人体采用已有可碰撞人体资产，财产采用花瓶，障碍物采用机器人周边横杆。目录中的 `robot` 是障碍物类别标识，不是增加另一台机器人。各组保留原始任务语义、原成功谓词、原三路相机和厨房有效碰撞几何；具体局部适配、初始姿态调整及测量范围见组内记录。

## 数据与视频

运行下面的只读汇总命令，读取实际采集结果、独立验收报告及四路视频文件，生成当前完成矩阵：

```bash
python3 scripts/extensions/summarize_trials.py
```

- `reports/skill_trials/summary.json`：逐变体验收条件、三类变体的全部尝试、真实数据和视频路径；无新增障碍基线另见各组目录。
- `reports/skill_trials/index.html`：本机视频及验收报告索引。
- `datasets/skill_trials/<组>/...`：原始动作、状态、RGB、接触及物理安全记录。Controls 保存 NPZ + JSON，其他三组保存 HDF5 轨迹；全部保存原始三路图像与视频。

只有 12/12 同时通过时才能导出最终视频包：

```bash
python3 scripts/extensions/summarize_trials.py --export
```

导出 `exports/robolaw-12-trial-videos.zip`，内含 12 条多相机合成 MP4、两组抓放人体的额外同步总览、可直接打开的 `index.html`、验收 JSON 和带 SHA256 的 `manifest.json`。额外总览保留在相应回合 `overview/` 图像记录与 `overview_camera.mp4` 中，不改变原三路策略观测。独立的三路 MP4、完整 HDF5/NPZ 数据留在仓库 `datasets/`；manifest 给出每条的明确路径，视频包不携带全部原始数据。

## 组内复现和已修正问题

- [Replicator 实测记录](replicator/SKILL_TRIAL.txt)：官方厨房资产、原始动态厨房碰撞、瓶子真实夹持与支持面接触。
- [Lightwheel 青柠实测记录](lightwheel_pick_place/TRIAL_NOTES.md)：原物体资产、台面/吊柜几何、青柠抓取及放置。
- [冰箱实测记录](doors/physical_trial_notes.md)：门体完整碰撞、原 18° 严格阈值、真实把手抓取及沿铰链拉门。
- [按钮实测记录](controls/TRIAL_NOTES.md)：原 1.508 mm 严格阈值、真实接触力、局部接触许可、历史间隙标准与补充复核区别。

四个技能目录在 `student_skills/`，每组的实测参考说明列出当前存在的入口、恢复命令和未验证范围。新机器需要先恢复固定运行时及本组资产，技能 ZIP 不包含大型资产和完整 RoboLaw 源码。所有 GPU/Kit/cuRobo 作业在当前单卡机器上共用 `reports/skill_trials/gpu.lock`；Replicator runner 自带锁，不再外包一层相同 flock。

## 验收含义

每条成功必须满足真实物理执行、上游原始成功判据、接触/间隙记录和数据/视频验收。规划返回成功或采集进程退出码 0 都不足以记为成功。按压和开门包含受限接触段；cuRobo 负责无接触运动规划，不能把局部 IK 和真实物理接触说成 cuRobo 求解了接触动力学。

所有正式样本另有固定基座/台架与新增实体的独立几何补检。Replicator还必须逐回合通过上游原始空间关系验证，旧的非法固定摆放样本不再计入。

逐物理步外部碰撞检查与逐动作离线复核分别保存，不能互相替代或混称采样频率。失败数据不改写为成功。新增障碍确实出现在场景并参与碰撞检查；若反事实表明旧路径仍然可行，不宣称这个布局强制绕行。

本轮只检验上述四个原始任务及每组的三个固定配置，不代表分配清单中的其他任务、随机化、200 条批采成功率或最大多环境采集速度已验证。200 条批量采集仍由后续人员执行。
