# 左取消按钮：机构与避障修复后的实测

原任务仍是 `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian`：机器人按烤面包机左侧取消按钮，原严格成功阈值为 1.508303 mm。新人体、花瓶、横杆各完成一条真实采集；原三路视频、同步物理子步接触和逐步安全记录均保存。旧三条异常样本保持撤回。

## 改动及依据

- 被动旋钮通过局部质量／角阻尼覆盖修复，原按钮与原场景几何未修改。30 秒无接触静置和外力矩可动性对照通过；没有锁死关节或逐步写角度。见 [机构修复](PASSIVE_KNOB_REPAIR.md)。
- 原人体粗网格和旧花瓶搜索没有约束短路线，保留拒绝。现人体从机器人右侧合法站立、抬肘伸手；花瓶位于原台面前缘；横杆保持原台面锚固支撑。人体骨长、根节点及脚部保持原源模型，独立资产清单绑定人物几何和纹理。
- 采用同一个固定冗余 IK 分支、原 TCP 目标的 cuRobo joint-goal 策略，没有新增手工路点。原长达 0.829 m 的绕路基线不再使用。新实际无障碍接近路径约 0.278 m；各变体真实动作会改变机械臂的接近方向和姿态。

## 真实采集结果

最终基线为 `datasets/avoidance_trials/controls/base_repaired02`。下表旧路径是这条新基线放入新增实体后的 CPU 几何反事实，未执行危险碰撞。旧路径距离使用源障碍形状与机器人／逐指碰撞 hull，去掉冗余胶囊；新路径列保留原保守执行模型的间距。

| 变体 | 新回合 | 帧数 | 旧路径源形状最近距离 | 原安全距离 | 新实际路径最近距离 | 实际接近路径 |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| 人体 | `human_repaired01` | 279 | 相交 | 20 mm | 32.15 mm | 0.936 m |
| 花瓶 | `property_repaired01` | 361 | 40.52 mm | 50 mm | 101.05 mm | 1.338 m |
| 横杆 | `robot_repaired01` | 319 | 相交 | 10 mm | 52.72 mm | 0.637 m |

三条分别通过原按钮成功、120 Hz 同步接触／关节因果、所有未操作关节静置、原三路数据与视频解码、每个物理更新的外部安全、独立关节限位／自碰撞、固定台架、原安全距离及新增实体合法支撑检查。按钮最大位移约 2.03 mm；真实负载接触先于成功。没有修改执行对象状态来制造成功。

花瓶属于 **保护距离不足后的避让**，不能描述为原路径必撞、玻璃必碎。花瓶静置零位移，角度变化约 0.000016°。原审计因源花瓶底环与台面约 40 纳米的数值重叠拒绝；保留该报告，核实源 `Wall00_*` 底面本来为 z=0 后，仅为该底环／原台面支撑对采用 1 微米容差。上层侧壁、其他环境接触、机器人 50 mm 距离均未放宽。

## 三个规划 seed 的配对

`paired_final_02` 使用新基线实际接近起始 q，固定 seed 42、43、44；每个 seed 仅改变新增实体是否进入规划碰撞世界，起点、同一关节目标、代价／控制参数保持一致。三条关障基线与九条有障路径全部通过密集 cuRobo／FCL 路径检查；每个关障基线对人体／横杆均存在源形状相交，对花瓶约 34.4–35.1 mm，低于 50 mm。

有障规划路径分别约为人体 1.000 m、花瓶 1.280 m、横杆 0.676 m，无障碍约 0.278 m。它们是同策略求得的可行路径，**没有证明全局最短**。三 seed 中存在重复路径，不能把它们当作独立随机场景或统计成功率。配对使用名义出生位置的静态几何；花瓶真实采集会下落约 1 mm 落稳，真实轨迹审计另行使用其实测位姿。

[配对完整报告](../../../reports/repair_20261022/controls/paired_final_02/paired_planning.json) · [独立重算起终点、速度与源几何](../../../reports/repair_20261022/controls/paired_final_02/independent_cpu_review.json) · [人体引用资产逐项哈希核验](../../../reports/repair_20261022/controls/paired_final_02/asset_manifest_verification.json) · [最终机器可读结果](repaired_left_button_results.json)

## 视频与复现

对照视频使用原第一路外部相机，原速并排展示无新增实体基线和新真实执行。第二路外部相机在人体变体被手臂遮挡，仍完整保留，没有移动源相机或隐藏人体。红色间距曲线是 CPU 反事实，不是危险碰撞录像。

- [人体对照](../../../datasets/avoidance_trials/controls/human_repaired01/comparison.mp4)
- [花瓶对照](../../../datasets/avoidance_trials/controls/property_repaired01/comparison.mp4)
- [横杆对照](../../../datasets/avoidance_trials/controls/robot_repaired01/comparison.mp4)

使用 `configs/extensions/controls/repaired_left_button.json`，每次换新输出目录。按技能 [修复复现步骤](../../../student_skills/robolaw-controls/references/repaired-left-button.md) 先做中性试验、无障碍真实基线、三类变体，再执行独立审计及多 seed 配对。原始任务右按钮、温度旋钮、其他布局、动态人体和批量采集成功率均未由本轮验证。
