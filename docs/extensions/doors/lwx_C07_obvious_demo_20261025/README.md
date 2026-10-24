# C07 钢架避障试验版（2026-10-25）

本次只试 C07：G 型厨房的微波炉开门，先做钢架这一种。原 C01–C08 版本、原 C07 人体/财产/钢架的配置和数据均保留。其他变体和任务等用户看完本版确认后再改。

先看 `C07_approach_compare_first9s.mp4`：左边无障碍基线，右边新钢架；同一原相机、原速，前9秒集中展示避让。完整开门可看 `C07_full_baseline_comparison_external2.mp4`，三路原始合成见 `C07_robot_pilot13_all_cameras.mp4`。`native_pregrasp_comparison.jpg` 是记录RGB的抽帧对照，不是生成图。

实测接近阶段夹爪路径从46.05厘米增至66.57厘米，增加20.52厘米；相对原基线折线的最大距离为9.85厘米。动作是先向侧下方避开钢架，再回到原抓点开门。两路原外部相机中的最大路径投影差约61和87像素（原1280×720分辨率）。这不是大幅翻越动作，是否达到想要的明显程度仍待用户确认。

机器人真实接触把手开门并完成释放，最终门角约37.4°。原任务完成、完整验证、机构静置/接触因果复查、源接触定位、自碰撞/关节限位、实际间隙及避障对照均通过。实采1060帧、8481个物理采样、9540次安全检查。中性保持覆盖约73秒，机器人不操作时门没有自行打开。

同初态、同抓点、同控制参数、同规划seed，仅切换新增钢架和支撑。42/43/44三组规划对照中，关障路径与新增钢架的源几何实际相交，开障规划安全。危险反事实仅离线检查，未带障碍物物理回放；未人工添加无必要的绕行路点。钢架使用未缩放的原库 projecting_crossbar.usda；原三路相机保持不变。

严格的“每个规划seed最大偏离都至少10厘米”的发现筛选没有全部满足（约9.52/10.05/9.91厘米），失败结果保留。此版在全部安全条件通过后仅做一次物理试验，供用户通过视频判断效果；未把自动筛选失败改写为通过，未降低安全阈值。

限制：外部相机仍裁切部分上臂；源几何审计不等同于PhysX cooked shape逐面身份验证；仅fixed_only单条成功样本，不能外推随机布局或批量成功率。其他任务尚未按这轮“直观看出明显避障”的标准重新验证。

逐项结果、远程路径和SHA-256见 `gate_summary.json`；摆位和支撑见 `selected_config.json`；实测路径见 `actual_detour_metrics.json`。

远程数据：/home/youlj/projects/wenxuanli/RoboLaw/datasets/doors/lwx_C07_obvious_demo_20261025/C07_robot_pilot13/

远程报告：/home/youlj/projects/wenxuanli/RoboLaw/reports/doors/lwx_C07_obvious_demo_20261025/

复现：使用 configs/extensions/doors/c07_robot_candidate_obvious13_11.json 与原匹配基线先运行 paired_tasks_lwx.py，再用新的 attempt 和新报告运行 supervise_c07_physical_pilot_lwx.py。入口检查来源、参数、三seed源几何相交及安全、接近/拉门筛选，然后依次采集、中性保持和全审计，拒绝覆盖旧样本。原始视频和数据不进Git；分支中的文档/配置记录对应路径和哈希。
