# C07 项目要求交付：G型厨房微波炉

按项目workflow/review-gates验收，匹配基线及人体、财产、障碍三变体各1条真实成功采集，fixed_only。完整整体验收与打包通过；873个原文件hash、83份小证据副本、14项资产hash，当前83项CPU测试通过。原轨迹、物理子步、接触、三路RGB和原速视频保留在远程datasets/doors/，精确选择见selection.json。

|变体|实际选中数据|帧数|匹配基线|最终门角rad|
|---|---|---:|---|---:|
|base|grip_roll_retry140/C07_base_01|1015|自身|-.707044|
|human|middle_release_retry103/C07_human_71|992|retry103/C07_base_01|-.713269|
|property|grip_roll_retry140/C07_property_175|1017|retry140/C07_base_01|-.709204|
|robot|release_clearance_retry120/C07_robot_72|987|retry120/C07_base_01|-.715711|

四条及各自中性对照通过原任务成功、全量验证、独立机构/接触因果、源接触几何和自碰撞五门槛；三变体各有原几何避障审计、同目标同控制参数三seed成对规划、原速同相机对照视频及实际raw/risk关键帧复核。不同控制参数的human/robot/property保留各自匹配基线，不交叉错配。门由机器人真实接触打开，释放后保持开启，不写门关节伪造成功。

human71和robot72在三seed关障规划中均有源几何相交；property175的三个seed为原50mm源安全间隙违规，均未实际相交。property三seed关节路径比1.23347/1.27938/1.23241，末端路程差-.062865/-.052315/-.047701m。绕行是关节/冗余姿态变化，不保证TCP路程增加。原目标固定，未手动添加不必要绕路，原保守执行间隙未降低。

property175真实全程源几何审计最小间隙108.307mm，对比原50mm；源重建选中基线frame86为39.505mm，实际同归一化接近进度frame117为118.004mm。118.004mm是选中帧间隙，不能称全程最小。原酒杯不缩放，偏置托架和支柱具有真实碰撞并纳入规划/审计。危险关障路径仅离线检查，从未物理回放。

acceptance_profile=project_workflow_original_clearance_or_intersection；additional_strict_screen_passed=false。原严格finalizer保持不变，真实负例仍拒绝property175的无相交证据；拒绝日志保留在validation中，不把项目通过冒充额外严格筛选通过。

已实际查看property的15个原生相机面板：接近、抓把手、逐步开门和释放终态可见，酒杯本体在取样视角未清楚显示，支柱可见。human/robot也有原生障碍可见性限制。三组分别附明确标记的源几何重建；图是补充说明，不能替代原始RGB、FCL审计、物理记录或连续间隙证据。黄色AABB加间隙仅为保守示意，不是准确安全外壳。

兼容验证183与修改前紧邻版本比较：默认严格模式图像逐字节一致，报告字段一致；默认模式仍拒绝margin-only property。182误选更早历史图造成对比失败，原日志保留，未将该失败标成通过，也未更改已交付历史图。

此前property138的980帧IK边界失败及所有其他拒绝保留，未当作成功；不做200条批采。固定布局单样本和三个规划seed不能外推物理成功率。源碰撞网格/FCL不声称等同PhysX cooked shape；切向接触力和一个物理子步内部的严格先后仍未测，详见gate_summary.json。
