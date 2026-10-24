# C01 项目要求交付：人体与财产摆放修复，原钢架保留

按 student_skills/robolaw-doors/references/workflow.md 及 review-gates.md 的项目要求完成四条选中数据及各自匹配中性/基线。项目原文允许无障碍原轨迹违反原安全间隙或真实相交；安全间隙违规不等于必然物理碰撞。本包 acceptance_profile 明确为 project_workflow_original_clearance_or_intersection，额外严格筛选 additional_strict_screen_passed=false。原严格 finalizer 未修改，不能把本包说成该脚本通过。

human87：592帧，property86：464帧；均使用实际成功438帧的较低合法抓点基线70。人体/财产确实要求原路线调整，三seed原几何相交、小幅绕行、原50mm或人体原门槛、实际接触开门、五项独立审计、中性机构、原速同相机视频和原生/风险画面复核均通过。不是将旧两条无阻挡样本重新标为通过。

robot保留用户认可的原钢架 robot_01：360帧，原匹配 base_03：366帧；真实开门成功，原安全、机构/接触因果、接触源几何、自碰撞、避障审计通过。旧审计文件名为 validation_full.json、review_audit.json、contact_geometry_audit.json；不是缺少审计。新打包器仅在明确 C01 项目交付中识别该旧 schema，不改原审计内容。人体/财产不能错配到钢架基线，钢架也不能错配到基线70。

钢架三个规划seed的原几何最小间隙为1.198/3.144/1.875mm，低于原10mm，但均未实际相交。TCP增加约.315-.320m，关节路径比约1.083/1.132/1.111。额外本地相交/15cm筛选不满足，明确保留，不把该结果称为开门失败。原基线/钢架各15个原生相机面板及最小间隙原速对照均实际复看：4.20秒原基线1.273mm/10mm，实际同一对照时刻83.6mm/10mm；83.6mm不被称为全程最小值。左侧障碍是离线反事实轮廓，未执行危险路径。

599个原文件hash、72份证据副本和14项资产hash已逐项复核。原始轨迹、物理子步、接触、视频仍在远程 datasets/doors/，selection.json 固定精确选择与审计schema。本包将历史无源码副本、源几何与PhysX cooked shape差异、子步内先后及切向力未测等局限保留在 gate_summary.json，不回填不存在的历史证据。单布局各一条成功数据和三个规划seed不外推为物理成功率。

完整当前CPU套件83项通过；项目 finalizer 另验证已交付C06的既有episode字段不变，并拒绝将C01旧schema用于C06。所有失败和原数据保留，不做200条批采。
