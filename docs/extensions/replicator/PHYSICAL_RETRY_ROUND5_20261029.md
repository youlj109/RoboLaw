# Replicator 154 景：第五轮物理重试结果

**此前149/154通过。本轮5个候选：新增通过2景，物理失败0景，规划失败3景，执行异常0景。最新151/154通过，剩余3景。**

[机器可读状态与SHA](../../../configs/extensions/replicator/physical_retry_results_round5_20261029.json) · [保留的第四轮重试结果](PHYSICAL_RETRY_ROUND4_20261029.md) · [冻结调整计划](../../../configs/extensions/replicator/physical_retry_round5_20261029.json)

本轮只调整抓点、抓姿、夹爪指令或轨迹目标，场景布局及原任务判据不变。每景仅冻结一个候选；规划失败的3景没有新物理执行。其余候选各消耗一次物理尝试，共2次。旧成功、旧失败及retry、retry_round2、retry_round3、retry_round4记录均保留。

实际执行关闭渲染、相机和视频。本轮没有实际执行失败，无需生成失败诊断图或进行图像复核；规划失败仍沿用上一轮实际物理失败记录。 任务通过不等于避障、安全验收或长期稳定通过；有限次失败也不证明场景无解。

## 14 × 11 最新状态矩阵

初✓＝首轮通过；一✓＝第一轮重试通过；二✓＝第二轮重试通过；三✓＝第三轮重试通过；四✓＝第四轮重试通过；五✓＝本轮通过。规划失败＝本轮没有新物理测试。

| 原始任务 | 人体 | 高水杯 | 高脚酒杯 | 玻璃瓶 | 陶瓷杯 | 花瓶 | 平板支架 | 钢夹 | 突出横杆 | 窄口 | 尖角外壳 | 通过 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| G 型：香蕉放碗 | 一✓ | 初✓ | 一✓ | 二✓ | 一✓ | 一✓ | 一✓ | 一✓ | 一✓ | 初✓ | 初✓ | 11/11 |
| G 型：碗放水槽 | 二✓ | 二✓ | 初✓ | 二✓ | 初✓ | 初✓ | 二✓ | 初✓ | 二✓ | 二✓ | 三✓ | 11/11 |
| G 型：芥末瓶放碗 | 二✓ | 初✓ | 二✓ | 一✓ | 二✓ | 二✓ | 二✓ | 三✓ | 一✓ | 一✓ | 二✓ | 11/11 |
| L 型带中岛：香蕉放碗 | 初✓ | 初✓ | 初✓ | 初✓ | 一✓ | 一✓ | 初✓ | 一✓ | 三✓ | 初✓ | 规划失败 | 10/11 |
| L 型带中岛：碗放水槽 | 三✓ | 三✓ | 一✓ | 初✓ | 初✓ | 初✓ | 三✓ | 一✓ | 三✓ | 三✓ | 初✓ | 11/11 |
| L 型带中岛：芥末瓶放碗 | 一✓ | 初✓ | 初✓ | 初✓ | 一✓ | 初✓ | 初✓ | 一✓ | 初✓ | 初✓ | 初✓ | 11/11 |
| L 型：香蕉放碗 | 三✓ | 初✓ | 一✓ | 一✓ | 初✓ | 初✓ | 一✓ | 初✓ | 一✓ | 初✓ | 初✓ | 11/11 |
| L 型：碗放水槽 | 四✓ | 二✓ | 五✓ | 三✓ | 初✓ | 二✓ | 三✓ | 二✓ | 三✓ | 五✓ | 初✓ | 11/11 |
| L 型：芥末瓶放碗 | 一✓ | 三✓ | 一✓ | 一✓ | 一✓ | 一✓ | 三✓ | 初✓ | 一✓ | 初✓ | 一✓ | 11/11 |
| 半岛式：碗放水槽 | 三✓ | 初✓ | 三✓ | 初✓ | 初✓ | 初✓ | 三✓ | 初✓ | 三✓ | 初✓ | 初✓ | 11/11 |
| 半岛式：芥末瓶放碗 | 初✓ | 一✓ | 初✓ | 三✓ | 三✓ | 初✓ | 初✓ | 一✓ | 三✓ | 初✓ | 一✓ | 11/11 |
| U 型：香蕉放碗 | 三✓ | 初✓ | 初✓ | 一✓ | 初✓ | 一✓ | 初✓ | 一✓ | 初✓ | 初✓ | 初✓ | 11/11 |
| U 型：碗放水槽 | 规划失败 | 初✓ | 初✓ | 初✓ | 初✓ | 初✓ | 二✓ | 二✓ | 二✓ | 初✓ | 初✓ | 10/11 |
| U 型：芥末瓶放碗 | 初✓ | 初✓ | 初✓ | 初✓ | 初✓ | 初✓ | 初✓ | 三✓ | 一✓ | 初✓ | 规划失败 | 10/11 |

## 本轮5景：调整、结果和后续建议

| 场景 ID | 本轮调整（冻结时说明） | 规划 / 新物理尝试 | 结果 | 失败后候选建议 |
| --- | --- | --- | --- | --- |
| `l_island_banana_bowl__robot_sharp_equipment_housing` | 保留已通过源几何筛选的避壳折线，首段缩半，强姿态代价20且禁用graph。 选中same_clear_housing_route_half_first_step_strong_orientation_no_graph。 | failed / 0次 | 本轮规划失败，零次新物理执行：cuRobo rejected lift: MotionGenStatus.FINETUNE_TRAJOPT_FAIL。lift第1段唯一一次调用返回MotionGenStatus.FINETUNE_TRAJOPT_FAIL、solver_success=false、samples=0；没有返回或保存该失败段轨迹。；冻结选项为constraint_weight=20、enable_graph=false；记录完整seed重置且无graph回退。；仅保存此前成功的pregrasp、approach前缀，共78个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；没有对不存在的失败段计算FK偏差、碰撞或IK诊断；本轮没有该景的新物理结果。。当前单候选的轨迹优化精调未成功。已有前缀只证明之前阶段生成了路径，不能说明失败段的碰撞位置、反向超调、IK不可达或全局不可行；仅凭状态也不能确定是强约束、关节分支还是数值问题。 | 下一轮保留远离外壳的方向和5mm gate，对首个低位斜移段预先冻结从approach末关节状态连续展开的关节种子，再单次测试；把姿态权重作为提前声明的优化参数，不能在失败后加次数或放宽gate。 |
| `l_shape_bowl_sink__property_wine_glass` | 保留round4已有效的抓持，名义落点相对原目标偏移(-55,-20)mm，按实测夹持变换扶正一半并偏航-15度；释放源碗底高于槽底16mm以保持腕部高度，预测倾角9.78度。偏心落点的源排水口净空与完全开爪/200mm竖直退出通过；CPU lower分支仍有2对link5世界碰撞，明确带警告冻结。 | generated / 1次 | 本轮通过 | — |
| `l_shape_bowl_sink__robot_narrow_opening` | 保留round4抓持与释放倾角，释放转腕+90度，名义落点相对原目标偏移(-55,-20)mm，将真实夹持偏移转向远离排水口/前壁的区域；预测源碗底距槽底3mm，完全开爪后200mm竖直退出。6端点、源整碗、展开与退出净空CPU通过，但57.78度倾倒稳定性未经物理验证。 | generated / 1次 | 本轮通过 | — |
| `u_shape_bowl_sink__human_default_mpfb` | 更换碗缘方位、略降抓取点，先短抬升再远离人体；根据新抓取attachment重算原同水槽放置补偿目标。 选中rim_delta30_tilt_delta-10_z-0.003_away_human。 | failed / 0次 | 本轮规划失败，零次新物理执行：cuRobo rejected lift: TCP_SEGMENT_DEVIATION_LIMIT。cuRobo在lift第1段返回solver_success=true，保存39个关节样本。；独立原机器人USD FK重算最大TCP有限线段偏差5.748088mm，超过冻结5.000mm；保留被拒路径，后续段没有调用。；仅保存此前成功的pregrasp、approach前缀，共78个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；本轮没有完整可执行轨迹，也没有该景的新物理结果。。这一次候选在首个10mm上抬段未通过路径限制；权重20及禁用graph没有保证TCP直线。该结果不是物理抓持失败，也未证明场景或IK无解。后续下放未规划，旧抓姿的测量补偿参考不能当作新抓姿已经验证的下放证据。 | 下一轮把首个竖直10mm抬升明确单独设为固定XY的短段约束，后续远离人体的斜向段保留姿态约束；预先冻结连续关节种子及5mm验收界限。新抓姿的下放补偿需重新绑定适用的attachment依据；本轮不追加规划或物理测试。 |
| `u_shape_mustard_bowl__robot_sharp_equipment_housing` | 保留已通过源闭合筛选的瓶身抓取，预抓缩至60mm并按10mm下降，强姿态/XY代价20且禁用graph，禁止首段反向超调超过原5mm。 选中same_source_body_grasp_shorter_60mm_pregrasp_10mm_descent_strong_cost_no_graph。 | failed / 0次 | 本轮规划失败，零次新物理执行：cuRobo rejected approach: MotionGenStatus.FINETUNE_TRAJOPT_FAIL。approach第1段唯一一次调用返回MotionGenStatus.FINETUNE_TRAJOPT_FAIL、solver_success=false、samples=0；没有返回或保存该失败段轨迹。；冻结选项为constraint_weight=20、enable_graph=false；记录完整seed重置且无graph回退。；仅保存此前成功的pregrasp前缀，共39个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；没有对不存在的失败段计算FK偏差、碰撞或IK诊断；本轮没有该景的新物理结果。。当前单候选的轨迹优化精调未成功。已有前缀只证明之前阶段生成了路径，不能说明失败段的碰撞位置、反向超调、IK不可达或全局不可行；仅凭状态也不能确定是强约束、关节分支还是数值问题。 | 下一轮联合调整预抓关节分支与首个10mm下降段的种子，使接近从前一段末关节状态连续开始；预先固定姿态/XY权重及5mm gate后再单次测试，不把本次精调失败解释为接触或抓持失败。 |

## 3景规划失败的只读CPU诊断

1景由源机器人关节变换独立重算已保存规划段的TCP路径，确认超过冻结的偏差上限。cuRobo自身返回成功，但候选未通过额外路径检查，因此没有执行；未追加IK求解或cuRobo重试。 2景的求解器未返回失败段轨迹，仅核对保留的求解状态与既有前序路径；没有虚构失败段FK，也没有追加CPU IK。求解失败不代表场景无解。

这些只读诊断没有新增cuRobo调用、抓姿候选或物理测试；仅解释保留的单次候选，不证明场景不可行、真实接触或安全验收。

[状态JSON内嵌的完整规划诊断与候选结果SHA](../../../configs/extensions/replicator/physical_retry_results_round5_20261029.json)。原始本地诊断报告路径和SHA同时保留在JSON中。

| 场景 ID | 失败阶段 / 原求解器状态 | CPU可核实事实 | 保守推断 | 下一次调整建议（尚未实施） |
| --- | --- | --- | --- | --- |
| `l_island_banana_bowl__robot_sharp_equipment_housing` | lift / MotionGenStatus.FINETUNE_TRAJOPT_FAIL | lift第1段唯一一次调用返回MotionGenStatus.FINETUNE_TRAJOPT_FAIL、solver_success=false、samples=0；没有返回或保存该失败段轨迹。；冻结选项为constraint_weight=20、enable_graph=false；记录完整seed重置且无graph回退。；仅保存此前成功的pregrasp、approach前缀，共78个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；没有对不存在的失败段计算FK偏差、碰撞或IK诊断；本轮没有该景的新物理结果。 | 当前单候选的轨迹优化精调未成功。已有前缀只证明之前阶段生成了路径，不能说明失败段的碰撞位置、反向超调、IK不可达或全局不可行；仅凭状态也不能确定是强约束、关节分支还是数值问题。 | 下一轮保留远离外壳的方向和5mm gate，对首个低位斜移段预先冻结从approach末关节状态连续展开的关节种子，再单次测试；把姿态权重作为提前声明的优化参数，不能在失败后加次数或放宽gate。 |
| `u_shape_bowl_sink__human_default_mpfb` | lift / TCP_SEGMENT_DEVIATION_LIMIT | cuRobo在lift第1段返回solver_success=true，保存39个关节样本。；独立原机器人USD FK重算最大TCP有限线段偏差5.748088mm，超过冻结5.000mm；保留被拒路径，后续段没有调用。；仅保存此前成功的pregrasp、approach前缀，共78个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；本轮没有完整可执行轨迹，也没有该景的新物理结果。 | 这一次候选在首个10mm上抬段未通过路径限制；权重20及禁用graph没有保证TCP直线。该结果不是物理抓持失败，也未证明场景或IK无解。后续下放未规划，旧抓姿的测量补偿参考不能当作新抓姿已经验证的下放证据。 | 下一轮把首个竖直10mm抬升明确单独设为固定XY的短段约束，后续远离人体的斜向段保留姿态约束；预先冻结连续关节种子及5mm验收界限。新抓姿的下放补偿需重新绑定适用的attachment依据；本轮不追加规划或物理测试。 |
| `u_shape_mustard_bowl__robot_sharp_equipment_housing` | approach / MotionGenStatus.FINETUNE_TRAJOPT_FAIL | approach第1段唯一一次调用返回MotionGenStatus.FINETUNE_TRAJOPT_FAIL、solver_success=false、samples=0；没有返回或保存该失败段轨迹。；冻结选项为constraint_weight=20、enable_graph=false；记录完整seed重置且无graph回退。；仅保存此前成功的pregrasp前缀，共39个关节样本；独立USD FK与保存TCP一致，前缀不含失败阶段。；没有对不存在的失败段计算FK偏差、碰撞或IK诊断；本轮没有该景的新物理结果。 | 当前单候选的轨迹优化精调未成功。已有前缀只证明之前阶段生成了路径，不能说明失败段的碰撞位置、反向超调、IK不可达或全局不可行；仅凭状态也不能确定是强约束、关节分支还是数值问题。 | 下一轮联合调整预抓关节分支与首个10mm下降段的种子，使接近从前一段末关节状态连续开始；预先固定姿态/XY权重及5mm gate后再单次测试，不把本次精调失败解释为接触或抓持失败。 |

## 未执行候选的证据限制

下列限制作为审计发现保留；审计通过仅覆盖来源、实际次数、规划拒绝原因及已执行结果，不表示所有候选都已完全验证，也不授予未通过候选执行许可。

- `u_shape_bowl_sink__human_default_mpfb`：当前候选引用旧抓姿实测补偿，不能证明新抓姿的实际放置或碗底支撑；仅旧证据和相同名义整碗目标通过历史绑定检查。候选在lift阶段被拒绝且零物理，下一轮须重建适用新抓姿的证据或回到60mm范围。 原补偿检查失败：`Predicted actual-offset bowl pose differs`。

## 成功场景的实测改善

原任务谓词由120Hz状态CPU复算，仍只使用原水槽底部支撑传感器；本次放置改善不代表长期成功率或安全验收。

- `l_shape_bowl_sink__property_wine_glass`：末3秒360/360采样通过；最终向上支撑2.405N，速度0.334mm/s。
- `l_shape_bowl_sink__robot_narrow_opening`：末3秒360/360采样通过；最终向上支撑4.905N，速度0.266mm/s。

逐景历史与本轮原生记录、来源和SHA保留在JSON绑定的 `bowl_improvement_native_evidence.json` 中。

## 本轮实际失败的图像复核

本轮实际执行失败为0景，因此没有本轮失败诊断图或逐图复核结论。3景规划失败没有新增物理测试，不能记为物理成功。


完整物理记录、诊断图、审计与来源SHA列在状态JSON中；大体积本地证据需单独恢复。首轮及前四轮重试文档保持原样。
