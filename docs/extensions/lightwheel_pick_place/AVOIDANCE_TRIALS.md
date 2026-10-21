# 青柠任务：真实避障对照补测

旧 `skill_trials` 三条只算物理流程验证；它们的原基线路径仍可行，不计有效避障。此次仅改变新增物体摆位，不移动原场景对象/机器人，不改变原成功谓词。原布局为 `fixed_layout.json`，seed42、35°倾角/yaw0、源青柠几何中心上15mm抓点固定；所有新成功样本必须与匹配的新无障碍基线比较。

CPU扫描由 `scan_avoidance.py` 读原成功 `base_attempt12` 的实际q、指爪和lime姿态完成，最终仍要对新合格基线完整120Hz再审。初态与终态/任务目标合法、所有源厨房与原对象/固定base/stand无非支撑交叠。

| 候选 | 新增物体位置/朝向 | 旧实测基线风险 | 后续要求 |
| --- | --- | --- | --- |
| human | `[3.15,-1.225,0]`，yaw270° | 臂壳最低11.82mm < 原20mm，54个15Hz样本 | 同TCP抓点/朝向的CPU冗余IK已证完整接近/抬升在30mm规划距离下可行；需真实MotionGen采集 |
| property | `[4.1,-.25,.922]` | transfer携物最低37.01mm < 原50mm，13个样本 | 原接触段端点均可行，需真实绕行/放置 |
| robot | `[4.1,-.15,.922]`，yaw0° | transfer机械臂与横杆相交，29个样本 | 原接触段端点≥31.27mm，需真实绕行 |

候选/CPU IK不是成功样本。人体首轮要求所有旧接触段q保持不变时没有解；第二轮仅允许机械臂冗余姿态变化，初态/最终姿态及源物体全部不变，找到上述候选。GPU规划必须再次完整检查，禁止直接执行危险基线。

## 源支持接触分类修复

新base01在lift第3动作被之前后加的“闭爪hull底−0.5mm、后续单调提高”条件拒绝：实测原生rest底0.916084642m，闭爪底0.917928994m，起步滚转降到0.917407382m；它只低过人为单调界约40µm，仍高于原生合法rest约1.3mm。base02把同几何路径改为2倍时长smoothstep仍发生同类滚转，说明继续减速不是合理修复。两次失败原数据/视频/日志全部保留。

新模型 `native_rest_until_detached` 只修正原源台面接触语义，所有人体/花瓶/横杆及其他厨房距离不变：

- 精确来源仅 `/World/envs/env_0/kitchen/counter_main_main_group/top_geometry_right`，原台面0.92m。
- 源区域xy距初始根位置<50mm；真实源碰撞凸包底不低于原生初始rest−0.5mm；任意资产root高度不作为接触面。允许离台前接触滚转，不要求果实旋转时最低顶点单调。
- 只有pregrasp/approach/close/lift可使用该局部支持例外。实际hull底离台至少原2mm间隙，且精确源台面接触消失后，`detached`永久锁定，后续落回不能重新获得例外。
- 每个物理步记录真实lime姿态、hull底、原始PhysX源接触点/法向/带符号力/分离量与detach前后状态。独立 `audit_source_support.py` 从原USD顶点重新计算高度与状态链；不能用规划球包代替。
- 6个测试覆盖实测滚转、低于原生rest、离开源xy/阶段、必须间隙与失接触同时成立、永久脱离后不能重入。

base04和后续变体恢复原速度与原35°/+15mm抓点，统一采用该源支持分类。`--smooth-lift`仅保留为失败诊断可复现参数，不当作成功技巧。

## 最终门控

每条新样本除原任务当前终态、真实双指接触与提升、完整厨房/携物/固定台架/自碰撞及RGB审计外，还必须有 `avoidance_audit.json`：同源reset与控制参数、有效基线原成功、几何反事实至少一个旧安全距离违规、新实际120Hz完整安全、合法初态/终态/源几何及实际路径差异。`baseline_counterfactual.passed=false`表示反事实安全检查失败，`violates_safety_margin=true`和顶层`avoidance_effective=true`才说明效应成立（新轨迹同时安全）；反事实不执行。首帧实测7关节差异≤1e-6 rad，源物体实际settling位置/朝向/速度差异单列，不混同名义reset相同。花瓶120Hz实测平移≤5mm、倾角≤2度。图 `path_comparison.png/pdf`展示实际q/TCP/lime/肘部路径与风险距离，不把随机差异独立作为证据。

当前仍在补采，不因候选或有视频计入成功配额。

`base_attempt03` 保留失败：原始接触正常，hull底高于原rest约1.295mm，但任意资产root因滚转低于额外root界11.7微米。此root界不表示真实穿透，故04起删除该冗余门，仅以完整真实碰撞凸包底、精确源面/XY/阶段和永久detach判断支持。报告明确`root_z_gate=false`；审计旧03仍执行其当时root规则。六项边界测试覆盖此实测反例。

## 新严格无障碍基线

`datasets/avoidance_trials/lightwheel_pick_place/base_attempt04` 已完成481动作、3848物理步。原当前 `final_on_destination` / `final_live_success` 均真，独立validation（原始双指接触、全部JPEG、三路视频+拼接）、完整外部世界和源支持重算均通过，自碰撞补审排队。实测整条q、lime与bowl数组与旧base12差值完全为0，见 `reports/avoidance_trials/lightwheel_pick_place/base04_old_path_comparison.json`；旧成功记录未补写，新基线拥有完整新增终态证据。三变体仍以新基线独立120Hz审计后才计有效。

## 已通过：property_attempt01

490动作/3920物理步，所有独立门控及`avoidance_audit.accepted=true`。原基线在实测花瓶初态下102步违反50mm，最小37.5757mm；新实际轨迹最小115.9723mm。transfer真实携物路径最大改变173.903mm，花瓶最大平移0.000504mm、倾角0.000239度。

名义源布局相同、实测臂q/lime初态完全相同，木碗差23微米；broccoli出现22.505mm/53.45度的原生settling差异，明确不是完全同物理态。两条真实路径在两套实测源背景的4组交叉检查全部通过，原路在新背景仍有44.266mm净距（要求2mm）；进一步扣除所有源背景15Hz观测的保守运动包络后仍有42.256mm余量。因此源杂物settling不能单独解释旧路径受阻。此检查及任务物初态1mm/1度硬门已成为所有新样本强制验收。


人体 human_attempt01 已完成正式全部独立门控，avoidance_audit.accepted=true。以同base_attempt04实测路径作反事实，437个120Hz样本低于20mm、最小11.811mm；障碍感知冗余IK保持原TCP抓点/朝向，真实新路径3792步最小38.065mm，474个动作。原当前终态、完整原始RGB/同步overview、源支持、完整世界、自碰撞、固定台架及两路径×两套实测源背景均通过。path_comparison.png/pdf已实际审看，距离曲线和关节姿态变化共同证明避障。


横杆诊断：原0.40m圆管在原位置的56个同TCP transfer IK均被cuRobo世界检查拒绝，移除横杆后56/56通过，自碰撞/限位56/56通过。左移可使目标可行却使home无效；网格找到的home/goal交集 y=-.08 又由实际厨房几何查出底座穿后墙，全部拒绝。单独新增 short_projecting_crossbar.usda，将悬臂长度0.40m变为0.28m并对应移动视觉端盖/警示环，半径、底座、立柱和全部间隙不变；原资产/失败记录保留。该新资产在[4.0875,-.15,.922]经CPU完整源/台架与接触端点检查通过，解析Cylinder反事实在旧transfer路径真实相交；当时仅是候选预检，后续robot02未通过，最终robot03通过全门控，详见末尾正式结论。


短杆 robot_attempt02 未通过：close第2动作的真实lime凸包底.915479747m，原生rest底.916084583m，超过既定0.5mm支持带0.104836mm；同帧exact source接触为向上20.94N、+1.230mm separation、尚未detach、XY变化仅0.204mm。未放宽支持规则。实际闭爪起点与base04 q差2.63e-5rad、TCP差0.34µm，因此不能将本次失败归因于大幅冗余分支变化；属于原高速二值夹爪的接触瞬态敏感性。预检四个合法候选后，robot_attempt03采用独立config的[4.095,-.12,.922]，同seed/同控制/同原场景/同阈值；若同类失败再现，先修可复现夹爪控制并重采匹配基线，禁止继续靠随机重试或改支持带。


## 最终正式样本（全部accepted）

共同基线 `base_attempt04` 已通过原终态、全部图像/接触、源支持、完整世界和3848物理步自碰撞。三个正式episode分别为 `human_attempt01`、`property_attempt01`、`robot_attempt03`，均在 `datasets/avoidance_trials/lightwheel_pick_place/`，所有 `avoidance_audit.accepted=true`。

| 类别 | 旧路径风险 / 原要求 | 新实际最小距离 | 动作 / 物理步 |
| --- | --- | --- | --- |
| human | 437步，11.811mm / 20mm | 38.065mm | 474 / 3792 |
| property | 102步，37.576mm / 50mm | 115.972mm | 490 / 3920 |
| robot | 181步，真实相交 / 10mm | 55.854mm | 498 / 3984 |

robot03使用明确0.28m短悬臂资产和 `avoidance_variants_robot03.json`（[4.095,-.12,.922]），保留原源场景/抓点/朝向/seed/控制/支持带。全程原成功predicate、当前原支撑/低速终态、真实双指夹持抬升、所有RGB与4视频、完整世界/自碰撞/源支持/固定台架/双路径双背景/运动包络都通过。transfer的实际lime/TCP最大路径差为134.903/124.339mm，q向量差0.66283rad；pregrasp仅0.00147rad，说明实际变化发生在受阻的搬运段。原杆不可规划、违法后墙候选、短杆02闭爪失败均保留；未调整碰撞阈值或夹爪物理参数。

每条有 `path_comparison.png/pdf`，根汇总入口另生成同相机原速 `comparison.mp4`。人体保留额外同步overview；其余三路全部保持原语义。独立C审查再次用真实hull窄相位复现human/property关键最小距离，排除了附加capsule代理造成假风险；控制/支持必需字段加存在且非None检查，三条均通过，见 `required_control_fields_audit.json`。这只是每类1条有效试采，不是200条采集或稳定成功率承诺。
