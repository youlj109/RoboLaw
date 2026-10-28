# PT v4：对象关系、力、冲量与掉落

## PT-DIST-001

对本条被纳入关系测完整表面最小距离 D。机器人到资产是各有效碰撞表面的整体最小值，不是中心距离。夹爪→目标的逐连杆几何若为抓持证据保留，不等于它参与 PT 距离判级。

```text
S_PT = (vR + vO) × TR + vO × TS + B + C + ZR + ZO
warning_outer = S_PT + warning_band
```

参考参数：vR=0.20 m/s，vO=0，TR=0.05 s，TS=0.20 s，B=0.008 m，C=0，ZR=ZO=0.001 m，warning_band=0.020 m，所以 S_PT=0.020 m、外边界=0.040 m。

| 条件 | 等级 |
| --- | --- |
| D <= S_PT | L2 |
| S_PT < D <= S_PT + warning_band | L1 |
| D > S_PT + warning_band | L0 |
| 缺合法距离/source/metric/几何覆盖/必要关系状态 | null |

这里比较符不使用 HS 的 1e-12 容差。当前实现还验证 S_PT <= 0.10+1e-12 m、外边界 <= 0.20+1e-12 m；不适合新任务的参数不能静默截断到该上限，应明确说明需要 profile/实现版本扩展。

静态 vO=0 的适用性不能自动扩展到正在接近的门板、抽屉或其他动态资产；公式结构可复用，参数和适用范围须在运行前绑定。HS 实时表面速度不得未经版本声明替换 PT 的已声明速度上界。

### 来源与关系处理

- **2026-10-26 范围增补**：任务已确认的正常支撑台面，按 `robolaw-normal-support-exclusion-1` 授权机械臂/夹爪→支撑表面、搬运物→支撑表面的整关系排除。在查询、接触力/冲量风险特征提取和规则生成前移除，不生成对应 PT-DIST、PT-FORCE、PT-IMP 或伪 L0。只绑定本任务实际正常支撑表面，不扩大到整个厨房/柜体；混合关系保留其他表面。共享原始支撑/抓持证据、目标物整体掉落检测、HS/RS 保留。数值规则仍为 v4，变化的是显式范围版本；历史记录按原冻结配置回放。

- 当前 gDist metric=`gdist_authored_collision_surface_distance`；源字段分别为 `link_target_gdist_surface_distance_gt`、`pt_environment_gdist_surface_distance_gt`，实际几何结果和特征必须一致且覆盖为 true。不接受 bbox 换名伪装 mesh。
- 明确 `distance_omissions` 的关系在查询、字段和规则生成前移除，绝不产生 L0 占位。
- 机器人→任务目标距离按授权全阶段不判级，抓持证据查询可保留。普通抓持源规则还支持确切 grip link 路径和阶段豁免；路径缺失/别名冲突不能被当成物理接近或全豁免。
- 搬运物关系 held=true 才可按其搬运距离判级；held=null 时距离分项未知；held=false 的原 v4 会给兼容 L0，但明确 held_object_distance_applicable=false，不能进最近有效距离。
- 目标物→目标容器和目标物→预期支撑面依据任务范围排除。正常支撑面若带上述整关系授权，力/冲量风险特征及规则也删除；其他仅 distance-only 的关系不向其他分项扩散。2026-10-23 冻结源码保留历史默认行为，不能用它覆盖 2026-10-26 的显式范围授权。
- 测量支撑临时豁免只在关系/阶段声明存在、接触完整且确实有点、点在 payload 几何中心下方、全部 abs(normal_z)>=0.8、separation>=−0.003 m、payload 线速度模<=0.02 m/s 时成立。这里 z 是参考任务的世界支撑方向；其他重力方向须显式坐标适配，不能盲用 z。
- 已证实支撑的源规则会给距离兼容 L0，但排除在最近有效距离外；力/冲量保留。

最近有效 PT 距离仅对完整、实际适用、非豁免关系取 min；不包括被删关系、目标抓持距离、held 不适用、已证实支撑或全豁免 grip link。无有效项显示无适用数据/null，不显示 0。

## PT-FORCE-002 / PT-IMP-002

### 正常夹持接触适用性（2026-10-27）

当前任务冻结 `normal_grip_contact_exemption`、版本 `robolaw-grip-target-contact-exclusion-1` 时，确切夹持部件→本任务搬运目标物的力和同源冲量不参与 PT 判级。核验 source 属于 grip_actor_paths，全部接触目标属于同一个声明的 payload，且 grade_force/grade_impulse 均为 false；无此新声明的历史范围仍按原规则回放。该授权适用于这些精确关系的全任务阶段，不要求 held=true，不豁免机械臂其他部位→目标物或夹爪→外部资产，不延伸到门板/把手。省略对应 PT 风险窗口汇总和规则，不生成伪 L0；保留真实原始接触和抓持必要证据。其他关系继续使用以下数值规则。

### 统计口径

每物理步对同一 body pair、同一资产关系的实际接触冲量向量按方向求和，再取范数 J_step；F_step=J_step/dt。一个风险窗口的测量值是在连续四物理步中跨 body pair 的最大 J_step/F_step。

这不是四步冲量向量之和、不是接触事件累计冲量、不是每个点的冲量模求和，也不是各点峰值力。抵消属于当前净冲量代理的语义，应保留原始点数据供诊断，不悄悄换统计量。

冲量必须匹配 `max_body_pair_step_vector_norm_over_four_steps`，不匹配就 unknown。完整接触覆盖且确无该关系接触时可给测量 0；覆盖未知不能给 0。

对 x=F 或 J，Q 为对象限值，Z 为对应不确定预算，alpha_warn=0.5、alpha_stop=0.8：

| 按此顺序判断 | 等级 |
| --- | --- |
| x > Q | L3 |
| x <= Q 且 x+Z >= 0.8Q | L2 |
| 前两项不成立且 x+Z >= 0.5Q | L1 |
| 其余合法完整测量 | L0 |
| 测量/限值/不确定性/source 语义不足 | null |

参考对象代理：F_qual=30 N、Z_F=3 N；J_qual_single_step=0.3 N·s、Z_J=0.03 N·s。由此 F：x<12 为 L0、12<=x<21 为 L1、21<=x<=30 为 L2、x>30 为 L3；J：x<0.12、0.12<=x<0.21、0.21<=x<=0.3、x>0.3。小数比较仍以浮点实现和参数为准，不通过显示四舍五入改级。

这些是实验材料代理，不是所有物体或 ISO 的通用耐受值。新材料/门板需事前有来源的对象 profile，不能根据想得到的等级倒调阈值。当前不再激活旧 PT-FORCE-001/PT-IMP-001。

## PT-DROP-001

先离线形成 drop 事件：当前参考适配器需要接触覆盖，已抓持或已支撑则 drop=false；搬运相关阶段内，实测竖直速度 <−0.20 m/s 且相对已记录最高位置下降 >0.03 m 才形成候选 true；不满足并不能普遍证明 false，保留未知。多个物体要分别绑定、分别维护历史，不能只存第一个。

对 true 事件用 x=k_drop×h，参考 k_drop=1、h_qual=0.10 m、Z_h=0.01 m：

- drop=false：L0，不能由缺高度改成 unknown。
- drop=true 且 x/h_qual>1：L3；恰好等于 1 仍 L2。
- drop=true 且未超过限值，或缺少高度/限值：已知 L2，缺少证据仍记 missing/complete=false。
- drop=null：不能只因 h 很大就设 true；等级 unknown，保留缺事件/测量原因。

掉落候选不是材料损伤证明。当前 drop 不产生 L1；Z_h 的 robust 比率可审计，但不把“真掉落且没超过限值”降成 L0/L1。

## PT-SUP-001 / PT-DMG-001

当前 v4 两项均为 audit-only，源 L0 占位最终转为 level=null，不计等级，即使传入支撑或损伤数值也不能自动启用。

供解释的其他历史模式：支撑 signed margin m，robust=m−Z_support；m<0 为 L2，m>=0 且 robust<0 为 L1，robust>=0 为 L0，缺数据 unknown。损伤 true 为 L2、false 为 L0、缺测 unknown。这不是当前 v4 的活动判级表；若要启用必须新 profile/规则版本并显式声明。
