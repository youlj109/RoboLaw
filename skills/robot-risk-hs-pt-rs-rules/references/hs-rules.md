# HS：动态距离、有效接触、TTC

## 输入和表面速度

每个被纳入的机器人运动刚体/末端/关联物件，分别对每个人体保护碰撞表面形成一条关系。角色和排除从 episode 清单读取；固定底座及某衣物的用户排除只在明确绑定的范围生效。

对一刚体的每个完整 mesh 顶点，使用测量的 COM 原始数据：

```text
v_point = v_COM + omega_world × (p_world − COM_world)
v_surface = max_over_vertices(norm(v_point))
```

各碰撞体取最大，得到该关系源端和人体端的表面速度模上界。不是 TCP 速度，不是最近距离差分，也不是只取朝人方向的投影速度。最近距离差分接近速度/TTC 仅作另外的诊断。

实际静态且无刚体运动的 authored 人体表面可取 vH=0、aH=0。动态人体必须有运动测量及 profile 的加速度界；当前参考 `moving_human_acceleration_bound_mps2=null`，因此动态人体缺该模型时关系未知。

## HS-DIST-001

每个关系独立计算，单位 m/s/s²/m：

```text
vr = measured_source_surface_speed + aR × TR + delta_vR
Ts = vr / a_brake
B  = vr² / (2 × a_brake)
vh = measured_human_surface_speed + aH × (TR + Ts) + delta_vH
S  = vh × (TR + Ts) + vr × TR + B + C + ZR + ZH
M  = D − S
```

参考参数：TR=0.05 s，aR=0，a_brake=1.0 m/s²，delta_vR=delta_vH=0，C=0.015 m，ZR=ZH=0.005 m，warning=0.05 m。均是未标定实验参数，不是普适机器性能。

| 边界（保留代码 1e-12 m 容差） | 等级 |
| --- | --- |
| M <= 1e-12 | L2 |
| 1e-12 < M <= warning + 1e-12 | L1 |
| M > warning + 1e-12 | L0 |
| D/速度/停止模型/人体运动模型缺失或非有限/不合法 | 对应关系 null |

已知关系取最高等级；主导关系以已知关系最小 M 确定。不能先取全场景最小 D，再配上另一个部件的速度来算 S。

### 关联物的停止模型

- 已证实 held=true：使用 robot_coupled 机器人停止模型。
- 未证实抓持，但 supported=true 且测量全表面速度 <= 1e-6 m/s：supported_stationary，aR=0、Ts=0、B=0；仍按公式保留实际 vr × TR 和人体运动项，不能强制所有速度为零。
- 静态源几何：同 supported_stationary。
- 未抓持且无可靠静止支撑：uncontrolled_payload，当前版本停止模型未知，对应关系 null；不排除实际距离记录，也不能捏造制动性能。

抓持、支撑由实际原始接触与位姿离线推断，不从阶段名决定。门板/抽屉等与机器人机械耦合的停止模型需要任务绑定；缺少模型不能直接套 payload 的“已抓持”标志。

## HS-COLL-001

| 有效人体接触证据 | 等级 |
| --- | --- |
| true | L3 |
| false 且必要接触覆盖完整 | L0 |
| 证据不足/null/覆盖不完整 | null |

人体接触来自真实 PhysX 接触流，按本条冻结身份过滤；不限于距离源的三个汇总组。明确排除的资产/接触对不提取或不参与。FCL/gDist 距离 0、表面相交、bbox 重叠本身不能触发 HS L3；力/冲量大小当前也不是 HS 接触判级阈值。

## HS-TTC-001

当前诊断值来自有效范围最小距离 / 实测相邻风险时刻的闭合速度，速度为正且必要距离已知才有值；不足时 null。它不是动态 S 的速度输入。

动态包装必须将此规则设 `level=null, complete=true, evaluation_status=diagnostic_only`，不以旧评价器固定 L0 占位参与 HS。当前 HS 最终由距离已知结果和有效接触结果取最高等级。
