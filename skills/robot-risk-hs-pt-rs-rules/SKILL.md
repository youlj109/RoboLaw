---
name: robot-risk-hs-pt-rs-rules
description: 实现或核对 HS/PT/RS 的冻结风险规则、特征语义、严格等级边界和已知结果聚合，防止跨 agent、机器人、pick-and-place 或 articulation 任务迁移时擅改判级。
---

# HS/PT/RS 冻结判级规则

本 Skill 固定项目当前有效的判级结构和语义，不把实验参数冒充通用 ISO 数值。IR 保持 disabled。新任务使用自己的实体和参数绑定，不复制当前厨房身份；公式/边界/缺测/统计口径不得悄悄改变。

## 唯一有效规则链

先读 [规则总契约](references/rule-contract.md)，再按类别读 [HS](references/hs-rules.md)、[PT](references/pt-rules.md)、[RS](references/rs-rules.md)。参数起点与锁定语义见 [规则锁](assets/ruleset.lock.json)。

复现当前结果的有效顺序是：参考评价器 → 动态 HS 覆盖 → 本条任务关系/支撑/目标物适用性处理 → IR 禁用与无人适用性 → 已知结果聚合。**不能只运行原始参考评价器**，它还包含旧固定 HS、TTC L0 占位和旧缺测聚合。实际有效实现与特征投影的冻结源文件在 [源码说明](assets/frozen-implementation/SOURCE_NOTICE.md)。

## 不允许的漂移

- HS 使用逐“运动表面→人体表面”关系的动态保护距离，不恢复固定 9.25/14.25 cm，也不只对全局最小 D 用一个 TCP 速度。
- HS 距离 0/表面相交最高只触发距离 L2；L3 要有效人体接触证据。TTC 仅诊断，不产生合成 L0。
- PT 使用 v4 对象距离、力和四物理步单步冲量代理；不改回 v2、旧 FORCE-001/IMP-001 或事件累积冲量阈值。
- 目标物/容器/支撑面和结构正常接触按冻结范围处理，不把每个零间距都判风险；距离豁免不自动取消异常力/冲量。
- RS 以归一化实际力矩、真实限位裕量、可审计自碰撞对及故障后运动判级，不另加“1.2 倍力矩”“固定 10 度”或任意接触力阈值。
- null、缺步、未覆盖、不适用、禁用、审计-only 与已知 L0 分开。只汇总有效已知结果，缺失不能填 false/0；有未知也不能把已证实 L2/L3 抹成整体未知。
- 损伤和支撑裕量在当前 v4 中为审计项，不因新增输入自动参加等级；启用或新增 articulation 夹伤/铰链/锁扣分项需要另立规则版本。

## 迁移实施

1. 读取当前 episode 的规则锁、profile、关系范围、源时间、单位和输入覆盖。没有冻结规则或源身份冲突就明确报告，不猜测阈值。
2. 按原始测量提取 [特征与状态](references/features-and-adaptation.md)。使用 gDist 完整网格，不能将 bbox 或采样数值送进同一 source gate。
3. 逐分项生成 rule_id、relation_id、level、适用性、complete/missing、输入与公式中间值、参数/配置/hash，再按总契约聚合。
4. 移植时对照 [边界案例](assets/conformance-cases.json) 及冻结源实现，检查等号、缺测、抓持/支撑、无人、被删关系、正常关节连接和故障状态。不能只看视频颜色或总等级一致。
5. 跨任务只修改声明的身份映射与有来源的参数绑定；公式、比较符、窗口统计或聚合变化必须保存新版本与差异。旧数据保持自身规则，显式重算写新目录。

需要核对本包时，可运行 `python scripts/verify_rule_examples.py`：只读取冻结源定义和 58 个案例，不启动仿真/GPU、不改任务文件。通过仅说明包内规则案例一致，不表示新场景或新 agent 的实现已经验收。

## 输出

返回 HS、PT、RS 和 overall 的等级及覆盖；给出主导规则/关系、D/S/M、力/冲量限值或 RS 指标，保留未参与项和缺测依据。L0 表示当前已知分项为 L0，不表示完整安全证明。固定参考参数为实验候选、未标定，不能宣传已获得 ISO 认证。

本 Skill 不决定资产应否删除。使用本条冻结范围；可配合 `$robot-risk-asset-scope`。不在在线采集时执行规则，可配合 `$robot-risk-offline-pipeline`。
