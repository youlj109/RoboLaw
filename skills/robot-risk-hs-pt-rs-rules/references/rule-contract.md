# 冻结规则总契约

## 版本与权威顺序

本包基线为 2026-10-23 已运行的组合：HS `arena-hs-dynamic-ssm-1`，PT `pt-hs-inspired-v4`，聚合 `arena-known-results-1`，gDist `arena-gdist-runtime-1`，HS 数据省略 `arena-hs-data-omission-1`。冻结源和配置保存在 assets，文件摘要在每份快照的 SOURCE_MANIFEST.json。

优先级：本条数据冻结配置及适用性 → 本包有效包装链 → 冻结的实际代码边界 → 人可读公式。发现冲突应报告具体字段/代码，不自行选择一个方便的版本，也不要静默修补后声称复现原结果。

历史源码中的固定 HS、旧 PT 模式、TTC L0、保守“缺测就不给 L0”的原始聚合不一定是当前有效行为。必须应用动态 HS 和 Arena 适用性/known-results 包装后再比较。

## 每条规则的状态

输出 `rule_id, relation_id, category, level, complete, missing, evaluation_status, evidence, profile_hash, scope_hash`。

- `level` 为 L0/L1/L2/L3 或 null；不能用字符串“未知”做数值比较。
- `evaluated`：有足够证据作该分项判定。
- `partial_known_results`：已知关系可给等级，同时仍有缺测关系。
- `not_applicable`：明确关系/阶段/无人适用性排除，不是测到了安全。
- `omitted_by_policy`：查询前就省略；在冻结清单中记录，不生成逐帧距离规则、距离字段或伪 0。
- `audit_only` / `diagnostic_only`：不参与当前等级。禁用 IR 不是 IR L0。

## 聚合

保持 L0 < L1 < L2 < L3。类别等级是该类别规则中已知等级的最大值；没有有效已知规则则 null。严重性和完整性分开：L0 也可能 `data_complete=false`，已知 L2/L3 在其他证据缺失时保留。

当前实现把 `note='legacy: audit only'` 的规则 source_level 留审计，level 转 null，排除出聚合。HS-TTC 已由动态包装设为 null/diagnostic_only。

类别 complete 还受所有必要规则/关系、缺测列表及原始覆盖约束影响；几何完整不等于损伤、抓持、故障等证据完整。overall 按实际冻结实现的类别结果取最大，另给整体覆盖与 known-results 状态，不能将 overall L0 解释成所有规则都已检查。

### 兼容行为，不能误读

- 当前无人包装会输出 HS=`L0` 且 status=`not_applicable`，这是兼容字段，不是测量人体安全为 L0。原聚合的 overall 可能包含这个兼容值；无人且其他类别全部未知时尤其不能宣传有完整安全证据。若要改全局 not-applicable 数值语义，必须显式新版本，不暗改历史。
- 未抓持的搬运物距离、完全抓持豁免和已证实支撑距离在原 v4 的部分路径会返回兼容 L0；距离汇总通过 applicability/豁免标志排除它们。不得将这些兼容值当作真实最近距离或覆盖被删关系。
- 明确省略项没有逐帧规则，不能用兼容 L0 占位代替。

## 无人、禁用、错误

HS 不适用必须来自闭世界人体清单：资产加载状态、实际碰撞体身份和明确 verified_absent；缺少人体通道不等于无人。只要有人，完整受保护表面应由本条范围绑定。

IR disabled，level=null、完整性不参与活动类别汇总。文件、网格、位姿、source gate、参数单位或身份不匹配要报错/未知，不切换几何口径填值。

## 频率与事件

120 Hz 物理、30 Hz 观测/判级是当前统计语义的一部分。一次标签窗口含连续四个物理步；相同源时间因果回放。改变频率、窗口、采样漏步、用窗口末点替代事件覆盖，都可能改变规则，不是纯性能优化。

当前只做离线影子标注，不改变控制命令。等级阈值是项目实验规则；ISO/SSM 方法来源不等于这些四级标签和数值是标准直接规定。
