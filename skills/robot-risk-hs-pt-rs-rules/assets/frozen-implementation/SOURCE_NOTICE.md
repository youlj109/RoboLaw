# 当前规则的冻结实现参考

SOURCE_MANIFEST.json 给出每个原始项目文件的位置与 SHA-256。基线为 RoboLaw `huirongzeng` 对应的实际运行目录，包含 2026-10-23 用户授权但当时尚未提交的 HS/PT 资产范围修改；不能将其全部谎称为某个 Git 提交的原始内容。

## 有效调用链

1. `safety_risk/evaluator_factory.py` 根据 profile 选择 `iso_ssm_candidate.Evaluator`。
2. `arena_adapters/hs_dynamic.py` 的 DynamicHSEvaluator 替换动态 HS 距离，并将 TTC 改为 diagnostic_only/null。
3. `arena_adapters/pt_assets.py` 的 apply_support_distance_rules 处理实际任务、目标、支撑与整项关系适用性。
4. `arena_adapters/risk_observer.py` 的 arena_coverage_report / aggregate_known_results 处理 IR 关闭、无人、缺测和已知结果聚合。
5. `risk_binding.py`、`grasp_evidence.py`、`rs_evidence.py` 说明实际特征提取语义；`hs_geometry.py`、范围策略和 frozen_geometry.py 说明几何/身份边界。

profile.reference.json 与 runtime-config.reference.json 是当前场景配置，不是所有任务通用身份或经过标定的机器参数。新任务改绑定时记录新 profile/hash，保持实际数值规则版本，明确参数适用性。

## 使用限制

这些文件用于逐项核对和移植，不构成脱离项目即可直接运行的完整仿真工程；Arena 适配层依赖项目其他接口，且存在单 payload、具体机器人和厨房字段。不要只改几个名字就声称 articulation 已支持。

参考包保留旧模式和历史实现，供源码路由/审计使用。不能因为文件里有旧固定 HS、旧 TTC L0、旧 PT v1/v2 或旧聚合，就跳过有效包装链。不要调用附带源码去做在线安全判级。

上游/参考代码保留其原归属；RoboLaw 自有贡献的项目 MIT 许可随 LICENSE.robolaw 保存，不替上游重新授权。
