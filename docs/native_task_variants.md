# 统一准备任务与风险变体

## 用途与范围

`scripts/prepare_risk_task_variant.py` 为任务目录中任一任务准备明确的风险变体输入。当前目录含 32 个任务、12 种变体：baseline、静态成人、6 种财产障碍、4 种机器人周边障碍。它不启动采集或 GPU，不生成风险等级，也不要求每组 200 条。

资产身份、文件路径、物理动静态类型和风险类别来自资产目录；放置配置只负责本次任务的摆放与显式人体范围授权。后续原生采集仍根据实际加载的网格、关节图和状态建立 HS/PT/RS 关系，而不是按场景类别预设等级。

## 准备一个已有示例

以下是经过单独检查的 G 型岛台横杆配置，不是其他厨房的通用坐标或默认布局：

```bash
python scripts/prepare_risk_task_variant.py \
  --task-id replicator_g_shape_mustard_bowl \
  --variant-id robot_projecting_crossbar \
  --placement configs/risk_asset_probes/g-island-wide.placement.json \
  --output /absolute/new-prepared-input
```

使用生成文件调用原有默认入口：

```bash
python scripts/risk_capture_workflow.py capture \
  --task-contract /absolute/new-prepared-input/task_contract.json \
  --obstacle-binding /absolute/new-prepared-input/obstacle_binding.json \
  --asset-cache /absolute/asset-cache \
  --policy-factory YOUR_POLICY_MODULE:create \
  --policy-config /absolute/your-policy.json \
  --output /absolute/new-run
python scripts/risk_capture_workflow.py evaluate --output /absolute/new-run
```

实际控制器仍须属于该任务；准备入口不生成策略，不以保持动作声称任务成功。风险特征和等级全部离线，时钟、gDist 后端、规则版本和豁免逻辑不因变体选择改变。

baseline 不传 `--placement`；其输出没有 `obstacle_binding.json`，采集时不传 `--obstacle-binding`。

## 输入契约

每个非 baseline 变体必须显式给出属于本任务的放置 JSON，包含：

- `task_sha256`：该任务契约的规范化摘要，不能只更换任务名称。
- `placement_mode`：`native_relations` 或 `world_pose`。
- `placement_evidence`：本任务摆放来源与用途，不能为空。
- 原生关系模式：明确 `placement_relations`，引用当前任务资产或机械臂 ID；随机范围可选，限制与现有放置采样器一致。实际 seed 与抽样仍在采集时冻结。
- 世界坐标模式：有限的 `position_xyz` 与归一化 xyzw `rotation_xyzw`。这是用户明确提供的当前任务位置；准备成功不证明位置可达。

模板不能覆盖 `asset_id`、`usd_path`、`static`、`category`。不能把旧模板的 `counter_top` 带入只声明 `island_top` 的任务。原生关系的构造参数和实际几何/可达性检查仍由加载时的 Arena 验证。

人体选择由 `configs/risk_human_assets.json` 声明真实 USD 根文件版本；当前为已有 MPFB 静态成人。默认不继承衣物省略。若明确提供 `hs_surface_omissions`，必须绑定相同源文件哈希、具体相对网格路径、授权和理由；加载后的完整人体清单还会检查该路径及剩余保护表面。根 USD 哈希不是依赖闭包证明，完整资源仍通过恢复流程及本条冻结网格确认。

## 输出与采集时再次检查

| 文件 | 内容 |
| --- | --- |
| `task_contract.json` | 选定任务的独立契约副本 |
| `obstacle_binding.json` | 非 baseline 的任务/资产/放置绑定及完整准备摘要 |
| `input_placement.json` | 原始放置输入，尚未抽样 |
| `asset_selection.json` | 资产来源、物理类型和目录记录/源文件摘要 |
| `variant_bundle.json` | 各输出文件哈希、默认采集入口所需参数、尚待运行的验证状态 |

原生环境构建器在采样和打开场景之前重新核对 `prepared_binding_sha256`、任务哈希、资产根文件哈希。准备后改任务、静态属性、资产路径或原始资产文件会被拒绝；需要重新准备新目录。历史手工绑定仍沿用原接口，但不会被标记为通过了新的准备检查。

输出目录必须不存在。可移植到其他主机时在目标主机重新准备，以解析当地资产路径；不要只改已记录摘要的绝对路径。

## 正确性检查与边界

```bash
python scripts/check_task_variant_bundles.py --output /absolute/new-bundle-check.json
```

检查覆盖全部 384 种任务/变体选择、24 个无效输入案例、准备文件摘要和采集前资产变化检测。矩阵中的摆放是明确标记的合成输入检查，不是 384 个物理布局或安全评估样本。

`runtime_placement_verified=false`、`runtime_scope_verified=false`、`safety_evaluated=false` 保留到相应阶段获得实际证据；配置准备成功不能把这些状态改成通过。正常目标/容器/支撑面、人体固定结构及操作接触豁免仍由任务范围规则绑定，不在这里增加删项或改变阈值。

## 2026-10-24 原生链路核验

通过准备入口输出文件直接调用默认采集/离线入口，两个案例均通过原生摆放、原始审计和专项核对：

| 案例 | 物理步 / 标签 | HS 关系 | 四帧网格距离记录 | 每帧 RS 对 |
| --- | ---: | ---: | ---: | ---: |
| Coastal 冰箱＋静态成人 | 12 / 4 | 90 | 8,320 | 119 |
| G 型芥末瓶＋横杆 | 12 / 4 | 0（无人） | 10,996 | 119 |

核对内容包括准备文件摘要、当前资产源文件、实际加载身份/物理类型、冻结后的采样参数、人体显式省略范围、独立 RS 清单、全部需查询关系的 gDist 结果、在线零风险调用与 IR 关闭。人体案例显式提供了既有授权衣物省略；它没有成为其他任务的默认值。保持动作不构成完整任务或风险覆盖范围的验收。

```bash
python scripts/verify_prepared_variant.py   --bundle /absolute/prepared-input   --run /absolute/run   --output /absolute/new-verification.json
```

具体结果和源数据摘要位于 `task_suite_validation_status.json.task_variant_preparation_20261024`。原始记录仅保存在运行主机，不上传 Git。
