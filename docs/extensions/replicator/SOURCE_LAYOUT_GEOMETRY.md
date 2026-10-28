# A 组原始布局的源几何检查与候选搜索

这两个入口只运行 CPU，不启动 Isaac Sim、cuRobo 或采集。输入必须是 A 组原任务、原厨房和原物体资产；原 On、NextTo、朝向、台架高度和目标身份均由原任务 YAML 校验。报告中的 `execution_ready` 始终为 `false`。

## 检查一个已有布局

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_source_layout_geometry.py \
  --layout configs/extensions/replicator/l_shape_bowl_sink_source_layout.json \
  --output reports/replicator_preparation/my_unique_layout_geometry.json
```

检查内容包括原始任务关系、台架与非地板厨房、物体与厨房、任务物体之间、台架与物体之间的原始源表面距离，以及物体最低 2 mm 顶点带内**每个真实源顶点**下方的原台面三角形。AABB 只用于距离查询的保守排除和原 Arena 关系验证，不作为源网格无碰撞或有支撑的证据。

源表面分离距离严格要求 `>= 0.002 m`，不添加容差。原始布局算出的 `1.999975 mm` 会保留为失败。支撑射线另用 5 mm 高度加 2 mm 顶点带宽作为更严格的初态诊断上限。原 On 源码的 tolerance 为 5 mm、clearance 默认为 10 mm，实际允许底面高度带是 **(-5 mm, +15 mm]**；两者不可混淆。所有门槛检查通过时退出 0，否则保存完整报告并退出 1；已有报告不会被覆盖。

## 搜索一个布局的近邻候选

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/search_source_layout_candidates.py \
  --layout configs/extensions/replicator/l_island_mustard_bowl_source_layout.json \
  --initial-diagnostic reports/replicator_preparation/remaining_geometry_cpu01/l_island_mustard_bowl_geometry.json \
  --output-dir reports/replicator_preparation/my_unique_l_island_mustard_search
```

初始诊断的布局内容和 SHA256 必须与输入完全相同。输出目录必须不存在。搜索前先保存 `search_declaration.json`，声明固定规则：

- 台架：世界 XY 平面半径 0.8 m，步长 25 mm；Z、朝向、台架高度固定。
- 任务物体：XY 半径 0.5 m，步长 25 mm；Z 候选为原值或 +1 mm。抬高 1 mm 仍须通过原 On 验证及更严格的射线诊断，用于获得真实大于 2 mm 的初始表面间隙；它不改变安全距离。
- 先台架、再物体，按位移欧氏距离升序；并列依次比较 XY 的 L1 距离、Z、正 X、正 Y。每个主体选择首个通过原关系、2 mm 源表面分离和真实台面射线检查的候选。
- 原厨房、背景物体、所有源资产、原始任务关系和目标不变。失败候选全部保留，找到首个合格点后停止该主体搜索，并记录未评估数量。此顺序局部搜索不宣称全局最优，也不宣称范围外无解。

输出包含：

| 文件 | 内容 |
|---|---|
| `search_declaration.json` | 搜索前声明、原失败报告与源码哈希 |
| `search_report.json` | 所有已评估点、失败原因、选点顺序、源依赖哈希与源码未变检查 |
| `candidate_layout.json` | 单独的候选配置，不覆盖原失败布局 |
| `candidate_geometry.json` | 对最终组合布局重新进行的完整源几何与原关系检查 |

该检查使用源 USD 三角形表面和支持的解析基本体；不会以 convex hull 代替源网格。它尚不证明封闭表面的实体包含关系、PhysX 烹制碰撞体、连续承托面积或物理稳定性。通过后仍需独立检查完整机械臂/夹爪、关节限位、可达性、真实中性保持、任务执行、逐子步安全、配对避障必要性和最终视频。`initial_geometry_passed` 只表示本报告列出的初态源几何检查通过，不能用作采集完成证明。

原先的 `reports/replicator_preparation/remaining_geometry_cpu01/diagnose_layouts.py` 与所有初始失败报告保留不变，以保留历史证据。正式入口不依赖执行该历史脚本中的字符串片段。

## L 型带中岛碗入水槽的完整域诊断

`search_l_island_bowl_domain.py` 搜索原 On/NextTo 的完整位置范围，保留两端可放下台架且完整机器人初态通过的实例。0.8 m 局部失败不能推出整个原任务无解。`certify_l_island_bowl_reach.py` 另用源实体内棱柱、连续排除域和完整机器人关节图外包界，检查原默认朝向下静置合法碗的直接接触必要条件。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/certify_l_island_bowl_reach.py \
  --domain-report-dir reports/replicator_preparation/l_island_bowl_full_domain_cpu01 \
  --output reports/replicator_preparation/my_unique_reach_certificate.json
```

最终保存的 `continuous_source_reach_certificate03.json` 修订了 02 的浮点域边界：源 float32 验证器可能接受解析边界外约 0.1 微米的点。新 `source_domain_rounding.py` 保存实际最外 float32 接受值、相邻拒绝值和输入舍入原像外包；原 2 mm 安全阈值不变。所有 20 个源机器人碰撞体和固定台架均覆盖。结论仅是默认固定 yaw、静置源初态下直接接触不可达，**不是任意物理策略全局无解**。旧报告与失败证据保留。

## CPU 回归验证

```bash
bash scripts/arena_python.sh -m unittest discover \
  -s tests/extensions/replicator -p test_source_layout_geometry.py
```
