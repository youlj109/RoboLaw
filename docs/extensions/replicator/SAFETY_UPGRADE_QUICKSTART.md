# 154个场景：先看图，再精查

进度网页：<http://127.0.0.1:8765>。首次刷新页面，之后每3秒更新。

本轮由一个使用者交给Agent自动处理，不需要学生分组、多个个人分支或逐场启动命令。

当前使用的154场景配置：`configs/extensions/replicator/visual_scene_variants_final01/manifest.json`。原版本保留，可逐场对照。

本轮先实际看完原154张图，再集中修正14个人体、14个夹架和13个台面物品候选。11个夹架使用显式记录来源的加长派生模型，原资产未改。只新增渲染76场景，复用17组已完成多视角和61张摆位未变的原图；当前全部154场景都有与配置对应的图片。新增视图的93场人工复看结果为85个候选、2个建议调整、6个仍有疑问。原图复用的61场保留原“候选”初筛结论。

这一步完成的是视觉筛查与候选场景搭建。2个建议调整的是半岛芥末瓶任务的门框和横杆，新增部件仍偏离主要操作区。原任务物体与厨房背景的相交问题在网页独立标记；即使新增物摆位可保留，也不能把原任务标成可执行。截图中的机器人为资产初始展示姿态，不是实际采集状态。

最终逐场复看记录在 `reports/replicator_safety_upgrade/visual_placement_final01/review_*.json`，配置、图片和复用证据均绑定SHA256。新增视图在 `reports/replicator_safety_upgrade/visual_placement/previews_final01/`；网页也包含先前补拍图和原图的直接链接。

## 当前执行顺序

1. 使用已有154张实际渲染图片粗筛，标注位置、人体朝向、遮挡和操作空间问题。
2. 优先修复人体伸手方向和钢夹架偏离操作区的共性问题；看不清的先补视角。
3. 原任务有完整成功基线时，复用真实关节、夹爪、携物状态，批量检查新增实体的源碰撞几何。
4. 再运行原生物理和安全规划，保存真实执行视频。缺少基线的任务继续等待实际验证，不用推测路径替代。

截图只用于粗筛。“候选可保留”不等于风险成立；单条基线风险成立也不等于有安全可行解或真实操作成功。

## 已有结果

- 原始场景清单和加载结果保留：`reports/replicator_scene_build/progress.json`。
- 原154场景图像初筛：`reports/replicator_safety_upgrade/visual_triage/summary.json`。
- 逐场意见：同目录的 `review_*.json`，每条绑定实际截图的SHA256。
- 每任务11图总览：同目录的 `contact_sheets/`。
- 成功基线清单：`reports/replicator_safety_upgrade/baselines.json`。
- 新轨迹风险摆位：`configs/extensions/replicator/safety_scene_variants/`。
- 逐任务风险证据与失败候选：`reports/replicator_safety_upgrade/tasks/`。
- 实际运行与失败记录：`reports/replicator_safety_upgrade/baseline_runs/`。

新摆位不能沿用原位置的图片审查结论。网页把原图审查、几何筛选、单基线风险、原生稳定、安全执行和视频分别显示。

## 自动CPU风险筛选入口

在仓库根目录运行：

```bash
python3 scripts/extensions/replicator/run_safety_upgrade.py --workers 2 --resume
```

该入口通过现有 `scripts/arena_python.sh` 使用正确容器解释器，核验成功基线后按原任务并行筛选，持续更新网页。只有具备有效成功基线的任务才进入轨迹风险筛选；缺少基线不会被自动标成通过。

仅聚合已落盘结果：

```bash
python3 scripts/extensions/replicator/run_safety_upgrade.py --refresh-only
```

监督器使用独占锁；已有监督器运行时不要重复启动。运行日志在 `reports/replicator_safety_upgrade/runs/`，失败候选与旧报告保留。CPU几何筛选不占用Kit或GPU；实际RTX渲染、cuRobo和原生物理由Agent统一调度。

## 已支持的真实执行入口

目前以下适配入口支持G型芥末瓶任务的11种新增实体。其他原任务必须使用各自适配器，不能只替换任务ID。

```bash
# 不执行物理：检查转换和匹配基线，输出具体采集命令。
bash scripts/arena_python.sh scripts/extensions/replicator/run_safety_scene_trial.py \
  --scene configs/extensions/replicator/safety_scene_variants/g_shape_mustard_bowl/property_tall_glass.json \
  --attempt example_preflight

# 执行真实采集：使用新的attempt，不覆盖上一次记录。
bash scripts/arena_python.sh scripts/extensions/replicator/run_safety_scene_trial.py \
  --scene configs/extensions/replicator/safety_scene_variants/g_shape_mustard_bowl/property_tall_glass.json \
  --attempt example_execution --execute
```

默认保存RGB与总览视频，使用共享GPU锁。采集器运行成功仍须通过独立物理、源几何和数据检查；未补全本场中性保持等证据时不会自动标记最终验收通过。当前不是200条批量数采入口。
