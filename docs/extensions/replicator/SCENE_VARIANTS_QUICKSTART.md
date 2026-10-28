# Replicator：14 × 11 场景搭建

本批覆盖 A 组全部 14 个原始任务，每个任务使用原范例的 11 种实体，共 154 个场景。半岛式厨房没有香蕉入碗任务，未补造新任务。

## 查看进度

打开 <http://127.0.0.1:8765>。顶部场景矩阵每 3 秒刷新；点击格子查看配置、几何检查报告、实际加载报告和预览图。场景加载进度与原有轨迹采集、避障验收进度分别统计。

```bash
python3 scripts/extensions/replicator/progress_dashboard.py --port 8765
```

## 场景文件

- 配置及完整清单：`configs/extensions/replicator/scene_variants/manifest.json`。
- 单场景配置：`configs/extensions/replicator/scene_variants/<task>/<variant>.json`。
- 可打开的 USD：`reports/replicator_scene_build/scenes/<task>/<variant>/scene.usda`。
- 实际 Isaac Sim 预览：同目录的 `preview.png`。
- 加载证据：同目录的 `simulation_load.json`。
- 逐对象可见性复查后的版本保存于 `scenes_v2/`、`scenes_v3/` 和 `scenes_v4/`；`reports/replicator_scene_build/progress.json` 及网页每格指向该场景最新检查通过的版本。
- CPU 摆位检查：`reports/replicator_scene_build/cpu/`。

11 种实体包括：MPFB 人体；高水杯、高脚酒杯、玻璃瓶、陶瓷杯、花瓶、平板支架；钢制夹架、突出横杆、狭窄门框、尖角设备外壳。

## 补齐配置与重新加载

使用已经安装好 Isaac Lab Arena 的本地运行环境。仓库现有容器入口如下；其他机器应按本机配置设置 `ARENA_CONTAINER`。

```bash
# 补齐尚未生成的配置，并检查原始任务关系、支撑面及新增实体初始几何
# 已成功生成的配置会复用，不会覆盖重建
bash scripts/arena_python.sh scripts/extensions/replicator/build_scene_variants.py

# 在 Isaac Sim 中逐场加载、渲染，更新网页进度
bash scripts/arena_python.sh scripts/extensions/replicator/build_scene_previews.py --device cpu --resume

# 检查每个任务物体是否出现在画面中；必要时只调整总览相机
bash scripts/arena_python.sh scripts/extensions/replicator/retry_scene_visibility.py \
  --device cpu --output reports/replicator_scene_build/visible_review --resume

# 只加载一个场景
bash scripts/arena_python.sh scripts/extensions/replicator/build_scene_previews.py \
  --device cpu --task g_shape_banana_bowl --variant property_tall_glass
```

`--device cpu` 指定仿真设备选项；RTX 渲染仍需要 GPU。配置生成与批量加载应依次运行，避免同时写进度文件。生成器默认跳过清单中 `checks.config_generated: true` 的场景；若需要修复初始机器人碰撞，可使用 `build_scene_variants.py --repair-initial-robot <检查报告.json>`，仅重建报告 `failed_scenes` 列出的场景。修改配置后，应重新加载对应场景；`--resume` 会核对配置、场景、预览与源资产哈希，失配时重新渲染。

## 本阶段完成范围

配置保留各原任务的厨房、操作对象、目标关系、机器人基座及支座高度。新增实体按各任务的操作区域摆放，使用实际资产碰撞几何和支撑面检查，不复用另一厨房的绝对坐标。

“已加载”表示场景在 Isaac Sim 中实际打开，机器人关节结构、原任务刚体和新增实体碰撞结构存在，并已生成渲染预览。本批使用单独的总览相机预览场景，不替代原数采相机。

本阶段不执行机器人任务，也不推进完整物理仿真；原 USD 的机器人姿态与采集控制器的初始姿态需区分。长期静置稳定性、真实操作成功、cuRobo 避障必要性和数据采集均须在后续独立验证，不能由“154 个场景已加载”推导为验收通过。

可见性复查使用实际渲染的实例像素，要求机器人、新增实体及每个原始任务物体各至少 20 像素；人体包围框完整落在画面内。物体仍可能被部分遮挡，这不是无遮挡摄影或物理成功验收。相机修改会逐场核对非相机 USD 内容完全不变。`audit_scene_visibility.py` 保留第一轮相机诊断历史，重新检查请使用上面的室内视角修正版入口。
