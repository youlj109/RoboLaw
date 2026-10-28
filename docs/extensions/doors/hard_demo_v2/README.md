# 人身安全 demo v2：自然伸手与走近／退开

本次只修订人身安全场景。财产安全与流水场景沿用 v1 的原视频、资产和配置。

## 修订内容

- 恢复人物自然直立姿态，调整朝向，使其正面走向冰箱。
- 用原人体骨架逐帧计算肩、肘、腕和双腿，再求值原蒙皮。消除旧版保持手掌绝对方向造成的腕折，以及两端网格插值造成的手臂缩短。
- 加入交替迈步、摆动脚抬升、支撑脚固定、身体起伏和轻微摆臂。人物沿斜线走近约 0.87 m，站稳、伸手、撤手，再退开；远端路线向厨房外侧留出 18 cm，避开灶台把手。
- 使用连续的肘弯方向，手掌跟随前臂。生成副本中的左右鞋底分别绑定原 foot 骨，高鞋帮平滑保留原小腿蒙皮，保留原网格和纹理，同时避免鞋底漂移与鞋帮翘出裤脚；原人体源文件不修改。
- 镜头扩大到全身，让脚步与手臂动作都能看到。

## 输出

- 场景：`assets/extensions/doors/hard_demo_v2/scene.usda`
- 人物逐帧 USD：`assets/extensions/doors/hard_demo_v2/person/person.usdc`
- 可编辑骨骼动画：`assets/extensions/doors/hard_demo_v2/person/animated_source.blend`
- 动作记录：`assets/extensions/doors/hard_demo_v2/person/animation.json`
- 视频：[14 秒新版人身 demo](../../../../exports/hard_demo_v2/01_human_walk.mp4)
- 合辑：[新版人身 + 原财产 + 原流水](../../../../exports/hard_demo_v2/three_demos.mp4)
- 场景来源、时间线、几何检查与渲染记录：`reports/doors_hard_demo_v2/`
- 最终交付哈希与解码检查：`exports/hard_demo_v2/manifest.json`

## 重建入口

从仓库根目录运行，使用现有 Blender 4.0.2 的 CPU 模式：

```bash
blender -b assets/people/mpfb_corridor/posed.blend --threads 4 --python scripts/extensions/doors/hard_demo_v2/animate_person.py
python3 scripts/extensions/doors/hard_demo_v2/build.py
python3 scripts/extensions/replicator/hard_demo_v1/export_animation.py --scene assets/extensions/doors/hard_demo_v2/scene.usda --output reports/doors_hard_demo_v2/animation_final.json.gz --fps 12 --frames 168
blender -b --threads 6 --python scripts/extensions/replicator/hard_demo_v1/render_animation.py -- --animation reports/doors_hard_demo_v2/animation_final.json.gz --output-dir reports/doors_hard_demo_v2/frames --samples 4 --threads 6 --width 960 --height 720
python3 scripts/extensions/replicator/hard_demo_v1/package_video.py --frames reports/doors_hard_demo_v2/frames --timeline reports/doors_hard_demo_v2/timeline.json --output exports/hard_demo_v2/01_human_walk.mp4 --title '01 人身安全 · 走近、伸手与退开'
python3 scripts/extensions/doors/hard_demo_v2/package_release.py
```

构建器默认拒绝覆盖已有场景；有意重新生成同一版本时使用 `--force`。依赖原人体、厨房和固定机器人资产均由来源报告记录。生成器中记录的建议站位与实际场景站位应分别读取：实际位置以 `source.json` 和场景 USD 为准。

## 验证范围

这是人物与门的脚本动画，机器人保持静止，没有生成机器人轨迹，没有进行模型判断或原生物理验收。骨架检查、实际鞋网格的接地检查、每个视频帧的实体几何检查和视觉检查分别记录，不能用脚骨不移动替代实际鞋底接地检查。

最终 168 帧的离散几何检查全部通过：人物与当帧门板、机器人和周围厨房均保留间距，鞋底无穿地。最近厨房间距约 21.67 mm；保持伸手姿势、让门继续开到 25° 的离线反事实会碰到手部。独立动作审查确认肘膝弯曲方向连续、骨长不变、支撑鞋底稳定。原有带仿射变形的圆柱把手采用有误差上界的保守外接柱面，具体方法和原始数值见 `geometry.json`。

视频 12 fps 的每一帧都来自当帧骨骼求值结果。更高频率下的连续运动、人体动力学及原生物理接触仍未验证。门暂停和继续打开属于演示脚本，不代表模型自动执行。

`camera_preview*` 是用旧 v1 姿态测试全身机位的诊断文件；`animation_initial_ik*` 是修正肘弯分支前的诊断导出；`animation_straight_route*` 和 `geometry_straight_route.json` 记录修正灶台碰撞前的直线走路方案。它们均不是 v2 最终视频输入。

最终视频只使用 `animation_final.json.gz`。`animation_route_v2*`、`keyframes_route_v2/` 和 `geometry_rigid_shoe.json` 保留鞋帮外观修正前的检查过程，不能代替最终输入。
