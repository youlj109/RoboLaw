# 原生任务的离线外观随机化与视角

该接口只改变离线画面的光照、表面颜色与粗糙度，不改变物理摩擦、恢复系数、碰撞几何或轨迹。位置/姿态变化必须在在线采集之前施加并保存实际结果；离线改变外观不能计为新的独立物理轨迹。是否需要随机化物理材质仍待用户确认。

## 建立本条任务的配置

先对本条冻结场景运行 `scripts/catalog_native_appearance.py --help` 所示的资产目录命令。目录会列出灯光、支持的 shader、实际生效的 Material 和可见绑定几何。只使用已确认的精确路径，不能套用其他厨房的路径。当前支持 OmniPBR 与 UsdPreviewSurface 的既有常量输入；贴图覆盖、连接输入、动画输入、未生效的 shader 或没有可见几何的材质会被拒绝。

外观配置结构示例（占位符需要替换，数值范围需按任务确认）：

```json
{
  "schema_version": "robolaw-native-appearance-1",
  "task_sha256": "当前 task_contract 的规范化 digest",
  "semantics": "visual_only",
  "lights": [
    {"path": "/精确灯光路径", "intensity_multiplier": [0.8, 1.1], "temperature_k": [4500, 6000]}
  ],
  "shaders": [
    {"path": "/精确生效Shader路径", "base_color_rgb": [[0.5, 0.7], [0.6, 0.8], [0.6, 0.8]], "roughness": [0.1, 0.2]}
  ]
}
```

任务 digest 使用 `kitchen_safety.task_contract.digest`，不是 JSON 文件字节哈希。所有目标先完成验证，再写入 USD session layer；原始 reconstruction 文件保持不变。每个属性用 seed、路径和参数名派生独立随机流，记录实际采样值、配置、Python/RNG 身份及来源哈希。

## 渲染与导出

```bash
python scripts/risk_capture_workflow.py render --output "$RUN" --appearance-config /absolute/appearance.json --appearance-seed 1001 --camera-config /absolute/cameras.json
python scripts/risk_capture_workflow.py video --output "$RUN"
```

两个 appearance 参数必须同时出现，只允许 render 阶段使用。不传时保持原始外观。所有新渲染使用新输出目录，不能覆盖历史结果。多外观版本可用 `scripts/render_native_task.py` 对同一条原始证据输出到不同目录。

`appearance_report.json` 保存实际抽样值、来源 raw/reconstruction 哈希、参数范围、可见绑定几何。`render_report.json` 额外保存该报告的完整文件 SHA256。视频导出检查这些身份并复制外观与相机配置；旧版缺少报告哈希的外观渲染需重新渲染到新目录，不能补写历史报告伪装成原始结果。

相机配置为 `robolaw-native-camera-1`，绑定同一个 `task_sha256` 与本条 `source_raw_sha256`，`cameras` 必须包含三个条目：`eye`、`look_at` 均为世界米制三元组，`focal_length_mm` 为焦距。它们是离线概览相机，不能冒充策略在线观测。世界相机配置只适用于经过检查的布局；随机化布局需要重新选择或推导视角。

## 验收边界

- 图像非黑、像素非空仅证明渲染输出存在，不能证明任务物体可见；必须检查遮挡、目标入镜和运动帧更新。
- 外观随机化记录成功不等于每一种材质在图像中都出现明显变化；透明资产、遮挡和贴图都可能限制可见效果。
- 离线视觉增强复用原始运动与安全标签，不产生新的动作成功证据，也不改变物理风险等级。
- 如果任务策略依赖图像，采集时仍须保存策略实际收到的在线图像。不能把后来增强的 RGB 声称为策略实际输入。
- 2026-10-24 的两种 seed 已完成同一玻璃杯短探针的四帧渲染，但默认相机存在遮挡，未通过完整视频视角验收。该证据不代表 32 项任务或全部材质随机化已验收。

后续明确视角修正：seed1001 在新的输出目录完成四帧渲染，报告零物理推进，最大位姿矩阵误差 2.22e-16；人工检查首帧，三路均能看到瓶子、碗和新增高杯。该检查仅适用于这一条静止短探针，不能替代完整运动及其他随机布局的视角验收。

seed1002 的相同明确视角也已完成并检查首帧；与 seed1001 使用相同原始轨迹和相机。实际 `video` 阶段导出的 MP4、appearance_report、camera_config 和 render_report 均与渲染目录逐文件 SHA256 一致。
## 指定时刻相机预览

直接调用 `scripts/render_native_task.py` 时，可用 `--preview-action-indices 0 350 761 782` 只渲染本条数据中明确选择的源时刻。索引必须唯一并存在于该条数据中；示例数值不能直接套用短轨迹。仍逐条核对原始观测与标签的动作/物理步身份，选中帧使用实测位姿，物理步不推进。

预览输出 `preview.mp4`、`source_frame_XXXXXX.png` 和明确的 `preview_only: true` 报告；字幕注明不是连续视频。报告保留源时刻列表、全部核对的样本数及渲染器哈希。工作流拒绝将它作为完整视频导出。先检查目标可见性、遮挡和曝光，再在新目录去掉此选项渲染全轨迹。预览通过不等于全部帧已通过视觉验收。
