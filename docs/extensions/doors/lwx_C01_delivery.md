# lwx：C01 基础冰箱开门交付

本次范围是 `kitchen_bench_lightwheel_open_fridge`，以 `docs/student_assignments/README.md`、`C_PROMPT.md`、`student_skills/robolaw-doors/SKILL.md` 和 `references/review-gates.md` 为准。个人分支 `lwx` 基于 `origin/ylj` 的 `a5ab6fe5d45d6eb75ca62c51754aba07bc437e64`，包含要求的 `76de5bb`；作者 `WinshareLi <22632139@zju.edu.cn>`。

机器报告及文件校验见 [gate_summary.json](lwx_20261023/gate_summary.json)、[artifact_hashes.json](lwx_20261023/artifact_hashes.json)、[asset_provenance.json](lwx_20261023/asset_provenance.json)。各结论来自独立报告，不从历史 `accepted=true` 推定。

## 实测交付

宿主独立克隆：`/home/youlj/projects/wenxuanli/RoboLaw`。本人容器：`robolaw-wenxuanli`，挂载该目录为 `/workspace`。以下数据路径相对此仓库；原始数据、图像和视频保留在远程磁盘，**不随 Git 上传**。

| 类别 | 正式 episode | 动作/RGB 帧 | 连续物理样本 | 释放后门角 rad | 抓点高偏移 m |
|---|---|---:|---:|---:|---:|
| 无新增风险基线 | `datasets/doors/lwx_20261023/base_03` | 366 | 2929 | 0.32023990 | +0.12 |
| 人体 | `datasets/doors/lwx_20261023/human_01` | 359 | 2873 | 0.31829816 | +0.18 |
| 财产 | `datasets/doors/lwx_20261023/property_01` | 359 | 2873 | 0.31738213 | +0.18 |
| 障碍物 | `datasets/doors/lwx_20261023/robot_01` | 360 | 2881 | 0.31773248 | +0.12 |

原门关节限位为 0 至 π/2，仍要求原始顺序谓词：先离开静置位，再严格开放度 `>0.2`（门角严格 `>0.3141593 rad`）。关闭自动成功重置只用于保留终态；判定直接调用原任务谓词。机器人通过实际关节控制、合指和拉门完成任务，包含松开把手及释放后观察。没有向目标门写关节/位姿、施加焊接约束或修改门驱动。

动作和 RGB 为 15Hz；机器人 q/dq、门和背景机构 q/dq、原始双指接触载荷/点/法线/分离量为 120Hz。物理记录从源 `env.reset()` 后第一状态开始，新增启动保持也全部记录，不删掉沉降期。源 reset 内部过程不在此记录范围。除每个物理子步外，外部安全检查还包含每动作结束复查；这些重复复查不冒充新增物理样本。

每个正式 episode 保留：

- `trajectory.json/.h5`、`physics.json`、`images.h5`、`metadata.json` 和原执行审计。
- 原 `external_camera.mp4`、`external_camera_2.mp4`、`wrist_camera.mp4`，均 1280×720、15fps。
- `all_cameras.mp4` 为原三路组合，只向同学展示组合文件不影响三路原始交付。
- 三变体另有 `baseline_comparison.mp4`：原速、同一原相机 `external_camera_2` 左右对照；较短一侧结束后明确标注末帧保持，没有时间拉伸。
- 对照左侧新增实体只是**离线反事实图示**，危险旧路径未在增广场景物理回放。金色为投影源碰撞网格凸轮廓/primitive 边界，青色为最近机器人部位，粉色虚线为轴对齐边界加裕度的说明图，不把二维轮廓等同于精确距离检验。

所有原视频和组合视频均完整解码，RGB JPEG 逐张解码、计数、原尺寸及相机位姿/状态对齐通过。原始渲染另有逐帧空间变化检查。人工视觉复核查看了三个变体四阶段三相机联系表和原速对照关键帧。人体背部遮挡 `external_camera`，`external_camera_2` 能看到手部、机械手与把手；保留原视角及该遮挡事实。

## 新验收证据

四组 `neutral_{base,human,property,robot}_01` 使用相同源资产/reset 和对应参数，均记录 40.6s、4873 个连续物理状态及原三路视频，覆盖正式操作 23.93–24.4s。观察 20 个活动厨房 articulation 的全部 48 个 DOF；操作回合只豁免被操作门，背景 DOF 仍逐步检查。预声明容差在 `configs/extensions/doors/review_gates_lwx.json`，未事后放宽；未锁死或修补源机构。自由财产物体的重力落座从最早状态开始保留，正式回合最大平移 1.6544mm，低于预声明 5mm，旋转/倾角低于 2°。

`review_audit.json` 检查加载接触出现在首次可分辨 2mrad 门运动**之前的物理子步**，拉门阶段双指接触计数持续存在、每子步至少一指有正常载荷，最终已经释放且原开门谓词成立。单指零法向冲量但接触仍存在时独立列出，不伪称丢失接触，也不宣称两指始终均承载。`audit_contact_geometry.py` 另用真实移动门体姿态，将原始加载接触点定位到源把手碰撞网格，使用预声明 4mm 容差。

因果证据不能解析同一物理子步内部先后；回归测试确保这种情形拒绝通过。未测切向摩擦，源三角面/hull 模型未被声称与 PhysX cooked shape 完全一致。

`validate_trial.py`、锁内 `audit_self_collision.py`、`audit_avoidance.py`、`audit_stand.py`、`audit_source_layout.py` 均独立通过。源 YAML SHA256 为 `679575de18414f7ccb49a5d43bcb2e1b952eb4c82c680ab79c55734aa638d3ee`。原 On/NextTo 关系保留，本次各实际布局日志有关系/重叠/可达性通过记录；实际源 base/yaw 与冻结源解逐项一致，源组合机器人/台架文件哈希相同。

本组 world 新纳入真正的源台架；源第一段粗胶囊向安装面下延伸约 75mm，会假报机器人自身安装处碰撞。只排除这段**非物理安装包络与自己的台架**之间的一对代理检查，真实机器人 hull/手指仍以原 2mm 检查台架，代理与其他实体仍保留。`stand_audit.json` 使用全程 120Hz 实测状态独立检查真实机器人和运动门体对台架。另有作用范围回归测试，避免此例外扩散到真实 mesh 或外部风险实体。

| 变体 | 原基线加入实体后的源几何距离 | 新真实执行最小距离 | 原执行门槛 | 证据类型 |
|---|---:|---:|---:|---|
| human | 相交 | 42.855mm | 20mm | 同一把手合法抓点适配 |
| property | 39.820mm | 100.484mm | 50mm | 同一把手合法抓点适配 |
| robot | 1.273mm | 42.556mm | 10mm | 同一抓点的固定目标重规划 |

财产安全距离不足不表述为必然破碎；反事实检查使用实际机器人碰撞 hull、各手指网格及原实体源网格/真实 primitive，而不是仅用保守盒或胶囊建立必要性。初始支撑、实体与源环境、运动门扫掠、台架及支座均单列检查。

三个变体在预声明 seed42/43/44 中，每个 seed 均用相同起态、源厨房、台架、控制参数和候选策略，只切换新增实体及其实际支撑的存在。所有开/关规划成功，关障路径放入实体后存在源几何安全距离违规，开障路径通过源几何及保守执行检查。保存全部候选拒绝、路径点、规划耗时、路径长度/时长和违规部位/阶段。

- human/property：三对均从 +0.12m 改选 +0.18m；另做选中点固定后的开/关规划对照。支持所测种子和策略范围内的**抓点适配必要性**，不写成固定终点绕行。
- robot：三对均保持 +0.12m，支持所测种子和策略范围内的**固定目标重规划必要性**。
- 没有人工加绕行路点，没有物理执行危险关障路径；三种子是规划实验，不是三条真实采集，更不证明所有可行路径都受阻。

完整成对证据：`reports/doors/lwx_20261023/paired_{human,property,robot}_01/{seed_42,seed_43,seed_44,report}.json`。11 项回归测试在实际容器通过。Windows 本地只具备其中 9 项所需依赖，新增碰撞作用范围测试需要远程已有的 scipy，没有为此改动本机环境。

## 复现命令

先确认容器仍挂载本人目录且不存在其他 GPU 工作。不要重建、删除共享锁。以下在**宿主机**进入本人容器：

```bash
cd /home/youlj/projects/wenxuanli/RoboLaw
docker exec -it -w /workspace robolaw-wenxuanli bash
```

以下在**容器内**执行。每次复现使用新的输出目录，报告及数据不得覆盖旧回合。

```bash
export PYTHONPATH=/workspace HEADLESS=1
unset ISAACSIM_ROOT_PATH ISAACLAB_PATH OMNI_SERVER
PY=/workspace/.venv-arena/bin/python
LOCK=/workspace/reports/skill_trials/gpu.lock
OUT=datasets/doors/lwx_reproduction_01
REP=reports/doors/lwx_reproduction_01
mkdir -p "$REP"

"$PY" student_skills/robolaw-doors/scripts/preflight.py --repo /workspace
"$PY" -m unittest discover -s tests/extensions/doors -p 'test_*.py' -v

for variant in base human property robot; do
  flock -n "$LOCK" "$PY" scripts/extensions/doors/collect.py \
    --device cpu --mode neutral --neutral-seconds 40 --max-steps 800 \
    --variant "$variant" --adaptive-handle-height \
    --variant-config configs/extensions/doors/avoidance_variants.json \
    --output "$OUT/neutral_$variant" >"$REP/neutral_$variant.log" 2>&1 || break
  "$PY" scripts/extensions/doors/audit_review.py "$OUT/neutral_$variant" || break
  flock -n "$LOCK" "$PY" scripts/extensions/doors/collect.py \
    --device cpu --mode collect --max-steps 500 \
    --variant "$variant" --adaptive-handle-height \
    --variant-config configs/extensions/doors/avoidance_variants.json \
    --output "$OUT/$variant" >"$REP/$variant.log" 2>&1 || break
  "$PY" scripts/extensions/doors/validate_trial.py "$OUT/$variant" || break
  flock -n "$LOCK" "$PY" scripts/extensions/doors/audit_self_collision.py "$OUT/$variant" || break
  "$PY" scripts/extensions/doors/audit_review.py "$OUT/$variant" --neutral "$OUT/neutral_$variant" || break
  "$PY" scripts/extensions/doors/audit_contact_geometry.py "$OUT/$variant" || break
  "$PY" scripts/extensions/doors/audit_stand.py "$OUT/$variant" || break
  "$PY" scripts/extensions/doors/audit_source_layout.py "$OUT/$variant" --logs "$REP" || break
  if [ "$variant" != base ]; then
    "$PY" scripts/extensions/doors/audit_avoidance.py "$OUT/$variant" --baseline "$OUT/base" || break
    flock -n "$LOCK" "$PY" scripts/extensions/doors/paired_planning.py \
      --baseline "$OUT/base" --variant "$variant" --output "$REP/paired_$variant" || break
    "$PY" scripts/extensions/doors/make_comparison.py "$OUT/$variant" --baseline "$OUT/base" || break
  fi
done
```

`--device cpu` 是物理设置，RTX 渲染和 cuRobo 仍使用 GPU，必须持有同一宿主文件映射的锁；这里的本组采集器没有内部持锁，外层 `flock` 只加一次。退出非零时保留失败证据，诊断后用新目录重试，不继续计为通过。

人体恢复遵循原入口 `build_reposed_human.sh`，本次使用官方 Blender 4.0.2。重建文件来源见新增 [artifact_manifest_lwx.json](../../../assets/extensions/doors/human_reach_elbow_up/artifact_manifest_lwx.json) 及 [feet_geometry_audit_lwx.json](../../../assets/extensions/doors/human_reach_elbow_up/feet_geometry_audit_lwx.json)。原 `artifact_manifest.json` 没有改写。脚/root 独立导入检查的 2948 个低于 20cm 顶点，双向最大差为 0；姿态报告、wrapper 和贴图哈希与原清单一致。

## 保留的失败与未测项

初始化诊断、失败基线、所有旧审计和日志保留在 `datasets/doors/lwx_20261023` 与 `reports/doors/lwx_20261023`，机器汇总列出每个候选，没有挑去失败。

- `diagnostic_boot_01`：本人容器 CUDA/NVML 不可用，未计成功；确认没有他人进程后重启本人容器恢复，未动其他容器或驱动。
- `diagnostic_boot_02`：原生 D6 多自由度关节分类缺失，修复观察器后重试。
- `diagnostic_boot_03`：1 秒中性入口诊断，不能代替正式 40 秒对照。
- `base_01`：上述非物理安装包络假碰撞，拒绝。
- `base_02`：原生开门成功，但新审计拒绝。初次接触定位使用的 authored USD extent 与真实碰撞顶点范围不符；改用源 mesh 并增加独立距离审计。双指接触和单指零法向载荷也分别报告。没有覆盖旧报告；使用修正记录器重新采集的 `base_03`。
- 人体 USDC **不与历史导出字节相同**；实际 SHA256 为 `20c0904268255e231952b311cd6277311d89431d9a186af695290217c60d85f1`。字节差异原因未确定，不推断为无关紧要的非确定性；实际文件、源文件及几何检查均有新清单，原期望哈希保留。
- 冻结源布局引用的历史 `base_attempt_08/metadata.json` 不在本克隆，历史字节溯源未复核。本次源 YAML/源组合机器人台架哈希、原关系验证、实际冻结位置及新真实基线已经独立核验。
- CPU USD 未启用 Kit 的 MDL 查找器，首次资产扫描将 `OmniPBR.mdl`/`OmniGlass.mdl` 记为未解析；保留该报告。第二版明确从当前 Kit 的 `kit/mdl/core/Base` 定位并哈希，76 个文件无缺失。原生 RTX 实际渲染和 RGB 检查另已通过。

| 任务 | 本次状态 |
|---|---|
| C01 基础冰箱 | 新基线、三条真实成功、四组中性对照、独立物理/视频审计、三种子成对规划完成；来源限制见上 |
| C02 U 型中岛 Farmhouse2 橱柜 | 下一任务，未实施/未验证 |
| C03 G 型右冷冻室 | 未实施/未验证 |
| C04 G 型左冰箱 | 未实施/未验证 |
| C05 U 型冰箱 | 未实施/未验证 |
| C06 基础微波炉 | 未实施/未验证 |
| C07 G 型微波炉 | 未实施/未验证 |
| C08 G 型烤箱 | 未实施/未验证 |

未做随机场景、11 类全覆盖、200 条批采或成功率实验。其余任务不能仅替换首任务名字后宣称可运行。
