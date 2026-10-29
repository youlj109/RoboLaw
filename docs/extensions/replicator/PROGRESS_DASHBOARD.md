# A 组本地实时进度页

地址：[物理执行与实际视频](http://127.0.0.1:8765/#physicalWorkspace)、[场景看图标注](http://127.0.0.1:8765/#annotationWorkspace)。已接入 14 个任务 × 11 个变体，共 154 个场景。进度每 3 秒自动刷新，只读实际报告和本机进程，不启动或停止采集，不占用 CUDA。

**无需启动本机服务即可查看：[最新 154 景结果与诊断](PHYSICAL_RETRY_ROUND13_20261029.md) · [最新结果 JSON](../../../configs/extensions/replicator/physical_retry_results_round13_20261029.json) · [保留的首轮结果](PHYSICAL_EXECUTION_154_20261028.md)**。GitHub 保存状态与证据 SHA；实际视频单独打包，本机面板及视频路径不是云端播放链接。

## 第十三轮物理重试：人体放碗释放高度

**第十三轮已完成：1 景实际成功、0 失败；当前采用版本154/154，原布局151/154。** U 型放碗／人体沿用第八轮布局，lower终点提升25mm至第十二轮已成功的中间点，并删去其后的失败下放段。其他阶段目标、抓姿、夹爪、retreat与原物理判据保持。本次续修的另一景是第十一轮已成功的芥末瓶／设备外壳，其结果不重复执行或计数。

在「重试轮次」选择第十三轮，查看本轮唯一实际episode与独立审计。22,029个原生物理状态确认原任务及最终范围、支撑、低速判据通过。执行未开启渲染，本轮无失败诊断图或新视频；播放器仍显示明确标记轮次的历史视频。154/154不是安全避障或长期稳定认证。

| 内容 | 保存位置 |
| --- | --- |
| 冻结计划 | `configs/extensions/replicator/physical_retry_round13_20261029.json` |
| 终态清单与原生状态 | `exports/replicator_physical_retry_round13_20261029/manifest.json`、`executions/` |
| 独立审计 | `reports/replicator_physical_retry_round13_20261029/audit_execution.json` |
| 独立人工物理标注 | `reports/replicator_dashboard/physical_retry_round13_annotations.json` |

标注接口 `GET/POST /api/physical-retry-round13-annotations` 与历史轮分开保存，实际结果SHA一致才可保存。只读核对发布来源：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round13.py --dry-run
```

完整矩阵和逐景记录见[第十三轮报告](PHYSICAL_RETRY_ROUND13_20261029.md)。第十二轮规划失败、第十一轮启动恢复及全部历史来源继续保留，原视频包与SHA不变。

## 第十二轮物理重试：人体放碗下放路点

**第十二轮已完成：1 景规划失败、0 次新增物理执行；当前采用版本153/154，原布局151/154。** 本轮只处理 U 型放碗／人体，沿用第八轮布局、原抓姿及六阶段最终 TCP。lower 前9段共351个实际返回路径点通过独立源 FK 复核，最后一段 TRAJOPT_FAIL，没有返回路径。第十一轮芥末瓶成功与全部历史保持。

在「重试轮次」选择第十二轮，查看本轮冻结路点、成功前缀和规划失败建议。`retry_round12.result=null`、`attempts=0`、`executed=false`；历史视频与实际结果仍明确标记所属轮次，不当作本轮执行。本轮没有实际失败，无新增诊断图或执行视频。

| 内容 | 保存位置 |
| --- | --- |
| 冻结计划 | `configs/extensions/replicator/physical_retry_round12_20261029.json` |
| 终态清单 | `exports/replicator_physical_retry_round12_20261029/manifest.json` |
| 独立审计及规划失败诊断 | `reports/replicator_physical_retry_round12_20261029/audit_execution.json`、`planning_failure_diagnostics.json` |
| 独立人工物理标注 | `reports/replicator_dashboard/physical_retry_round12_annotations.json` |

标注接口 `GET/POST /api/physical-retry-round12-annotations` 与历史轮分开保存；本轮没有实际结果，物理标注保存保持禁用。只读核对发布来源：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round12.py --dry-run
```

完整矩阵和逐景诊断见[第十二轮报告](PHYSICAL_RETRY_ROUND12_20261029.md)。已有正式输出不自动覆盖，旧报告、标注、视频及原包SHA保持。

## 第十一轮物理重试：求解策略修复

**第十一轮已完成：1 景实际物理成功、1 景规划失败、0 实际失败；当前采用版本153/154，原布局151/154。** 新成功是 U 型芥末瓶／尖锐设备外壳，独立原生状态复核确认原任务及最终范围、支撑、低速判据通过。U 型放碗／人体在 lower 返回 TRAJOPT_FAIL，失败段没有路径，零次本轮物理执行。两景仍采用第八轮布局，没有修改障碍摆位。

在「重试轮次」选择 **第十一轮 · 求解策略修复（2 景）**。芥末瓶本轮结果是一次无画面物理执行，上方仍为历史视频；本轮没有实际失败，不新增失败图或执行视频。人体 `retry_round11.result=null`、`attempts=0`、`executed=false`，显示绑定本轮候选的规划诊断，不可用旧图冒充本轮执行。

芥末瓶启动进程曾退出139。恢复前manifest原始字节、日志、源代码及目录检查均已保存，审计确认目标场景episode尚未开始；START记账标记不是物理episode证据。恢复后只执行原冻结候选一次，启动失败消耗0个物理episode。保留该记录不等于确认SciPy、protobuf或其他单一组件为根因。

| 内容 | 保存位置 |
| --- | --- |
| 本轮冻结计划 | `configs/extensions/replicator/physical_retry_round11_20261029.json` |
| 本轮终态清单 | `exports/replicator_physical_retry_round11_20261029/manifest.json` |
| 原生审计及规划失败诊断 | `reports/replicator_physical_retry_round11_20261029/audit_execution.json`、`planning_failure_diagnostics.json` |
| 启动恢复来源复核 | `reports/replicator_physical_retry_round11_20261029/pre_episode_startup_failure_01_review.json` |
| 独立人工物理标注 | `reports/replicator_dashboard/physical_retry_round11_annotations.json` |

标注接口为 `GET/POST /api/physical-retry-round11-annotations`，与前十轮分别保存；只允许绑定有效本轮实际结果的标注，不自动重新执行。人体规划失败没有本轮实际结果，查看规划诊断即可。

第十轮记录仍可从轮次选择器查阅：两景规划失败、0物理。芥末瓶JS路径的终点准确但中途旋转约21.215度，硬门槛正确拒绝；第十一轮改回有姿态／竖直软代价的笛卡尔lift，权重1并启用graph，保留8mm／4度／-2mm硬门槛。任务通过不等于安全避障验收。

最终结果已发布；只读核对来源用：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round11.py --dry-run
```

完整矩阵、各景结果与启动恢复记录见[第十一轮报告](PHYSICAL_RETRY_ROUND11_20261029.md)。已存在的正式输出不会自动覆盖，前十轮报告、标注、视频及原包SHA均保留。

## 第九轮物理重试：保留布局的轨迹优化

**第九轮已完成：2 景规划失败、0 次新增物理执行，当前采用版本仍为 152/154，原布局保持 151/154。** 本轮保留第八轮配置、源 USD 与场景版本，不调整摆位。芥末瓶通过 pregrasp／approach，在首个 lift 段优化失败；人体通过前四阶段，在 lower 返回 IK_FAIL。两景独立审计及只读诊断已完成，没有新增执行视频或诊断图。

物理面板的「重试轮次」选择第九轮，分别显示规划状态、成功前缀、后续建议及零物理尝试。本轮 `retry_round9.result=null`、`attempts=0`、`executed=false`。同一采用布局已有的实际结果不被清空：芥末瓶 `current_revision_result` 仍为第八轮实测失败，附 `current_revision_result_retry_round=8` 与 `current_revision_result_scope=latest_physical_attempt_on_retained_layout`，不是第九轮执行。上方播放器及第八轮诊断图均为明确标注的历史记录。

| 内容 | 保存位置 |
| --- | --- |
| 两景冻结计划 | `configs/extensions/replicator/physical_retry_round9_20261029.json` |
| 规划与终态进度 | `reports/replicator_physical_retry_round9_planning_20261029/progress.json`、`exports/replicator_physical_retry_round9_20261029/manifest.json` |
| 独立审计与规划失败诊断 | `reports/replicator_physical_retry_round9_20261029/audit_execution.json`、`planning_failure_diagnostics.json` |
| 本轮独立人工标注 | `reports/replicator_dashboard/physical_retry_round9_annotations.json` |

标注接口为 `GET/POST /api/physical-retry-round9-annotations`，不会覆盖前八轮。本轮没有实际结果，物理标注保存保持禁用；查看规划诊断及后续建议即可。芥末瓶的抬升合同是相对实际 approach 末端 FK 的横移≤8mm、转角≤4°、下沉≤2mm；本轮失败段没有返回路径，不能标记为合同通过或拒绝，更不能当作已验证抓取修复。

结果已正式发布；需要只读核对发布来源时运行下列命令，不启动 Kit、物理或再次规划，也不覆盖已有文件：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round9.py --dry-run
```

逐景明细和完整 14 × 11 矩阵见[第九轮报告](PHYSICAL_RETRY_ROUND9_20261029.md)。原物理判据与历史记录保持；独立审计通过不表示两景任务通过，也不代表安全避障验收。

## 物理执行与实际视频

截至 2026-10-28，154 景规划已完整生成，**首轮真实物理执行已全部完成：70 成功、84 任务失败、0 执行错误，154 个真实视频**。顶部面板读取 `exports/replicator_154_physical_demos_20261028/manifest.json`，独立显示 planned、physical pending/running/success/failed/error 和有效视频数量。首轮每景执行一次；实际抓放失败保留原结果和视频，不计作规划失败，也不自动重跑。逐任务数据见 [14 个任务的首轮统计](RELEASE_154_PLANNED_SCENES.md#首轮物理执行统计)。U 型香蕉的 11 景相机补录也已完成（7 成功、4 失败），最终演示包已生成。

按场景选择可查看真实执行结果、失败原因和 overview 视频。初始化或录制错误单独显示，`executed=false` 表示没有开始执行，不标成抓放失败或成功。结果必须与 R10 的场景、配置、轨迹及 SHA 绑定；视频链接还需实际执行结果 SHA 和视频 SHA 验证通过。缺失或不一致显示待核，不用旧规划动画替代。

G 型香蕉人体的源网格相交、U 型碗窄口的底座支撑边界两项历史质量提示继续保留。实际任务成功也不自动解除这些提示或代表安全避障验收。人工标注和原轨迹界面保持独立。

首次执行、恢复和 `--dry-run` 的完整命令见 [154 景发布与输入恢复说明](RELEASE_154_PLANNED_SCENES.md)。最小输入包包含执行所需的 R10 progress、154 份 result/source_scene、各实际轨迹路径及六份相机来源证据；大型源资产仍须另行恢复，全部网页历史图片与人工标注不在包内。首轮及补录的原生执行审计已通过，最终演示包中的 165 个视频也已完整解码并通过文件校验。视频包为 `exports/replicator_154_physical_video_bundle_20261028.zip`，解压后打开 `index.html`；含 154 景主视频及 11 个原遮挡视频，详细大小和 SHA 见发布说明。

初始化启动次数与物理 episode 分开记录。L-island 香蕉人体景曾在首次 `env.step` 前停滞；旧 `initializing`、`executed=false` 记录保留，新初始化通过 `initialization_history` 绑定旧记录，实际结果写入独立子目录。这只恢复尚未开始的唯一 episode，不重跑已经执行的任务；额外初始化记录随视频包保留，不能替代该景的正式结果或真实视频。

已有批次在没有 running 项且上一进程退出后，推荐主机运行 `python3 scripts/extensions/replicator/run_physical_demo_batches.py --camera-overrides configs/extensions/replicator/physical_demo_camera_overrides_20261028.json`，按原始任务启动新 Kit 进程并使用 8 个 PhysX 线程。该协调器的 `--dry-run` 只核对待执行分组，不代替执行器的源文件、轨迹及资产 SHA 检查；调度器不会重试已经执行的失败场景或自动处理初始化恢复。

U 型香蕉 11 景因首轮相机被天花板遮挡，另在独立目录各真实补录一次。面板的首轮成功/失败计数与原视频入口保持不变，额外显示「相机补录已保存 N/11」。选择这些场景时，有效新视频可以默认播放，但明确标注「相机补录 · 第2次」，同时列出原首轮成绩与链接。补录结果与首轮、冻结计划、控制参数及视频 SHA 绑定才可访问；无论新录成败，最终演示包固定选择全部 11 个新视频，不按成绩择优。补录计划仅供本机已有物理证据使用，新克隆首次执行使用相机 override 配置即可。

## 首轮失败 84 景：单次调整与无渲染物理测试

本轮已完成：**36 景转为成功、48 景仍失败、0 执行错误，最新合计 106/154 景成功**。48 景已完成状态回放渲染和逐景视觉复核；自动观察、推断及后续修改建议与人工标注分开显示。详见 [本轮结果与全部失败诊断](PHYSICAL_RETRY_ROUND1_20261028.md)。

本轮只处理首轮的 84 个失败场景，每景冻结一份调整方案后实际测试一次。调整为原轨迹的速度、加速度与阶段停留时间，保留原场景、几何路径、夹爪指令和原始任务判据。执行时不启用相机或渲染，也不生成本轮视频；已结束的结果保留，不自动再试。

物理面板新增独立的「本轮重试」计数与筛选，读取 `exports/replicator_physical_retry_round1_20261028/manifest.json`。首轮 **70 成功 / 84 失败**和相机补录记录保持不变；「最新实际成功」合并首轮成功与本轮新增成功，尚未完成的场景保留此前实际结论。4 个已经做过相机补录的失败场景显示累计第 3 次物理 episode，其余为第 2 次，本轮均只尝试一次。

1. 用「本轮重试」筛选运行中、成功或失败，选择场景查看本次调整说明及实际执行结果。**上方播放器仍是首轮或明确标记的相机补录视频，不是本轮测试视频。**
2. 本轮测试结束后，仅对仍失败且有实际状态记录的场景生成默认 6 张关键阶段诊断图；可通过 `--frame-hints <JSON>` 额外加入失持等事件的实测状态，去重后最多 10 张。事件提示必须匹配该景实际结果与状态 NPZ 的 SHA，索引范围及阶段逐项核对，不做插值。图片读取实际保存的机器人、任务物体和厨房动态刚体姿态，在停止的时间轴上重建画面；不再运行抓放或推进物理步骤。图片带 `DIAGNOSTIC STATE REPLAY` 水印，是**实测状态重建图**，不能称为执行相机原始帧或新的成功证据。
3. 诊断图及报告保存在 `reports/replicator_physical_retry_round1_diagnostics_20261028/`。网页仅展示与本轮实际结果、状态 NPZ、metadata、物理背景及图片 SHA 匹配的内容；尚未生成显示等待，初始化未执行或缺少状态不会伪造画面。
   补充视角在 `reports/replicator_physical_retry_round1_diagnostics_detail_20261028/`，暗曝光补图在 `reports/replicator_physical_retry_round1_diagnostics_dark_20261028/`。只展示视觉复核引用并通过同一实际状态及 SHA 校验的补图；更换观察相机不改变场景布局。曝光标签区分请求参数与实际效果，遮挡限制也在逐景观察中说明。
4. 在「实际失败原因 / 后续调整方案」中填写观察，约 0.65 秒后自动保存，失败时可点击「保存 / 重试保存」。内容独立保存到 `reports/replicator_dashboard/physical_retry_annotations.json`，绑定实际结果与配置 SHA，不覆盖原场景或轨迹标注，也不触发再次执行。来源变化或并发编辑会拒绝覆盖并保留当前页面草稿；遇到冲突先复制草稿，再比较服务器内容。

本轮是独立结果集，原 154 景发布快照、首轮视频及已发布视频包不被改写。任务成功、诊断回放和安全避障验收仍分别记录。

## 第二轮剩余 48 景：抓姿与轨迹调整

**第二轮 48 景已结束：29 景各实际测试一次，其中 19 成功、10 失败、0 执行错误；另有 19 景规划失败，零次新物理测试。最新累计 125/154 景通过，剩余 29 景。** 每景只冻结一个抓点、抓姿、夹爪指令或轨迹目标调整候选，保持场景布局及原始任务判据。全部 48 景审计已通过，10 景实际失败已完成逐景视觉复核，19 景规划失败已完成绑定候选结果的只读 CPU 诊断。详见[第二轮完整结果与失败建议](PHYSICAL_RETRY_ROUND2_20261028.md)及[机器可读状态](../../../configs/extensions/replicator/physical_retry_results_round2_20261028.json)。

已生成 **190 张可用状态重建图：95 张主图、95 张有效补图**。逐景复核记录明确列出实际查看的 **164 个图像路径**，覆盖全部 10 景实际失败；可用图片总数不代表每张均已人工查看。补图只改变观察视角和曝光，仍使用同一次真实执行保存的物理状态，没有新增物理执行。原图遮挡、曝光及无法确认的接触原因保留在各景复核说明中。

在[物理执行面板](http://127.0.0.1:8765/#physicalWorkspace)的「重试轮次」选择 **第二轮 · 抓姿与轨迹（48 景）**，再用「所选轮次」筛选全部 48 景、等待规划、规划中、规划失败、待测、运行、成功、失败或错误。切回 **第一轮 · 时序优化（84 景）** 可查看历史结果；「首轮执行结果」仍筛选最初的 154 景。上方播放器仍播放首轮或明确标记的相机补录视频，第二轮无渲染测试没有新执行视频。

第二轮已有 **19 景规划失败，均为 `planning_failed`、`attempts=0`、`executed=false`**；这些景没有本轮物理结果或新诊断图，不能计为本轮实际抓放失败或成功。网页保留其上一轮实际失败，并显示新规划的失败阶段和原因。其余候选的待测、运行及实际结果分别统计；「最新实际成功」只合并已验证的新物理成功，不把规划生成或CPU诊断当作成功。

本轮来源彼此独立：

| 内容 | 保存位置 |
| --- | --- |
| 冻结的 48 景调整计划 | `configs/extensions/replicator/physical_retry_round2_20261028.json` |
| 新规划进度及候选结果 | `reports/replicator_physical_retry_round2_planning_20261028/progress.json` |
| 实际执行进度 | `exports/replicator_physical_retry_round2_20261028/manifest.json` |
| 第二轮人工物理标注 | `reports/replicator_dashboard/physical_retry_round2_annotations.json` |
| 实际失败的状态重建图 | `reports/replicator_physical_retry_round2_diagnostics_20261028/scenes/<scene_id>/` |
| 19 景规划失败的只读CPU诊断 | `reports/replicator_physical_retry_round2_20261028/planning_failure_diagnostics.json` |

第二轮的「实际失败原因 / 后续调整方案」约 0.65 秒自动保存，绑定第二轮实际结果及配置 SHA；接口为 `GET/POST /api/physical-retry-round2-annotations`。第一轮仍使用原来的 `physical_retry_annotations.json` 和 `/api/physical-retry-annotations`，同一场景跨轮次也不会覆盖标注。只有本轮实际结果已落盘且来源有效时才能保存物理标注；规划失败景没有本轮实际结果，查看其规划诊断及建议即可。来源变化、并发编辑或保存失败时保留草稿，先核对再重试，不会触发新规划或物理执行。

### 第二轮结束后的审计、渲染与发布命令

以下命令在仓库根目录执行，要求第二轮 manifest 为 `complete`。本轮实际执行、审计、失败诊断、视觉复核及结果发布现已完成；以下保留复现入口，发布脚本仍须验证终态来源及逐景绑定，且不会覆盖已存在的发布文件。命令不会再做一轮抓放。

先完成全部 48 景的只读审计，包括 19 景规划失败的零物理尝试检查：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_physical_retry_round2.py \
  --demos exports/replicator_physical_retry_round2_20261028 \
  --output reports/replicator_physical_retry_round2_20261028/audit_execution.json \
  --require-complete
```

仅为本轮 `executed=true` 的实际失败生成诊断图。先做不启动 Kit 的来源与状态预检，再运行渲染：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/render_physical_retry_failures.py \
  --manifest exports/replicator_physical_retry_round2_20261028/manifest.json \
  --output reports/replicator_physical_retry_round2_diagnostics_20261028 \
  --dry-run

bash scripts/arena_python.sh scripts/extensions/replicator/render_physical_retry_failures.py \
  --manifest exports/replicator_physical_retry_round2_20261028/manifest.json \
  --output reports/replicator_physical_retry_round2_diagnostics_20261028
```

需要单景时增加 `--scene <scene_id>`；恢复同一最终 manifest 的未完成渲染使用 `--resume`。`--frame-hints <JSON>` 可加入绑定实际状态 SHA 的事件帧。若本轮没有实际执行失败，跳过渲染。诊断图来自保存的实测状态，时间轴停止且没有新物理步；规划失败不参与此渲染。

实际查看所有本轮失败图片后，将逐景可见事实、保守推断和下一步建议保存为 `reports/replicator_physical_retry_round2_20261028/failure_visual_review.json`。每条复核需绑定该景本轮 `execution_result_sha256` 和 `diagnostic_report_sha256`，不能复用上一轮图片冒充本轮观察。无实际失败时可没有该文件，或保留合法的空 `entries`；19 景规划失败另由CPU诊断完整覆盖，不宣称根因确认或场景无解。

最后先核对发布条件，再生成独立的第二轮结果：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round2.py --dry-run
python3 scripts/extensions/replicator/publish_physical_retry_round2.py
```

发布脚本默认读取上述 manifest、审计、CPU诊断和视觉复核；`--dry-run` 也要求最终来源一致，但不写文件。正式输出为 `configs/extensions/replicator/physical_retry_results_round2_20261028.json` 与 `docs/extensions/replicator/PHYSICAL_RETRY_ROUND2_20261028.md`，包含 154 景最新状态、14 × 11 矩阵、48 景调整结果及 19 景规划诊断。旧 `retry`、旧文档及视频保留，新记录写入 `retry_round2`；已存在的发布文件不会被自动覆盖。任务成功仍不代表安全避障或长期稳定验收。

## 第三轮剩余 29 景：失败项优化

**第三轮已结束：27 景各实际测试一次，其中 23 成功、4 任务失败、0 执行错误；另 2 景规划失败、零次新物理执行。累计 148/154 景成功，剩余 6 景。** 全部 29 景独立审计已通过，4 景实际失败已完成绑定实测状态的逐景图像复核，2 景规划失败已完成只读 CPU 诊断；正式结果已发布。详见[第三轮完整结果与失败建议](PHYSICAL_RETRY_ROUND3_20261029.md)及[机器可读状态](../../../configs/extensions/replicator/physical_retry_results_round3_20261029.json)。

在[物理执行面板](http://127.0.0.1:8765/#physicalWorkspace)的「重试轮次」选择 **第三轮 · 失败项优化（29 景）**。每景仅冻结一个抓取或路径候选，通过一次规划后才进行一次无画面物理执行；场景布局、源物理和原任务判据不变。2 个 `planning_failed` 条目均为 `attempts=0`、`executed=false`，没有本轮物理结果，保留最近一次真正执行的历史结果；不能把它们当成新物理失败，也不表示场景不可行。网页上方仍是历史执行或相机补录视频，本轮没有新增执行视频。

| 内容 | 保存位置 |
| --- | --- |
| 冻结的 29 景调整计划 | `configs/extensions/replicator/physical_retry_round3_20261029.json` |
| 新规划进度及候选结果 | `reports/replicator_physical_retry_round3_planning_20261029/progress.json` |
| 实际执行清单 | `exports/replicator_physical_retry_round3_20261029/manifest.json` |
| 第三轮人工物理标注 | `reports/replicator_dashboard/physical_retry_round3_annotations.json` |
| 实际失败的状态重建图 | `reports/replicator_physical_retry_round3_diagnostics_20261029/scenes/<scene_id>/` |
| 2 景规划失败的只读 CPU 诊断 | `reports/replicator_physical_retry_round3_20261029/planning_failure_diagnostics.json` |

「实际失败原因 / 后续调整方案」在本轮独立保存，接口为 `GET/POST /api/physical-retry-round3-annotations`，绑定本轮实际结果及配置 SHA，不覆盖前两轮标注。只有本轮实际结果已落盘且绑定有效时才可保存物理标注；规划失败景查看其 CPU 诊断及修改建议。自动保存、草稿及并发保护与前两轮相同，不触发新规划或物理执行。

### 第三轮审计、诊断渲染与发布入口

以下为本轮复现入口。物理执行、审计、失败诊断、图像复核及结果发布均已完成，不再次抓放；已有证据应直接查看。首次恢复这些证据时须保留冻结计划、实际结果、原生状态及来源 SHA，旧输入包不能代替新轮输入。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/audit_physical_retry_round3.py \
  --demos exports/replicator_physical_retry_round3_20261029 \
  --output reports/replicator_physical_retry_round3_20261029/audit_execution.json \
  --require-complete

bash scripts/arena_python.sh scripts/extensions/replicator/render_physical_retry_failures.py \
  --manifest exports/replicator_physical_retry_round3_20261029/manifest.json \
  --output reports/replicator_physical_retry_round3_diagnostics_20261029 \
  --frame-hints reports/replicator_physical_retry_round3_20261029/event_frame_hints.json \
  --detail-view --exposure-ev -3 --dry-run
```

渲染前先通过上述 CPU 预检；需要首次生成画面时去掉 `--dry-run`，恢复同一最终来源的未完成图像时加 `--resume`。`--scene <scene_id>` 可筛选场景。只读取本轮实际失败的保存状态，停止时间轴，不运行新物理步。主图及补图的相机、曝光和实测帧索引均记录来源；补图报告须写入该景视觉复核的 `supplementary_diagnostic_reports` 并通过 SHA 校验才显示。画面遮挡和无法确定的接触原因要明确保留，不能从事件名称直接推断根因。

逐景实际查看后，将可见事实、保守解释及下一轮建议汇总到 `reports/replicator_physical_retry_round3_20261029/failure_visual_review.json`，绑定每景本轮结果、诊断报告及实际查看的图片路径。2 景规划失败由独立 CPU 诊断覆盖，不伪造本轮物理图像；已生成图像数量与实际查看路径数量分别统计。

当最终审计通过、4 景实际失败都有绑定的视觉复核、2 景规划失败都有绑定的 CPU 诊断后，运行：

```bash
python3 scripts/extensions/replicator/publish_physical_retry_round3.py --dry-run
python3 scripts/extensions/replicator/publish_physical_retry_round3.py
```

正式输出为 `configs/extensions/replicator/physical_retry_results_round3_20261029.json` 和 `docs/extensions/replicator/PHYSICAL_RETRY_ROUND3_20261029.md`，包含 154 景最新状态、14 × 11 矩阵和本轮 29 景的调整与结果；保留全部 `retry`、`retry_round2` 历史，新层为 `retry_round3`。发布脚本不覆盖已有输出，不修改旧视频包。通过发布门禁仍不等于安全避障或长期稳定验收。

## 快速标注调整方案

1. 打开看图标注链接；若已打开旧版，刷新一次。「场景调整 · 看图标注」可按任务、布局、人体／财产／障碍物、标注状态筛选，也可以搜索场景名称。桌面点击「专注标注」可让图片、视角切换、方案和保存状态同屏显示，`Esc` 退出。
2. 在左侧选择场景，中间查看大图，点击缩略图切换总览、侧视或俯视；点击大图可放大，并切换原始尺寸。
3. 右侧选择「需要调整」或「无需调整」，填写调整方案，例如“手掌向前移 5 cm，降低 3 cm；保留芥末瓶上方的抓取空间”。输入后约 0.6 秒自动保存，显示「已自动保存」即可继续。
4. 用「上一个／下一个」或「下一个未标注」连续标注；非输入状态下，左右方向键切场景、上下方向键切视角。编辑时 `Ctrl+Enter`（macOS 为 `⌘+Enter`）保存并切到下一场景。

页面会恢复上次选择的场景与筛选条件。每个场景的方案独立保存，切场景时仍会完成原场景的保存；每 3 秒的进度同步不会重建输入框或打断光标。

正式标注存放在 `reports/replicator_dashboard/scene_annotations.json`，包含场景 ID、状态、方案、保存时间、版本号及所依据配置／图片的 SHA256。服务重启后仍会读取此文件；「导出标注」可下载 JSON，也会附带本浏览器尚未同步的草稿。此文件是用户方案，独立于自动复查报告与安全验收，修改标注不会改变仿真配置或提高验收计数。

输入会先保存在当前标签页的独立草稿，再自动同步到本机文件；刷新该标签页会恢复未同步输入，另一个标签页的保存不会清掉它。浏览器仍保留本地草稿备份，并兼容升级前的草稿。断网或保存失败时页面显示明确状态，恢复连接后重试；也可以点击重试。若两个页面修改同一场景，或场景图片／配置已经变化，页面保留草稿并提示选择服务器内容或保存自己的草稿，不会静默覆盖。关闭标签页、更换浏览器或设备前，应确认「已自动保存」；未同步草稿仅在当前浏览器中保留。

标注接口：`GET /api/annotations` 读取，`POST /api/annotations` 保存。写入使用文件锁和原子替换，并校验当前场景、配置、视图及版本；标注文件损坏时拒绝覆盖。服务仍只监听本机，不发布到云端。

## 进度与场景证据

第一版 `final01` 已完成 **154 个场景配置与预览覆盖**：41 项调整、113 项保留；61 张配置未变且审核绑定完整的原图直接复用，93 组新多视角均已人工粗筛（85 项看图候选、6 项待确认、2 项建议调整）。新多视角中 87 项满足像素轮廓条件、6 项渲染视图不足；人工“待确认”与渲染“视图不足”分别统计，前者也可能来自原任务冲突或障碍与任务关系不清。第一阶段完成不代表场景全部可执行，更不代表安全基准验收。

第二版 `final02` 已完成本轮 8 项视觉调整：2 项半岛厨房障碍摆位、2 项相机补视角、4 项涉及 L-island mustard 原布局的问题。原布局修复同步到该任务全部 11 变体，因此刷新 15 个场景的视图，涉及 13 个配置变更。该版本合计 44 项调整、110 项保留，93 组多视角均已人工复看为视觉候选，61 组沿用已绑定的原图，没有未复看项。21 个此前核实风险的配置保持原字节。L-island 的 11 项源布局修复通过独立证书核验，仍需匹配新基线；U mustard 与 L-shape mustard 的 22 项原布局阻塞继续保留。以上均不代表原生物理、轨迹或安全执行验收。

第三版 `final03` 已发布：本轮修复 U mustard 与 L-shape mustard 的两份原始布局，并同步到 22 个变体；154 项配置中 62 项调整、92 项保留，99 组新多视角加 55 组原图复用覆盖全部场景。当前三份独立源布局证书共绑定 33 项变体，33 项均需匹配新基线，此前已知的旧源布局阻塞为 0。新 22 项的人工复看单独计数，出图完成不会自动判为看图候选。页面优先读取 `reports/replicator_safety_upgrade/adjust05/status.json`，文件尚未发布时回退至 `adjust02/status.json`。阶段文字与分组进度不增加通过计数。每个新配置需绑定本任务的独立修复证书、重渲染图片及人工复看；L-island 已有 11 项证书继续保留。三份任务证书必须持续与当前配置匹配，才能保留源布局修复计数及解除对应旧阻塞。缺失、串用其他任务证书或哈希不匹配均显示源修复待核，不算任务完成。

网页优先显示已发布的 `final03/progress.json`，否则依次回退第二版和第一版；各旧版本继续保留。新版本未发布前仍显示前一版已核实的计数，不提前撤下旧布局阻塞。最终数字以页面的绑定证据实时统计为准。第二版核验报告为 `reports/replicator_dashboard/final02_verification01.json`（154 配置哈希、769 个链接等）和 `final02_verification02.json`（最后一项复看绑定与最终计数）。第三版配置、证书与 22 个新场景的图片/链接核验记录在 `reports/replicator_dashboard/final03_verification01.json`。

在仓库根目录启动（只需要 Python 3，无前端依赖）：

```bash
python3 scripts/extensions/replicator/progress_dashboard.py --port 8765
```

前台运行时用 `Ctrl+C` 停止。当前已启动的后台实例 PID 记录在 `reports/replicator_dashboard/server.pid`，日志在 `reports/replicator_dashboard/server.log`；重启前先核对 PID 对应本服务，再停止它，避免重复占用端口。

页面提供 14 个原始任务及基线、人体、财产、障碍物的状态，支持按布局、状态和关键词筛选。点击任务可看最近尝试的失败原因、原始视频、对照视频和报告。活动进程显示真实 PID、运行时长、日志中已观测阶段；进程退出后会自动更新。视频支持浏览器分段读取和拖动播放。

页面顶部显示独立的 **物理执行与实际视频** 面板，其后保留轨迹、图片标注及历史场景检查。**新摆位与多视角复看** 依次优先读取 `visual_placement_final03/progress.json`、`visual_placement_final02/progress.json`、`visual_placement_final01/progress.json`、`visual_placement/progress.json`，并汇总版本目录及 `visual_placement/` 内的 `render*progress.json`。新配置、已调整、保留当前配置、摆位待修、新图生成和像素轮廓条件独立计数；每格展示新总览、操作区侧视和俯视图。实际配置、语义几何证据及 PNG 哈希变化会撤回对应计数。新图是 render-only 的源机器人姿态，像素条件通过不代表人工复看、物理稳定或安全执行通过。

`preview_reuse.json` 可列出沿用原图的条目：网页核对当前及原始配置的 SHA256 和 JSON 内容相等，同时核对原图、原视觉审核及原渲染报告的 SHA256，才显示“沿用原图，摆位未变”。该计数独立于新图和多视角像素条件；不会把旧图的看图结果复制成新图结论。原图与新图可以同时追溯。

后续保留 **154 张原场景图像初筛**，分别统计当前截图已看、建议调整、可保留待验证、视图不足和历史需重看。点击格子查看截图、画面观察、调整建议、优先级及置信度，也可打开该任务的 11 图总览。视觉结论不提升任何几何、风险、物理、数采计数，“可保留待验证”不等于安全合格。

图像初筛读取 `reports/replicator_safety_upgrade/visual_triage/index.json` 及同目录的 `review_*.json`。审核文件顶层 `entries` 按 `scene_id` 合并；条目包含 `visual_status: adjust|candidate|uncertain`、`priority: 1|2|3`、`issue_codes`、`observation`、`recommendation`、`confidence`、`preview_path` 和 `preview_sha256`。网页核对当前场景预览路径和完整文件 SHA256，换图或图片内容变化时保留旧观察，但标记“历史·需重看”并从当前已看计数中移除。视觉观察仅绑定所展示截图，不自动转移到新安全候选摆位；静态源机器人姿态也不能代表实际执行轨迹。

下一块 **154 个场景安全升级** 独立累计几何检查、单基线风险、原生稳定性、安全执行和视频保存五项进度，并显示当前筛选 / 执行条目及待修复原因。点击任意格子可查看该场景的各项检查和本地证据。未提供或未通过的检查显示“待验证”；单条基线确认的风险不表示多种子稳健性。这个矩阵与已有场景搭建、原任务完整数采验收相互独立。

安全升级读取 `reports/replicator_safety_upgrade/progress.json`，采用 `schema_version: 1`、`stage`、`updated_at`、`entries`。条目以 `task_id` 和 11 种已有 `variant_id` 唯一匹配，支持 `pending`、`waiting_baseline`、`screening`、`geometry_passed`、`risk_verified`、`executing`、`accepted`、`needs_repair` 八种状态。通过标记必须明确为 JSON `true`；几何、风险、原生稳定性、安全执行均需本升级目录下真实存在的非空 `evidence_path`。风险还要求几何通过及 `baseline_status` 为 `verified` 或 `accepted`；安全执行还要求风险和原生稳定性通过且无执行错误。视频要求 `video_saved: true` 及本升级目录下非空的 `.mp4` `video_path`。只有五项齐全的 `accepted` 才显示本轮验收通过，不增加原任务的多种子完整交付计数。

安全条目可链接本升级报告目录下的报告、视频、日志，以及 `configs/extensions/replicator/scene_variants/`、`configs/extensions/replicator/safety_scene_variants/` 或 `configs/extensions/replicator/safety_upgrade/` 下的配置；允许复用已有场景搭建目录的预览图，但已有场景加载报告不能充当安全升级证据。网页验证文件存在和计数前提，不重新解码视频、运行碰撞检查或物理仿真。首次上线这一功能后需刷新浏览器一次，之后每 3 秒自动同步。

页面另保留 **14 × 11 = 154 个原场景搭建矩阵**：人体 1 种、财产 6 种、障碍物 4 种。每格依次区分待生成、配置已生成、静态已检查、仿真加载中、仿真已加载和失败，点击可打开该场景的配置、检查报告和实际仿真截图。各状态计数互斥；例如已加载的场景不再重复计入“静态已检查”。场景搭建不增加下方原始任务或数采成功计数。

场景状态由构建器原子写入 `reports/replicator_scene_build/progress.json`，格式为 `schema_version: 1`、`stage`、`updated_at`、`total`、`entries`；每个条目包含 `task_id`、`variant_id`、`status`、`checks` 及可选的 `config_path`、`scene_path`、`preview_path`、`validation_path`、`evidence_path`、`log_path`。11 个变体 ID 与已有资产一一对应；重复或未知条目不会增加计数。缺少进度文件时可读取 `configs/extensions/replicator/scene_variants/manifest.json`，否则显示待开始。

“仿真已加载”要求构建器写入 `status: ready`，加载、非空渲染、源资产未变、机器人关节结构、任务物体刚体及障碍物碰撞六项检查全部为真，且 `evidence_path` 指向实际存在的本地报告。物理初始化、稳定性、避障及操作成功等本阶段未验证声明不会误算为加载失败。缺证据会降回准备状态；网页只展示构建器保存的加载证据，不自行运行仿真，也不把加载成功解释为操作或避障成功。原始证据路径仅允许仓库内的指定目录。

诊断结果会单独标为“诊断记录”，并显示在最近动态中；完成诊断观察不计作任务或安全变体成功。已结束采集的日志按完整输出路径关联，存在歧义时不显示链接。最新成功的采集会显示成功说明，历史失败仍保留在各次记录中。

活动进程也识别本仓库 `reports/replicator_preparation/` 下的实际 Python 诊断脚本，通过进程实际挂载文件与仓库文件核对，排除调度器、其他仓库副本和仅作为参数提及的脚本。同一 worker 不因外层 supervisor 重复计数。

状态含义：

- **待开始 / 已准备**：无物理成功证据，已有布局配置不等于采集通过。
- **运行中**：检测到真实存活的本组 Python 进程，不保证仿真有进展或必将成功。
- **未通过**：最近尝试失败，保留失败报告，不代表任务不可完成。
- **部分完成**：物理采集成功或有独立准备证据，尚未满足完整交付验收。
- **检查通过**：该次中性观察等单项检查通过，不计作原始任务或安全变体完成。
- **已完成**：完整交付报告包含源资产、匹配基线、三类变体、物理与视频、源几何避障和三个规划 seed 的必需门禁；规格必须绑定实际保存文件。

网页检查验收清单中文件的存在和大小，并校验不大于 1 MiB 文件的 SHA256；大文件只查大小，不重新解码或复核全部媒体哈希，也不替代独立物理验收。原始成功条数与完整任务验收分别统计。断线时页面保留最后数据并明确提示已过期。

服务只绑定本机 `127.0.0.1`，不对公网发布。文件入口限本组报告、视频、配置及文档，禁止越出仓库或读取 `.git` 等非交付路径。若浏览器在另一台机器，需连接到运行本服务的机器或使用自己已有的 SSH 端口转发。

验证：

```bash
python3 -m unittest discover -s tests/extensions/replicator -p test_progress_dashboard.py -v
python3 -m unittest discover -s tests/extensions/replicator -p test_scene_annotations.py -v
node --test tests/extensions/replicator/test_annotation_draft_storage.cjs
```

入口文件：`scripts/extensions/replicator/progress_dashboard.py`；页面：`docs/extensions/replicator/dashboard/index.html`。

多视角渲染入口（GPU 由统一调度者执行）：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/visual_scene_previews.py \
  --manifest configs/extensions/replicator/visual_scene_variants/manifest.json \
  --output reports/replicator_safety_upgrade/visual_placement/previews_run01 \
  --progress reports/replicator_safety_upgrade/visual_placement/render_run01_progress.json
```

`--prepare-only` 只在 CPU 合成 USD 和选择相机，`--task` / `--variant` / 可重复的 `--exclude-variant` 可筛选场景。每次运行使用新的输出目录；`--resume` 仅复用当前配置及图片哈希仍相符、像素条件已通过的报告。默认 1280×848，同时检查像素面积和二维轮廓尺寸，并保留机器人与场景实体。不同视角可互补；人体手部的投影区域仍需人工确认。默认对明显过曝的画面补拍 -2 EV，保存原曝光图与 RTX 设置；`--exposure-retry-ev 0` 可关闭。曝光调整只影响观测渲染，不修改源灯光、材质、姿态或物理。

`--camera-overrides <JSON>` 可对指定场景增加哈希绑定的观察视角；支持相机位置、目标、焦距及逐视角的 -6 至 0 EV，禁止隐藏实体或修改灯光/几何。该文件的 `entries` 使用 `scene_id`、`config_sha256`、`camera_overrides`，可附 `inspection_prims` 检查手部或夹口端的既有 USD prim。准备报告与多视角报告保存覆盖文件哈希、相机参数和实际曝光设置。相机专用补拍场景必须使用有覆盖记录的新图，旧图仍作为历史保留。

最终合并入口（CPU，不启动仿真）：

```bash
python3 scripts/extensions/replicator/finalize_visual_scenes.py --check-only
python3 scripts/extensions/replicator/finalize_visual_scenes.py
```

默认等待已冻结的 `visual_scene_variants_v3/manifest.json` 与 `visual_tabletop_repairs/manifest.json`，输出到全新 `configs/extensions/replicator/visual_scene_variants_final01/` 和 `reports/replicator_safety_upgrade/visual_placement_final01/`。存在的最终目录不会覆盖。合并严格保留 21 个已核风险配置字节，台面搜索失败保留此前配置并标待修；仍不代表原生物理或安全执行通过。新最终 `progress.json` 完成后网页优先读取它，原版本继续保留。

最终 `render_manifest.json` 只列需要新图的场景：原图初筛为 candidate、配置 JSON 未变且原加载/图片/审核证据均绑定的场景进入 `preview_reuse.json`。已有多视角渲染如果 config SHA256 相同、报告和 PNG 绑定完整，则进入 `render_reuse.json`，允许配置路径不同；原 needs_view_review 会保留，不重复相同视角。最终渲染进度仍可保存到 `visual_placement/render_final01_progress.json`，网页按 scene_id 及配置 SHA256 汇总。

最终人工复看读取当前优先版本目录的 `review_*.json`，各条包含 `scene_id`、`config_sha256`、`visual_status: candidate|adjust|uncertain`、`observation`、`recommendation`，以及 `reviewed_views[{preview_path,preview_sha256}]` 或当前多视角报告的 `evidence_path/evidence_sha256`。网页核对当前配置和图片/报告绑定后才计人工已看；换图、配置改变或记录不完整会标历史需复看。可用 `preferred_views[{preview_path,preview_sha256}]` 指定人工选出的主图及补充图；不提供时使用已复看图片的顺序。主图必须仍属于当前多视角报告且哈希匹配，不能因像素较多就替代人工选图。像素 ready 不会自动成为人工 candidate；沿用原图的初筛也与新图复看独立计数。

L-island mustard 原始任务的瓶↔烤面包机、目标碗↔背景香蕉相交由独立报告 `l_island_mustard_original_geometry_issue.json` 记录。网页对该任务全部 11 变体增加 `original_task_geometry_blocked` 与证据链接，展示显眼阻塞说明。该覆盖层不修改冻结的配置/manifest，不抹去摆位检查或视觉候选，也不将它们解释为可执行或验收通过。

通用原布局覆盖层读取 `visual_placement_final01/original_task_geometry_issues.json`。只有任务条目明确为 `original_task_geometry_blocked`、适用于该任务全部变体且源几何证据文件的 SHA256 匹配，才对对应 11 项增加阻塞标记；`no_matching_geometry_report`、缺基线和无相交记录都不会自动视为碰撞。页面保留任务级原因及源报告链接，冻结配置和 manifest 不变。

源布局修复按场景条目绑定其任务的独立证书：当前配置、修复后的基准布局、至少两份源检查报告均须绑定完整 SHA256，且证书的 `passed`、`relations_passed`、`source_geometry_passed` 明确为真。网页还核对证书任务 ID，以及当前配置去掉新增 `obstacle` 后与修复基准布局完全相等。新版本可同时保留多个任务、不同修复轮次的证书，不用全局证书代替逐任务核对。通过后显示“新源布局修复已核”和“需匹配新基线”，旧相交证据保留为旧版本记录；源布局已变而证书缺失、变化或绑定不完整时显示“源修复待核”。旧相交结论只适用于源布局字段相同的配置，不自动套用于改变后的源布局。

第二版 L-island mustard 的 11 项修复证据完整后，原布局阻塞只剩 U mustard 与 L-shape mustard 的 22 项；第三版会依据这两任务各自的新证书决定是否撤下对应阻塞。源关系和源几何修复不验证原生物理稳定性、轨迹、操作成功或安全执行；改变过的源布局需重新匹配基线，网页不继承旧布局的数采或安全验收结论。

第三版实际采用 `configs/extensions/replicator/l_shape_mustard_adjust16/base_layout.json` 的 L 型源布局证书，以及 `u_shape_mustard_adjust05/base_layout.json` 的 U 型源布局证书；U 型尖壳单项采用 `adjust06` 的摆位证据。只读文件白名单包含本轮实际的 `adjust05/`、`adjust06/`、`adjust16/` 报告目录及最终配置目录。阶段状态仍从 `adjust05/status.json` 读取，不因中间尝试编号改变。

## 快速轨迹生成与独立标注

本轮后续修改遵循用户的固定布局要求：不改变机器人基座、原任务物体、目标容器、人体或障碍物的位置、朝向和尺度，不修改初始关节姿态，不删除或屏蔽新增物的碰撞。规划修订仅限于合法抓取区域内的抓点、夹爪姿态、预抓取方向、轨迹路点和规划模型正确性修订。初态若确有碰撞，抓点优化不能自动消除，保留失败记录。修改建议不是已经通过的求解结果。

“轨迹生成”区按当前 154 个场景展示待开始、规划中、已生成、失败、跳过。每景只尝试一条轨迹，失败直接保留原因和阶段，不在此轮反复调参。已生成只表示 cuRobo 规划数据已保存；不表示执行成功，也未验证长期物理稳定性或过程避障。

进度读取 `reports/replicator_trajectory_generation_20261026/progress.json`，每景最终结果读取 `scenes/<scene_id>/result.json`。网页核对任务、变体和当前配置 SHA256；完整轨迹文件存在且 SHA256 匹配后才计为已生成，失败前的部分轨迹另列。变更场景配置后，旧结果显示来源已过期，不继承生成计数。轨迹投影预览独立存放在同目录的 `preview.json`，同时绑定配置、结果和轨迹数据 SHA256；它是规划数据投影，不是仿真执行视频。

网页按日期和轮次自动发现 `reports/replicator_trajectory_retry<N>_<YYYYMMDD>/progress.json`，优先显示最新一轮，分析目录不参与选择。每轮保留已有完整规划，只处理上一轮失败项；每景选择一次修改或标注，修改后仅运行一次规划，失败进入下一景。进度条按本轮失败项数量计算，同时展示本轮新增、本轮失败、历史保留及累计已生成数量。单景展示上一轮失败阶段、本次规划修改及上一轮结果链接。新结果与原标注的结果 SHA256 不同，旧方案标为“历史·待复核”，不计入当前结果已标注数，也不会自动写入或覆盖。重试清单用当前 `retry_round` 标记待处理项，用 `carry_forward: true` 标记历史保留项，并绑定 `prior_result_path`、`prior_result_sha256` 与 `recipe`。

`action: annotate_only` 用于已有证据表明当前固定约束受阻的场景：必须提供 `review_note` 和 `annotation_evidence: {path, sha256}`，保持失败状态，记录一个标注动作、零次规划尝试。旧路径片段可以供检查，但页面和预览明确标为上一轮数据。缺失证据或字段会保存为工作器失败，不能计作标注成功；本轮不重复处理，下一轮补正。源网格实际相交与保守碰撞包络受阻分开说明，后者不能称为物体真实穿模。

查看失败阶段、当前场景图和规划预览后，可在独立“轨迹调整方案”中记录该如何调整抓取姿态、抬升高度、中间点或放置方向。轨迹标注自动保存到 `reports/replicator_dashboard/trajectory_annotations.json`，不会覆盖场景摆位标注。状态为 `unreviewed`、`adjust`、`keep`，仅表达人的方案，不改变规划或验收状态。

接口为 `GET /api/trajectory-annotations` 与 `POST /api/trajectory-annotations`。POST 字段为 `scene_id`、`status`、`note`、`expected_revision`、`config_sha256`、`result_sha256`（尚无结果时为 null）。服务器使用独立文件锁和原子写入，并拒绝覆盖更新过的版本、变更后的配置和轨迹结果。旧标注保留供人工比较。

查看入口：[轨迹检查与标注](http://127.0.0.1:8765/#trajectoryWorkspace)。先选“失败”筛选，打开规划投影或交互播放，查看失败阶段，填写方案并等待“已自动保存”，再切下一景。中途失败保留已经生成的路径段；没有路径段时仍可查看请求目标和当前场景图。提示是供人工参考的修改方向，不是对碰撞对象的定位结论。标注不会自动触发重新规划。

本轮生成器使用截图对应的源 USD 关节姿态，保留当前场景配置；依次尝试预抓取、接近抓取、抬升、转运、放置和撤离。每阶段一次 MotionGen 求解，默认阶段超时 6 秒；六阶段均成功才保存为完整轨迹。夹爪闭合角、携物变换与任务接触屏蔽是明确记录的规划假设，未经过物理抓取验证。cuRobo 自身的求解约束仍启用，不追加物理执行、长期稳定性或事后避障检查。

### 固定布局修订重试：启动与恢复

以下命令在仓库根目录运行。第一轮修订已完成：133 景新增 10 条，保留原有 21 条，累计 31 条。以下为历史复现入口，无需重复运行。首次创建目录时，在宿主机冻结首轮结果、场景与修订方案：

```bash
python3 scripts/extensions/replicator/quick_trajectory_retry.py --initialize-only
```

随后启动规划；中断后也只用此命令继续：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/quick_trajectory_retry.py --resume
```

只处理首轮失败的 133 景，首轮成功的 21 景保留。每景使用冻结方案尝试一次，失败直接进入下一景。`--resume` 跳过本轮已有成功、失败和跳过记录；已开始但未完成落盘的尝试记为 `INTERRUPTED_ONCE` 失败，不重新规划。若中断前已有终态结果落盘，则恢复该结果。不要删除输出目录或运行首轮生成脚本来恢复本批次。

在宿主机另一个终端生成实时预览，CPU 读取真实规划路径与原图相机，不启动 Isaac Sim：

```bash
python3 scripts/extensions/replicator/quick_trajectory_preview.py --output reports/replicator_trajectory_retry_20261026 --watch
```

全部结束后生成汇总：

```bash
python3 scripts/extensions/replicator/quick_trajectory_summary.py --output reports/replicator_trajectory_retry_20261026
```

结果、预览及汇总均保存在 `reports/replicator_trajectory_retry_20261026/`。原标注保留为历史方案，网页按新结果的 SHA256 提示复核。`fixed_layout_integrity.json` 核对布局、来源文件与每景一次尝试记录；汇总检查文件与数据完整性。这些检查不执行物理仿真，不验证长期稳定性、实际操作成功或过程避障。

### 第二轮失败分析与单次重试

第二轮清单为 `configs/extensions/replicator/trajectory_retry_v2.json`，分析证据在 `reports/replicator_trajectory_retry2_analysis_20261026/`。只处理上一轮剩余的 123 个失败场景，保留此前 31 条完整规划。网页优先显示 `reports/replicator_trajectory_retry2_20261026/`，并链接上一轮结果。

本轮保持全部初始布局和关节不变，记录两个经过源几何诊断的修正：香蕉在 L 带中岛、L 和 U 厨房横跨原台面两个相邻共面构件，抬升阶段的源支撑接触映射覆盖这两个构件；碗的原计算下放目标使携物包络与水槽排水件相交，释放目标 Z 增加约 12.72 毫米，XY、朝向及目标水槽不变，碗顶仍低于盆沿。后者是规划目标高度修正，不是移动初始物体或水槽；结果明确记录 `final_payload_pose_preserved: false` 和具体高度差，不声称与旧目标位姿完全相同。碰撞几何和新增危险物碰撞均保留。其余调整限定抓取姿态和轨迹路点。

第二轮已完成：123 景各尝试一次，新增 75 条、失败 48 景；保留此前 31 条，累计 106/154 条完整规划。规划耗时约 28 分 17 秒。参数与文件一致性复核通过，未执行物理或路径避障验收。以下命令仅用于恢复未开始的场景或重新生成预览/汇总，不删除输出或重复初始化：

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/quick_trajectory_retry.py --source reports/replicator_trajectory_retry_20261026 --output reports/replicator_trajectory_retry2_20261026 --recipes configs/extensions/replicator/trajectory_retry_v2.json --resume
python3 scripts/extensions/replicator/quick_trajectory_preview.py --output reports/replicator_trajectory_retry2_20261026 --watch
python3 scripts/extensions/replicator/quick_trajectory_summary.py --output reports/replicator_trajectory_retry2_20261026
```

每景仍只有一次完整序列尝试、每阶段最多一次求解；遇到失败即进入下一景。CPU 目标可达性预筛不代表碰撞约束下可达，也不代表物理操作成功。


### 第三轮：单次修改或标注

第三轮已完成：48景中，30景各规划一次，新增15条、15景仍失败；18景仅更新诊断，不调用求解器。其中11项标注写入成功，7项缺少证据绑定字段而被拒绝，本轮保留工作器失败、没有重复动作。累计121/154条完整规划，剩余33景。规划过程约7分30秒。

结果位于 `reports/replicator_trajectory_retry3_20261026/`，清单为 `configs/extensions/replicator/trajectory_retry_v3.json`。369个冻结文件、初始关节与上轮最终物体目标核对通过，106条历史成功和133条网页标注保留。`retry_outcome_review.json` 的整体结论为不通过，明确列出7项标注交付错误；不能用文件完整性通过替代它。缺少证据的初始化检查已经修复，下一轮补正标注，不改本轮冻结记录。详情见 [第三轮报告](../../../reports/replicator_trajectory_retry3_20261026/ROUND_REVIEW.md)。


### 第四轮：补正标注并继续单次规划

第四轮已完成33个动作：15景各规划一次，新增7条、8景仍失败；18景仅更新诊断，全部保存成功。累计128/154条完整规划，剩余26景。规划过程约4分28秒。上一轮7项标注字段错误已补齐，历史失败阶段及路径片段沿SHA绑定的结果链恢复，未重跑这些场景或覆盖旧记录。

结果位于 `reports/replicator_trajectory_retry4_20261026/`；清单为 `configs/extensions/replicator/trajectory_retry_v4.json`。365个冻结文件、初态与最终物体目标核对通过；121条历史成功、133条网页标注保留；154场景的308个预览链接和文件绑定通过。独立结果复核通过，保留两项原始 `GRAPH_FAIL` 中间状态提示：本机cuRobo在后续轨迹优化成功时可能保留该字符串，实际成功布尔与已保存路径片段分别核验，不按字符串单独判定。未来结果额外记录 `solver_success` 原始布尔。

第五轮处理8个新的规划失败和18个固定约束诊断项，仍为每景一次修改或标注。以上均为规划与文件证据，不代表物理操作或过程避障验收。详情见 [第四轮报告](../../../reports/replicator_trajectory_retry4_20261026/ROUND_REVIEW.md)。


### 第五轮：8次单次规划与18项诊断

第五轮已完成：8景各规划一次，新增4条、4景失败；18景仅更新诊断，全部保存成功。累计132/154条完整规划，剩余22景。耗时约2分45秒，场景布局、初态和最终物体目标保持；353个冻结文件、128条历史成功及133条人工标注核对通过。结果完整性、独立复核和308个网页预览链接检查通过。

清单：`configs/extensions/replicator/trajectory_retry_v5.json`；结果：`reports/replicator_trajectory_retry5_20261026/`。第六轮可调整项为4个人体碗任务：2个transfer、2个retreat；其余18项仍为固定约束诊断。每景每轮仍只修改或标注一次，失败不在同轮反复尝试。详情见 [第五轮报告](../../../reports/replicator_trajectory_retry5_20261026/ROUND_REVIEW.md)。这些是规划数据结论，不代表物理操作成功。

### 第六轮：转运推进与撤离调整

第六轮已完成22个动作：4景各规划一次，新增2条、2景失败；18景仅复核诊断，全部保存成功。累计134/154条完整规划，剩余20景，耗时约1分32秒。L型带中岛与U型人体碗任务已生成完整轨迹；L型和半岛式人体碗任务通过转运与放置，均停在撤离IK_FAIL，进入下一轮单次调整。

351个冻结文件、132条历史成功及133条人工标注保持；数据完整性、独立结果和308个预览链接复核通过。清单为`configs/extensions/replicator/trajectory_retry_v6.json`，结果为`reports/replicator_trajectory_retry6_20261026/`。详情见[第六轮报告](../../../reports/replicator_trajectory_retry6_20261026/ROUND_REVIEW.md)。以上仍为规划结论，未进行物理操作或过程避障验收。

### 第七轮：两个撤离任务生成完成

第七轮已完成20个动作：L型与半岛式人体碗任务各规划一次，均成功；18景仅复核固定约束，标注全部保存。累计136/154条完整规划，剩余18景，耗时约50秒。349个冻结文件、134条历史成功及133条人工标注保持；完整性、独立结果和308个预览链接复核通过。

剩余18景为2项源碰撞网格相交、11项碗的携物球模型阻挡、5项机器人初态球模型阻挡。在布局、初态、最终物体目标和碰撞模型均不变的条件下，无法靠换抓点或后续路点消除，继续重复标注不会产生新轨迹。逐景证据与边界见[第七轮报告](../../../reports/replicator_trajectory_retry7_20261026/ROUND_REVIEW.md)。清单为`configs/extensions/replicator/trajectory_retry_v7.json`，结果为`reports/replicator_trajectory_retry7_20261026/`。规划生成不等同于物理任务成功。


### 第八轮：授权的一次场景设置优化

2026-10-28按新授权处理剩余18景：每景只优化一次新增物体摆位，再规划一次。新增7条完整轨迹，11景失败，累计143/154；规划耗时约4分17秒。原136条成功、133条人工备注保留，没有二次改位或二次规划。

18张真实新摆位截图已发布，轨迹预览绑定新背景；网页分列本轮修改、失败阶段及场景质量提示。G型香蕉人体仍有人体衣服与橱柜网格相交，另3景有底座越出台面AABB边界提示，其中2景虽生成轨迹仍带质量问题，不能算场景验收通过。

清单为`configs/extensions/replicator/trajectory_retry_v8.json`，新配置在`configs/extensions/replicator/scene_retry_once_20261028/`，结果在`reports/replicator_trajectory_retry8_20261028/`。文件、一次性约束及网页复核通过，未做物理执行或过程避障验收。详见[第八轮逐景报告](../../../reports/replicator_trajectory_retry8_20261028/ROUND_REVIEW.md)。本轮已结束，剩余失败不会自动进入下一轮。

### 第九轮：5 景摆位调整与 6 景抓姿或路点调整

按“继续，再优化一轮”处理剩余 11 景，每景一份方案、一次完整规划。新增 9 条，累计 **152/154**，规划耗时约 3 分 31 秒。4 个碗任务横杆仅改朝向，U 型芥末瓶横杆仅水平平移；4 个财产物碗任务改对角碗沿抓姿，另 2 个芥末瓶任务分别改预抓取和转运路点。物体初态与 R8 最终目标保持，没有二次调参或规划。

剩余两景：U 型芥末瓶酒杯场景由转运失败推进到撤离 `IK_FAIL`；U 型芥末瓶横杆场景由预抓取失败推进到接近 `IK_FAIL`。均保留实际部分轨迹，本轮结束。

5 张真实新布局截图与 11 份轨迹预览已发布；354 个冻结文件、143 条历史成功的 1001 个文件及 133 条人工标注核对通过，308 个预览链接有效，网页 stale=0。G 型香蕉人体、U 型碗窄口两项历史质量警告继续显示；规划生成不等同物理任务或安全基准验收。

清单：`configs/extensions/replicator/trajectory_retry_v9.json`；结果：`reports/replicator_trajectory_retry9_20261028/`。详见[第九轮逐景报告](../../../reports/replicator_trajectory_retry9_20261028/ROUND_REVIEW.md)。


### 第十轮：154 景规划全部生成

本轮两景均只调整轨迹目标，布局不变：U 型芥末瓶酒杯场景降低撤离路点约 6.3 cm；横杆场景保留原抓点、倾腕 27° 并保持携瓶世界位姿。各一次完整规划均生成成功，累计 **154/154，0 景失败**，正式规划耗时约 50 秒。

152 条旧成功记录的 1064 个文件及 133 条人工标注保持，308 个预览链接核对通过；网页已更新最后两份轨迹预览，stale=0。原人体穿模及窄口底座支撑边界两项历史质量提示仍显示；完整规划不能替代物理操作、稳定性或安全避障验收。

清单：`configs/extensions/replicator/trajectory_retry_v10.json`；结果：`reports/replicator_trajectory_retry10_20261028/`。详见[第十轮逐景报告](../../../reports/replicator_trajectory_retry10_20261028/ROUND_REVIEW.md)。
