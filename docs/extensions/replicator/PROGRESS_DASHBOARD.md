# A 组本地实时进度页

地址：[物理执行与实际视频](http://127.0.0.1:8765/#physicalWorkspace)、[场景看图标注](http://127.0.0.1:8765/#annotationWorkspace)。已接入 14 个任务 × 11 个变体，共 154 个场景。进度每 3 秒自动刷新，只读实际报告和本机进程，不启动或停止采集，不占用 CUDA。

## 物理执行与实际视频

截至 2026-10-28，154 景规划已完整生成，真实物理执行批次已启动但尚未全部完成。顶部面板读取 `exports/replicator_154_physical_demos_20261028/manifest.json`，独立显示 planned、physical pending/running/success/failed/error 和有效视频数量。每景只执行一次；实际抓放失败保留原结果和视频，不计作规划失败，也不自动重跑。

按场景选择可查看真实执行结果、失败原因和 overview 视频。初始化或录制错误单独显示，`executed=false` 表示没有开始执行，不标成抓放失败或成功。结果必须与 R10 的场景、配置、轨迹及 SHA 绑定；视频链接还需实际执行结果 SHA 和视频 SHA 验证通过。缺失或不一致显示待核，不用旧规划动画替代。

G 型香蕉人体的源网格相交、U 型碗窄口的底座支撑边界两项历史质量提示继续保留。实际任务成功也不自动解除这些提示或代表安全避障验收。人工标注和原轨迹界面保持独立。

首次执行、恢复和 `--dry-run` 的完整命令见 [154 景发布与输入恢复说明](RELEASE_154_PLANNED_SCENES.md)。最小输入包包含执行所需的 R10 progress、154 份 result/source_scene 和各实际轨迹路径；大型源资产仍须另行恢复。网页历史图片与标注不在最小输入包内。只有 154 个真实视频齐全且通过校验后才能生成完整物理 demo 包，当前进度以面板和 manifest 为准。

初始化启动次数与物理 episode 分开记录。L-island 香蕉人体景曾在首次 `env.step` 前停滞；旧 `initializing`、`executed=false` 记录保留，新初始化通过 `initialization_history` 绑定旧记录，实际结果写入独立子目录。这只恢复尚未开始的唯一 episode，不重跑已经执行的任务；额外初始化记录随视频包保留，不能替代该景的正式结果或真实视频。

已有批次在没有 running 项且上一进程退出后，推荐主机运行 `python3 scripts/extensions/replicator/run_physical_demo_batches.py`，按原始任务启动新 Kit 进程并使用 8 个 PhysX 线程。该协调器的 `--dry-run` 只核对待执行分组，不代替执行器的源文件、轨迹及资产 SHA 检查；调度器不会重试已经执行的失败场景或自动处理初始化恢复。

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
