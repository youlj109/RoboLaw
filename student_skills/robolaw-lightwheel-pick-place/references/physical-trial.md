# 青柠入木碗：实际执行参考

本参考对应 `kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian`，源 YAML SHA256 为 `c49ae7320caaa136df9aa6e25194e862b555379ae4fc3f5b4e1f559093005598`。这是 Lightwheel L 型 Scandinavian 厨房，不是原 one-wall 芥末瓶任务。读取本组实现时使用本文件；其他4个任务仍需分别实现与物理验证。

## 当前证据边界

2026-10-21，青柠任务的三类固定布局代表变体均完成1条真实成功采集，原任务终态predicate、双指抓取/提升、逐物理步外部碰撞、固定基座/台架、独立RGB/轨迹验收均通过。首个物理成功基线为 `base_attempt12`；它没有保存后来新增的当前终态 `object_on_destination`/支持力矩阵，严格新版验收的终态支持证据项失败，保留原记录不回填。最终三变体均包含该证据。

| 变体 | 最终目录 | 动作/每路RGB帧 | 120Hz物理检查 | 真实提升 | 最近新增障碍表面距 |
| --- | --- | ---: | ---: | ---: | ---: |
| 人体 | `human_attempt03` | 473 | 3784 | 160.23mm | 458.26mm |
| 财产花瓶 | `property_attempt02` | 473 | 3784 | 162.71mm | 232.31mm |
| 横杆障碍 | `robot_attempt02` | 418 | 3344 | 162.71mm | 33.76mm |

每条close与lift最后5个动作都有同步双指接触证据。固定base/stand独立最小距分别为45.64、383.39、412.66mm；花瓶全程最大位移0。三路原视频与 `all_cameras.mp4` 均完整解码，人体另有同步总览。`validation.json`、`full_world_audit.json`、`fixed_base_hazard_audit.json` 和自碰撞/限位的 `self_collision_audit.json` 是交付验收证据；human03额外保存独立 `source_support_audit.json`。完整逐物理自碰撞审计使用实测臂关节及finger_joint同步的mimic夹爪配置，其他6夹爪关节的真实外部几何由FCL检查。

这些位置的源基线几何回放仍可行，不宣称障碍迫使绕行。人体靠近固定台架安全区域，横杆限制操作空间，财产花瓶位于同一源工作台；新增物体初始不穿厨房、原物件或台架。这里只证明此固定布局各1条，不推断随机化、其余8种障碍或200条采集成功率。本组其余4个原始任务尚未物理验证。

数据目录为 `datasets/skill_trials/lightwheel_pick_place/`，报告目录为 `reports/skill_trials/lightwheel_pick_place/`。保留各 `base_attemptNN` 的失败结果和日志，不能把已有视频等同于成功数据。

## 已存在的入口

从包含本组代码的仓库运行，先检查 `scripts/arena_python.sh`、`ARENA_CONTAINER`、`/workspace` 挂载和实际解释器。以下命令对应本机容器适配；其他机器应调整容器/挂载。技能 ZIP 不携带执行器、第三方源码或大资产。

```bash
python3 student_skills/robolaw-lightwheel-pick-place/scripts/preflight.py --repo .
bash scripts/arena_python.sh student_skills/robolaw-lightwheel-pick-place/scripts/runtime_inventory.py
mkdir -p reports/skill_trials/lightwheel_pick_place
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/restore_assets.py
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/restore_lightwheel.py
bash scripts/extensions/lightwheel_pick_place/run.sh collect.py --device cpu --variant base --seed 42 --placement-seed 42 --output /workspace/datasets/skill_trials/lightwheel_pick_place/base_new
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/validate.py /workspace/datasets/skill_trials/lightwheel_pick_place/base_new
bash scripts/extensions/lightwheel_pick_place/run.sh audit_self_collision.py /workspace/datasets/skill_trials/lightwheel_pick_place/base_new
```

`run.sh` 设置 `HEADLESS=1`、相机模式与共享 GPU 锁，默认600秒单次限时；输出目录必须新建，不覆盖旧尝试。运行日志另存本组 reports。GPU锁为 `/workspace/reports/skill_trials/gpu.lock`。本入口只支持1环境、1条尝试，`--num-envs` 大于1会拒绝；没有 `--resume` 或成功配额实现。

三类现有标识为 `human`、`property`、`robot`，分别使用 MPFB 人体、vase、projecting_crossbar。确认基线通过后，依次将上例 `--variant base` 替换为相应标识，并给新输出目录，再分别运行验收。变体通过 `fixed_layout.json` 固定原始已解场景关系，使加入障碍不重新采样青柠、碗和机器人；其源 YAML 关系仍保留。变体有效性另需几何/动作证据，不因实例已加载就宣称迫使绕行。

本组人体虽然位于原外相机的几何视锥，真实渲染被台架/机器人遮挡。人体展示使用额外 `--overview-camera`：保留原三路 `images.h5`、`cameras.json` 和1920×360合成视频，另存同步1280×720 `overview_camera.mp4`。额外总览用于展示人与任务环境，不替换任何原policy相机；先用 `run.sh probe_overview.py --output <新目录>` 检查真实初始画面中人、机器人与碗可见，再采human。总览原JPEG/位姿在 `overview/`，`frame_alignment.json` 记录两套相机在同一动作前拍摄、拍摄前后物理步数不变；验收器逐帧解码并核对时间/位姿。不能用视锥计算代替实际可见性检查。

每条变体补充固定基座和完整物理状态自碰撞审计，以下用新人体输出举例：

```bash
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/audit_fixed_base.py /workspace/datasets/skill_trials/lightwheel_pick_place/human_new
bash scripts/extensions/lightwheel_pick_place/run.sh audit_self_collision.py /workspace/datasets/skill_trials/lightwheel_pick_place/human_new
```

固定已解机器人姿态时，`RotateAroundSolution` 是已消费的解算修饰标记，其yaw已写入基线pose；再次连同 `IsAnchor` 应用会触发源解算器错误。局部适配只移除该已消费标记，保留源 `On`/`NextTo` 检查，不再次旋转或更改原布局。

## 资产与源任务

- `environment.py` 局部解析源 graph，保留机器人支座、基座空间关系、原相机、原 `PickAndPlaceTask` 和全部对象。目标青柠之外，木碗、西兰花、红薯、红椒均保留，厨房植物也保留为碰撞物。
- Lightwheel 源厨房为 `robocasakitchen` layout 3/style 8；`restore_lightwheel.py` 经 SDK 恢复对应厨房、`Broccoli010`、`SweetPotato005`，并写实际机器的 `asset_paths.json`。不要复制本机 `/home/user` 缓存绝对路径给另一台机器。
- `restore_assets.py` 从固定 Arena 注册资产路径递归恢复青柠、木碗和红椒 USD 及依赖，记录下载来源与 SHA256。引用纹理/材质也要检查；日志出现某些上游非 mesh 几何警告不等于本组碰撞覆盖已验证，需查实际 CollisionWorld。
- 源 ready pose 会碰到本厨房上柜。本组只改 reset 关节位姿为 `[0,-60,0,-150,0,90,0]` 度，保留源基座关系；执行过程中禁止写状态搬运物体。此姿态不是5个任务通用 home。
- 原始物体有静态缩放，例如红薯根节点 scale=1.5。更新刚体世界位姿时应对参考刚体变换剥离静态缩放，再计算刚体增量；几何已烘焙的1.5缩放必须保留。不能把所有源对象强行变成单位尺度。

## 已证实的抓取与碰撞陷阱

本次青柠源局部 collision mesh 尺寸约 `75.51 × 61.34 × 60.04 mm`。`base_attempt06` 实测几何中心约 `[3.84307,-0.32425,0.94595]` 米；世界坐标只用于复核该固定布局，后续必须从实际姿态重新计算。

1. **夹爪近台面的球近似比实物低。** 原35°倾斜、yaw0的中心抓取，最低内指球底为0.918487m，真实开指碰撞几何底为0.926389m；台面0.92m。cuRobo因此拒绝接近末点，而自碰撞/关节限制本身有效。将 TCP 提至实测中心上15mm后，实际双指可闭合夹持，不是通过忽略台面解决。`base_attempt08` 的连续5个close采样证明双指都有真实接触载荷，接触在青柠局部z约32–33mm。
2. **旋转圆果的外接方盒会误报穿模。** 08真实夹持末态，旧OBB最低z=0.895745m，真实collision网格/紧致凸包最低z=0.917929m，方盒额外向下约22.18mm。旧OBB与植物相交，但实际网格/凸包到植物约15.77mm；真实凸包垂直提升16cm的82点CPU检查最小约13.78mm。应使用源collision凸包，不能删掉植物/台面或移动原目标来修复该误报。
3. **cuRobo携物球是搜索近似。** 本组使用紧致凸包作FCL携物几何，cuRobo球代理明确记录是否覆盖全表面。未全覆盖的内部球不能单独作为安全证明；每条规划密集路径及每个120Hz物理步都检查真实凸包、实际夹爪和完整世界。代理针对源支撑面所做局部裁剪也须记录阶段、平面和限制，不能变成全程忽略台面。
4. **原始静置支撑与mesh表面并不完全重合。** 本源PhysX凸分解静置姿态中，原mesh最低点约0.916085m，对0.92m台面有约3.915mm差异；FCL碰撞距离的-1是相交哨兵，不是1m穿透。pregrasp/approach/close仅保留实际初始最低点再0.5mm内的源支持，且青柠root XY须在源锚点5cm内。早期lift的固定3mm带在human01卡住合法夹持起点：其实测残余深度3.334mm，已比初态改善0.581mm。新lift先确认夹持底部未低于原静置界，再用实际夹持最低点减0.5mm为下界；每物理步先按旧界检查，再将下界随真实抬升单调收紧，同样限制源XY。`audit_source_support.py` 直接从USD碰撞顶点和逐physics真实姿态独立重算界链，验收器自动要求通过。此许可只限该青柠与精确源台面，不允许机器人碰台面、横向拖拽或碰植物；旧property02/robot02保留采集时3mm模型，不回填成新规则，也不跨任务照抄容差。
5. **Robot子树排除会遗漏静态台架。** 通用世界读取排除 `/Robot`，而运动包络并不包含stand。人体 `[3.4,-1.05,0]` 虽通过手臂轨迹检查，却与源台架真实mesh相交。本组候选人体改到 `[3.25,-1.05,0]`，到stand实测45.64mm，完整源厨房无非支撑相交，基线481个实际状态全部通过。最终新场景仍须真实采集并检查原相机；CPU相机视锥只说明可能可见，不证明没有遮挡。使用 `check_human_stand.py` 与候选扫描/回放脚本保留补充证据，不能省略固定基座几何。

可重复运行的CPU证据（不需要Kit/GPU，但需要对应真实场景/轨迹文件）：

```bash
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/geometry_probe_spheres.py
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/geometry_probe_contacts.py
bash scripts/arena_python.sh scripts/extensions/lightwheel_pick_place/geometry_probe_carried_lime.py
```

对应JSON写到本组 reports，同名脚本中记录输入尝试和比较模型。第三份脚本显式重建旧OBB，避免实现已换凸包后失去历史可复现性。CPU几何可证实包络误差和候选可行性，不能替代物理提升/放置成功。

## 成功、接触、相机与审计

必须保留原Arena任务predicate；成功回调里的历史 `terminal_terms` 不足以证明最终物体仍在碗里。终态需保留原success和当前 `object_on_destination` 的结果，同时实际松开夹爪。接触证据来自PhysX双侧指爪接触，分别检查close、lift及携物阶段；几何贴近不代替载荷接触。`simulation_attachment` 必须为false，`planning_attachment_tcp_to_lime` 只是规划模型，不是物理约束。

原任务并不要求放下后沿世界Z直线上撤。栏杆变体的 `robot_attempt01` 在真实放入后，上撤候选被cuRobo球代理拒绝且未执行；保留该失败记录。局部执行器可以在可选retreat的预检阶段拒绝且该阶段尚未执行任何动作时，记录拒绝原因，保持张开的夹爪原位等待稳定，再检查原终态predicate。不得吞掉已执行路径中的物理失败，也不得把可选动作省略变成省略松爪、低速支撑或最终在碗内检查。

动作与RGB为15Hz，物理审计120Hz。每条记录包括所有源动态对象状态、机器人各关节/身体、三路相机姿态、逐物理步 `full_world_audit.json`、`contacts.json` 和变体的 `clearances.json`。验收器独立重算接触序列/物体局部接触坐标、提升量、所有HDF5数组有限性/对齐、每张JPEG完整解码、三路原MP4和合成预览完整解码、相机与机器人挂载一致性；物理审计应每个动作至少8条且全有效。不能用最后一个安全距离替代整条审计。

vase需全轨迹位移与倾角验证，不能只看最终回到原位。当前只验证几何/接触、位移与稳定性，不模拟玻璃破碎或人体伤害。将 `fixed_only`、原任务判定、数据完整性和增广有效性分别报告；仅固定单条不能推断200条成功率或多环境最大速度。
