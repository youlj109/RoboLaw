# RoboLaw 四人任务安排与快速开始

更新日期：2026-10-23。共同开发基线为 `origin/ylj`，本轮修复提交 `76de5bb`。四人按不同原始任务独立完成端到端开发，没有串联依赖。

**每位同学自己从 `origin/ylj` 创建以本人姓名缩写命名的分支，例如 `zxw`、`lgd`、`lwx`。** `ylj` 是共同参考基线，个人分支不加 `ylj-` 前缀。姓名与 A/B/C/D 分工的对应关系由实际分配确定，本文不作预设。

**已经安装主分支的同学，请先按下方步骤获取 `ylj`，再从它创建个人分支。** 本轮安全扩展、修复和学生技能以 [`ylj` 分支](https://github.com/youlj109/RoboLaw/tree/ylj)为参考；仅在主分支执行 `git pull` 不会把 `ylj` 的实现合入当前分支。线上入口：[共享任务安排](https://github.com/youlj109/RoboLaw/blob/ylj/docs/student_assignments/README.md)、[四组技能源码](https://github.com/youlj109/RoboLaw/tree/ylj/student_skills)、[物理与避障修复说明](https://github.com/youlj109/RoboLaw/blob/ylj/docs/extensions/AVOIDANCE_REPAIR_20261022.md)。

31个原始环境中，已完成的 Lightwheel `pick_and_place` 芥末瓶入碗及11个变体作为参考；剩余30个环境分配如下。每个原始任务先做人体、财产、障碍物三个变体，不要求立即复制11种资产。

## 分工总表

| 学生 | 范围 | 环境数 | 分支 | 直接可用的 Prompt |
|---|---|---:|---|---|
| A | Replicator 厨房抓放 | 14 | 本人姓名缩写 | [A组启动 Prompt](A_PROMPT.md) |
| B | Lightwheel 其余抓放 | 5 | 本人姓名缩写 | [B组启动 Prompt](B_PROMPT.md) |
| C | Lightwheel 开门 | 8 | 本人姓名缩写 | [C组启动 Prompt](C_PROMPT.md) |
| D | Lightwheel 按钮与旋钮 | 3 | 本人姓名缩写 | [D组启动 Prompt](D_PROMPT.md) |

任务数按实施难度分配，按钮、旋钮和开门的接触控制工作量较大，不按数量平均。

## 每组完整任务清单

环境ID与原YAML路径、源码哈希以对应技能 `references/assignment.json` 为准。以下列出完整ID，避免混淆厨房布局。

### A组：Replicator 厨房抓放

先复现修复后的 G 型芥末瓶入碗，再扩展同布局其他操作，最后迁移其他布局。

当前证据范围：首任务三类已修复重采；人体有三个 seed 的转运段规划对照，财产和横杆只有单基线必要性证据。

| 编号 | 布局 | 操作 | 完整环境 ID |
|---|---|---|---|
| A01 | G型 | 芥末瓶放入碗 | `kitchen_bench_replicator_g_shape_mustard_bowl` |
| A02 | G型 | 香蕉放入碗 | `kitchen_bench_replicator_g_shape_banana_bowl` |
| A03 | G型 | 碗放入水槽 | `kitchen_bench_replicator_g_shape_bowl_sink` |
| A04 | L型带中岛 | 香蕉放入碗 | `kitchen_bench_replicator_l_island_banana_bowl` |
| A05 | L型带中岛 | 碗放入水槽 | `kitchen_bench_replicator_l_island_bowl_sink` |
| A06 | L型带中岛 | 芥末瓶放入碗 | `kitchen_bench_replicator_l_island_mustard_bowl` |
| A07 | L型 | 香蕉放入碗 | `kitchen_bench_replicator_l_shape_banana_bowl` |
| A08 | L型 | 碗放入水槽 | `kitchen_bench_replicator_l_shape_bowl_sink` |
| A09 | L型 | 芥末瓶放入碗 | `kitchen_bench_replicator_l_shape_mustard_bowl` |
| A10 | 半岛式 | 碗放入水槽 | `kitchen_bench_replicator_peninsula_bowl_sink` |
| A11 | 半岛式 | 芥末瓶放入碗 | `kitchen_bench_replicator_peninsula_mustard_bowl` |
| A12 | U型 | 香蕉放入碗 | `kitchen_bench_replicator_u_shape_banana_bowl` |
| A13 | U型 | 碗放入水槽 | `kitchen_bench_replicator_u_shape_bowl_sink` |
| A14 | U型 | 芥末瓶放入碗 | `kitchen_bench_replicator_u_shape_mustard_bowl` |
### B组：Lightwheel 其余抓放

先复现 L 型青柠入木碗并补新版检查，再依次处理柠檬、香蕉、汤勺、百事可乐。

当前证据范围：青柠三条保留真实任务成功和历史单轨迹证据，尚未完成本轮新增完整静置与多 seed 检查。

| 编号 | 布局 | 操作 | 完整环境 ID |
|---|---|---|---|
| B01 | L型 Scandinavian | 青柠放入木碗 | `kitchen_bench_lightwheel_place_lime_in_wooden_bowl_l_shaped_scandinavian` |
| B02 | U型带中岛 Farmhouse2 | 架上柠檬放到盘子 | `kitchen_bench_lightwheel_pick_lemon_shelf_on_plate_u_shaped_with_island_farmhouse2` |
| B03 | U型带中岛 Farmhouse2 | 香蕉放入木碗 | `kitchen_bench_lightwheel_place_bananas_in_wooden_bowl_u_shaped_with_island_farmhouse2` |
| B04 | G型 Large Scandinavian | 汤勺放到盘子 | `kitchen_bench_lightwheel_place_ladle_in_plate_g_shaped_large_scandinavian` |
| B05 | U型带中岛 Farmhouse2 | 百事可乐放入篮子 | `kitchen_bench_lightwheel_place_pepsi_in_basket_u_shaped_with_island_farmhouse2` |
### C组：Lightwheel 开门

先复现基础冰箱并补新版检查，再迁移其他冰箱、橱柜、冷冻室、微波炉和烤箱。

当前证据范围：基础冰箱三条保留真实开门和历史单轨迹证据；人体/财产涉及合法抓点适配，不能混称固定终点绕行。

| 编号 | 布局 | 操作 | 完整环境 ID |
|---|---|---|---|
| C01 | 原始基础场景 | 打开冰箱 | `kitchen_bench_lightwheel_open_fridge` |
| C02 | U型带中岛 Farmhouse2 | 打开橱柜 | `kitchen_bench_lightwheel_open_cabinet_u_shaped_with_island_farmhouse2` |
| C03 | G型 Large Scandinavian | 打开冷冻室 | `kitchen_bench_lightwheel_open_freezer_g_shaped_large_scandinavian` |
| C04 | G型 Large Scandinavian | 打开冰箱 | `kitchen_bench_lightwheel_open_fridge_g_shaped_large_scandinavian` |
| C05 | U型带中岛 Farmhouse2 | 打开冰箱 | `kitchen_bench_lightwheel_open_fridge_u_shaped_with_island_farmhouse2` |
| C06 | 原始基础场景 | 打开微波炉 | `kitchen_bench_lightwheel_open_microwave` |
| C07 | G型 Large Scandinavian | 打开微波炉 | `kitchen_bench_lightwheel_open_microwave_g_shaped_large_scandinavian` |
| C08 | G型 Large Scandinavian | 打开烤箱 | `kitchen_bench_lightwheel_open_oven_g_shaped_large_scandinavian` |
### D组：Lightwheel 按钮与旋钮

先复现修复后的左取消按钮，再开发右取消按钮，最后开发烤箱温度旋钮。

当前证据范围：左按钮三类已修复重采，各有三个 seed 的接近段规划对照；右按钮与温度旋钮未验证。

| 编号 | 布局 | 操作 | 完整环境 ID |
|---|---|---|---|
| D01 | G型 Large Scandinavian | 按左侧烤面包机取消按钮 | `kitchen_bench_lightwheel_press_left_toaster_cancel_button_g_shaped_large_scandinavian` |
| D02 | G型 Large Scandinavian | 按右侧烤面包机取消按钮 | `kitchen_bench_lightwheel_press_right_toaster_cancel_button_g_shaped_large_scandinavian` |
| D03 | G型 Large Scandinavian | 旋转烤箱温度旋钮 | `kitchen_bench_lightwheel_turn_oven_temperature_knob_g_shaped_large_scandinavian` |

## 统一交付要求

每个原始任务先交付：匹配的无新增风险成功基线，以及人体、财产、障碍物各1条完整真实成功数据。控制参数不同需各自匹配基线。学生不承担200条批采。

| 项目 | 必须提交的内容 |
|---|---|
| 场景与来源 | 原任务ID/YAML/成功判据、源资产及配置哈希、合法布局关系 |
| 物理质量 | 覆盖任务时长的中性检查、必要的机构可动性检查，修复依据与边界 |
| 避障设计 | 代表性短基线、源几何风险证明、同参数多seed规划对照、失败候选 |
| 真实操作 | 人体/财产/障碍物各1条；真实接触与成功终态 |
| 安全审计 | 原间距、台架、自碰撞、携物、环境碰撞及财产稳定性，明确采样范围 |
| 数据视频 | 原轨迹/动作/图像，原三路MP4、必要总览、原速同视角对照；完整解码和目视检查 |
| 交接 | 代码、配置、复现命令、报告、视频/数据路径与哈希、失败及未测项 |

当前历史样本的证据强弱不同。A人体及D三类有三seed分段规划对照；A财产/横杆仍只有单基线必要性证据；B/C尚待补本轮新增检查。学生的新任务按新版要求逐项验证，旧成功数不能代替。

## 快速开始

### 已经安装主分支：在本人独立克隆中切换

保留现有安装，先进入自己的 RoboLaw 仓库，检查工作区和远程。下面的切换步骤要求 `git status --short` 没有输出，且 `origin` 指向 `youlj109/RoboLaw`。有未提交修改时先保存自己的工作，或使用下方“新建独立克隆”步骤；不要强制重置、清理文件或自动覆盖修改。

```bash
git status --short
git branch --show-current
git remote -v
git fetch origin refs/heads/ylj:refs/remotes/origin/ylj
```

显式获取 `ylj` 也适用于当初只克隆主分支的情况。在干净的个人工作区执行下方命令，输入**自己的姓名缩写**；例如本人姓名缩写为 `zxw` 时才输入 `zxw`，其他同学使用自己的名字。命令仅创建本地、远程均不存在的新分支，直接以 `origin/ylj` 为起点，无需先创建本地 `ylj`。

```bash
(
  read -r -p '请输入本人姓名缩写，作为个人分支名：' student_branch || exit 1
  case "$student_branch" in
    ''|main|master|ylj|ylj-*)
      echo '请使用本人姓名缩写；不能使用共同基线或原 ylj- 分组命名。'
      exit 1
      ;;
  esac
  git check-ref-format --branch "$student_branch" >/dev/null || exit 1
  remote_branch=$(git ls-remote --heads origin "refs/heads/$student_branch") || exit 1
  if git show-ref --verify --quiet "refs/heads/$student_branch" || [ -n "$remote_branch" ]; then
    echo '该个人分支已经存在，请按下文沿用其历史，不要覆盖。'
    exit 1
  fi
  git switch --no-track -c "$student_branch" origin/ylj
)
```

`--no-track` 避免把共同基线 `origin/ylj` 设为个人分支的推送目标。首次提交后用 `git push -u origin <本人姓名缩写>` 建立对应的个人远程分支（替换尖括号中的文字）。

若姓名分支已经存在，先切到该分支，保留原提交，再执行 `git merge origin/ylj` 同步共同基线；有冲突时逐项解决，不使用 `reset --hard` 或强制推送。若姓名分支仅在远程存在，先获取并跟踪它，不能同名重建后覆盖。若已按旧文档建立分组分支并有个人提交，先确认所属者，再将本人分支安全重命名或将其提交合入本人的姓名分支；不得丢弃旧工作，也不自动删除旧远程分支。

切换后核对：

```bash
git branch --show-current
git merge-base --is-ancestor 76de5bb HEAD
git log -1 --oneline
```

第二条应退出成功，表示包含本轮修复；随后打开本组 `student_skills/.../SKILL.md`。原来的 Python、Docker 和资产安装不必盲目重装，但必须按技能检查依赖版本和资产完整性。已有本地安装的旧技能也要更新为 `ylj` 中的整份技能目录，不能只替换 `SKILL.md`。

### 新建独立克隆

尚无仓库，或原目录由多人共用时，每人克隆到独立目录。下面的目录名可自行调整：

```bash
git clone --branch ylj https://github.com/youlj109/RoboLaw.git RoboLaw-personal
cd RoboLaw-personal
```

进入新克隆后，执行上一节的交互命令，输入本人姓名缩写并创建个人分支，再开始开发。已有个人分支沿用，不强制覆盖；共享工作目录不要同时切分支。

### 启动技能并开发

1. 读取仓库中的对应技能。Codex可安装整份技能目录后调用；Cursor可直接将SKILL.md加入文件上下文，要求Agent读取其相对引用。无需先依赖特定IDE插件安装方式。
2. 打开本人的A/B/C/D_PROMPT.md，将代码块完整复制给Agent。也可直接复制线上个人 Prompt 到当前主分支上的 Agent，它会先确认本人姓名缩写并处理分支，再读取技能、执行CPU预检。
3. 复现本组首任务，补齐未通过的新检查；随后逐个扩展任务，完成一个即可增量提交。不要等待另外三组。

本机可用环境为arena-kitchen-dev，仓库挂载/workspace。独立克隆后必须确认容器的/workspace实际对应本人的目录，否则脚本可能仍在执行旧仓库；同机多人运行还需将GPU锁映射到同一宿主机文件。Git不包含所有大资产、运行时、视频和原始数据；换目录或换机器须检查并恢复缺项。技能预检只验证元数据，不代表GPU/物理可运行。

## GPU协作与验收纪律

四人可并行开发和执行CPU检查，同机Kit/cuRobo任务使用共同GPU锁排队。runner已持锁时不要重复套锁，不终止他人任务。环境或挂载不同先检查再运行。

必须分别回答：原任务是否由机器人真实完成、场景物理是否正常、新实体是否确实要求避让。禁止以success=true、障碍可见或路线变长替代证据。多seed规划不等同多次完整采集；源保护距离违规不等同物理碰撞。

## 文件入口

- [A组直接可用Prompt](A_PROMPT.md)
- [B组直接可用Prompt](B_PROMPT.md)
- [C组直接可用Prompt](C_PROMPT.md)
- [D组直接可用Prompt](D_PROMPT.md)
- [修复总报告](../extensions/AVOIDANCE_REPAIR_20261022.md)
- 四份技能包：`exports/robolaw-student-skills.zip`（本机生成文件，不随Git提供；从云端获取时直接使用 `student_skills/` 中的完整目录）

assignment.json中的base_commit是最初分工时的历史基线；此次开发从origin/ylj及已包含的76de5bb修复开始。任务YAML与依赖哈希仍按原清单核对，不能随意更新哈希绕过不一致。
