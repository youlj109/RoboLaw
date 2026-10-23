# 学生A：Replicator 厨房抓放启动 Prompt

将下面代码块中的全部内容复制给 Codex 或 Cursor Agent。适用于当前仍在 `main`/`master` 的已有克隆：Agent 会先确认本人姓名缩写，再以 [ylj 开发基线](https://github.com/youlj109/RoboLaw/tree/ylj)创建或同步个人分支，再读取技能。仓库路径由当前工作目录确定，无需替换占位符。应在本人的独立克隆中执行；若误打开他人的共享工作目录，先建立独立工作目录。

```text
你正在协助我完成 RoboLaw 安全任务增广。我是学生A，独立负责Replicator 厨房抓放，共14个原始环境。

我的技能文件：student_skills/robolaw-replicator/SKILL.md
我的任务清单：student_skills/robolaw-replicator/references/assignment.json
我的个人分支：我的姓名缩写本身，例如zxw、lgd、lwx，不加ylj-或任务组前缀。若上下文已有我明确提供的姓名缩写，直接使用；否则先询问我“你的姓名缩写是什么？”，得到答案后再处理分支。不要根据A/B/C/D、Git作者或示例猜测我的姓名。
首任务：kitchen_bench_replicator_g_shape_mustard_bowl
共同开发基线：origin/ylj，包含修复提交76de5bb。
ylj参考：https://github.com/youlj109/RoboLaw/tree/ylj
我目前可能只安装了main/master；先完成下方“二、立即开始”的基线准备，再读取技能或运行仓库脚本。

请实际执行开发，不只给计划。取得ylj基线并进入个人分支后，读取本组SKILL.md、references/review-gates.md、references/workflow.md、references/assignment.json，以及SKILL链接的实测参考，重点读取references/repair-20261022.md。不要假设历史accepted=true自动满足新验收要求。

一、职责和边界
先复现修复后的 G 型芥末瓶入碗，再扩展同布局其他操作，最后迁移其他布局。
当前状态：首任务三类已修复重采；人体有三个 seed 的转运段规划对照，财产和横杆只有单基线必要性证据。
任务ID、原YAML和源码哈希从assignment.json逐项核对。不要开发其他组任务。
只在以下本组目录实施：
kitchen_safety/extensions/replicator/
configs/extensions/replicator/
scripts/extensions/replicator/
tests/extensions/replicator/
assets/extensions/replicator/
docs/extensions/replicator/
如发现公共组件问题，记录最小复现和所需改动，避免覆盖其他组实现。不要终止其他人的任务或在共享目录切换分支。

二、立即开始
1. 先确认本人姓名缩写。未知时先询问，等待我的答案；这期间可以只读检查仓库，不创建或切换个人分支。获得答案后将其作为本次任务的student_branch值；后续命令执行前确保该变量已按答案赋值且非空，使用git check-ref-format --branch "$student_branch"验证，禁止使用main、master、ylj或ylj-分组名。不同shell调用不会自动保留变量，应重新赋值或使用已确认的具体姓名分支名。
   执行git status --short --branch和git remote get-url origin，确认是在本人的独立克隆，且origin指向youlj109/RoboLaw（HTTPS或SSH均可）。远程不符时先报告，不擅自改写远程。保留所有未提交修改；工作区不干净时不要直接切换/合并，不reset、clean或自动stash，可保留原目录并建立独立克隆继续。
   在确认远程后显式执行：git fetch origin refs/heads/ylj:refs/remotes/origin/ylj
   该命令兼容只获取过主分支的single-branch克隆；不要只用git pull main代替。
   先检查本人姓名分支是否在本地或远程存在。在干净的个人工作区，确认本地和远程均不存在该姓名分支且student_branch已按本人答案赋值后执行：git switch --no-track -c "$student_branch" origin/ylj
   若姓名分支已存在，保留其提交，在student_branch已赋值后执行git switch "$student_branch"；用git merge-base --is-ancestor origin/ylj HEAD检查是否包含最新基线，缺少时审阅差异并执行git merge origin/ylj，解决冲突时保留双方有效工作，禁止强制重置或丢弃已有工作。若只在远程已有个人分支，先获取并沿用该分支，再检查基线，不另建同名分支覆盖其历史。若已按旧文档创建分组分支并有个人提交，先确认所属者，再安全重命名为本人姓名分支，或将其工作合入已有姓名分支；不得丢弃提交，也不自动删除旧远程分支。
   不从main/master直接开始开发，也不要将origin/ylj设为个人分支的upstream；已有误配置应取消，个人upstream在提交时用下述git push -u设置。
   完成基线准备后，再读取上文列出的本组技能及参考文件。assignment.json中的旧base_commit用于溯源，不要据此回退工作分支。
2. 执行：python3 student_skills/robolaw-replicator/scripts/preflight.py --repo "$PWD"
3. 核对主分支安装时已有的依赖、资产、容器挂载、GPU资源与运行时；不要盲目重装。切换代码分支不会自动下载Git忽略的资产、数据或视频。独立克隆需确认容器/workspace映射到本人当前克隆，不能误运行原共享目录代码，也不要擅改他人正在使用的容器挂载。若本机提供arena-kitchen-dev，可执行：bash scripts/arena_python.sh student_skills/robolaw-replicator/scripts/runtime_inventory.py
4. 先复现首任务，确认各验收报告及视频，再逐个扩展剩余任务。不要把首任务入口仅替换名字后冒充其他任务可运行。
5. 如果缺资产或运行时，按仓库文档恢复；无法恢复时准确报告缺项，继续能独立完成的源码/配置/CPU工作，不伪造物理成功。

三、每个任务的实施要求
- 保留原任务对象、目标、机器人、布局关系、成功谓词及安全距离；初始化必须合法。
- 先做机器人中性保持，检查目标及未操作门/旋钮等机构，观察时长覆盖完整任务。自由物体自然沉降需预先说明并完整记录；关节异常不能隐藏在沉降期。
- 先得到代表性的较短无新增风险成功基线，再设计human人体、property财产、robot障碍物三个变体。robot在本仓库是障碍物类别名。
- 新实体必须有合理支撑，不与源厨房、台架或必需操作几何非法重叠。不是放进场景就算有效增广。
- 将基线放入新增实体的几何模型做离线反事实，源碰撞网格或真实primitive确认风险。仅保守盒/胶囊违规不能当作真实几何必要性。保护距离不足不能写成必然碰撞或破碎。
- 相同初态、目标、控制策略及参数下，用预声明seed42/43/44做障碍开关成对规划。保存全部成功/失败。危险旧路径只能离线检查，不在含障碍场景执行。
- 同一把手的合法抓点适配和固定起终点绕行分别报告；不人为添加绕行路点或挑异常长基线来制造差异。
- 真实执行靠机器人接触完成。禁止传送目标、逐步设置目标关节、仿真焊接携物来伪造成功；规划碰撞attachment与物理约束必须区分。
- 使用各任务原安全参数，不将参考任务20/50/10mm等数值无条件复制到所有任务。

四、最小交付
每个原始任务提供匹配的无新增风险成功基线，以及human/property/robot各1条完整真实成功数据。当前不要做200条批量采集，也无需等待其他学生的执行器。
保留原始轨迹、动作、接触、物理审计、原三路视频；人体必要时加同步总览。生成原速同视角基线/新执行对照视频，完整解码并实际查看关键帧。
独立检查原成功谓词、源布局关系、台架、自碰撞、外部安全距离、财产稳定性、背景机构稳定性、图像/状态同步。报告动作率和物理率的实际覆盖范围，不能把部分阶段审计写成全程。
三个规划seed不是三条物理采集；一条成功不证明批采成功率。未测项、失败候选和旧数据保留。

五、资源与提交
本机Kit/cuRobo统一使用/workspace/reports/skill_trials/gpu.lock；多个克隆/容器中的该路径必须映射同一宿主锁文件，不能各自新建一把锁。遵循本组runner：已内部持锁的不再外套同一锁；其他GPU入口须按技能加锁。CPU工作可并行。--device cpu不保证不使用GPU。
每完成一个原始任务，确认当前分支与已确认的本人姓名缩写一致后提交；确保student_branch已按本人答案赋值，再执行git push -u origin "$student_branch"，附配置、复现命令、验收报告和视频路径。检查Git diff，只提交本任务源码/配置/文档，遵守已有大资产与数据忽略规则；数据/视频另行列路径及哈希，不宣称已随Git上传。
收尾用任务表列出：已完成、失败、未验证、下一任务。不要降低验收条件来凑数量。
```

共用分工和交付标准见 [任务安排与快速开始](README.md)。本组技能入口：[SKILL.md](../../student_skills/robolaw-replicator/SKILL.md)。
