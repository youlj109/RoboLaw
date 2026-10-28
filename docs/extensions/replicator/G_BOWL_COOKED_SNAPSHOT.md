# 原碗／水槽碰撞表示快照

这是独立诊断入口，不是采集执行器，也不改变正式接触规则。它创建同一原始任务、canonical layout、原碗 USD 和本组厨房物理 overlay 的新环境，reset 后暂停，只读原碰撞 prim 与公开 cooking API。没有控制动作、显式物理步进、物体位姿写入、缓存清除或成功样本输出。

```bash
bash scripts/arena_python.sh scripts/extensions/replicator/snapshot_bowl_cooked_geometry.py \
  --device cpu --physics-threads 2 --torch-threads 2 \
  --output reports/replicator_preparation/g_bowl_cooked_snapshot_02
```

真实运行需要 Kit，使用既有 `/workspace/reports/skill_trials/gpu.lock`，由根代理排队。`--dry-run` 在独立 CPU 进程中读取源 USD；正式入口在 Kit 初始化后才导入 USD，防止系统 USD 与 Kit USD ABI 冲突。启动前仍捕获执行源码、JSON 输入、资产和当前安装 API 定义的 SHA；API 文本按原始 bytes 保存，保留 CRLF。

新文件是 `cooked_geometry_snapshot.py`、`snapshot_bowl_cooked_geometry.py`、`g_bowl_cooked_snapshot.json` 和两份定向 CPU 测试。22 项几何／身份正反例通过；另有 2 项导入顺序正反例。当前冻结清单是 `reports/replicator_preparation/g_bowl_cooked_snapshot_freeze03/freeze.json`。首次启动因提前加载 USD 失败，原目录 `_01` 和 `startup_failure.json` 保留；进程退出 0 没有被算作快照成功。

## 读取与限制

- 返回每个凸体原始局部顶点、索引、polygon、plane，另存完整源 prim 变换及世界顶点。当前安装的 `ConvexMeshDataDemo.py:209–234` 直接用源 mesh 的 `ComputeLocalToWorldTransform` 变换返回顶点，是局部坐标约定的证据。
- 返回序号只表示本次 callback 顺序。安装 demo 明确说明凸分解顺序不稳定，不能当 native shape ID。
- 保存实际 `RigidBodyView.count`、`prim_paths`、`max_shapes` 和完整 offset 数组。文档将 `max_shapes` 定义为 view 中所有刚体 shape 数的最大值；只有实际确认单一 body 时，才能由此推导该 body 的 shape 数。仍然没有 shape slot 到 cooking hull 的对应关系，数量相等也不证明几何等价。
- property query 绑定 compound 的原路径、局部位置和 bounds。安装测试明确一组 convex decomposition 只回调一次，不能把回调数当凸体数。
- 原 `BasinBottom` 是 `boundingCube` 静态 collider。如果 cooking 或 rigid-body query 不支持，保留原错误和 unavailable，不加 RigidBodyAPI、不改 approximation、不造替代 floor。原静态 floor 的 offsets 也不猜为 0。
- 保存完整 composed stage、所有已用 USD layer 的内容／文件 SHA、前后原始位姿与物理步号，检查查询期间没有物理推进或场景变化。

结果始终 `accepted=false`、`physical_trial_passed=false`、`runtime_shape_equivalence_verified=false`。新初始化场景的 cooking 查询不是历史 probe 接触时的运行时 shape 导出，也不能证明 contact patch 与所录 post-step body pose 的时间完全相同。

## 与目标支撑规则的关系

`destination_contact_probe_01` 已独立记录 56 个释放物理子步，51 个原源三角形交叉失败全部保留；所有其他源碰撞对、完整 3649 状态和原相机通过各自检查。末态原支撑和 footprint 成立，但线速 0.06247 m/s 高于原 0.003 m/s，原成功谓词仍为 false。独立 actual-q CUDA 自碰撞全部为 0，其绑定复核另存 `destination_probe01_self_review01.json` 和 `destination_probe01_diagnostic_bundle02.json`；历史 bundle01 不覆盖。

下一规则只能针对原碗与原水槽底面的预期支撑接触：lower 保持 2 mm；其他所有 pair 保持原 2 mm；释放后绑定真实接触身份、源空间区域／实体下界、原向上支撑和连续状态，再保留原稳定成功谓词。快照用于量化源表面与 cooking 表面的几何差异，不能以历史最大穿透值加余量、contactOffset 数值或其他源支撑面的旧 band 直接生成新容差。正式规则仍待本次快照结果与根代理审核。
