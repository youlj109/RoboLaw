# 默认 gDist 网格距离后端

默认后端为 `gdist`，原生实现版本 `arena-gdist-runtime-1`。HS、PT v4、RS 使用同一个 CUDA 完整三角网格查询队列。在线采集只保存原始状态、接触、命令和冻结网格；风险距离、特征、等级、RGB 和视频仍离线处理。IR 关闭。

## 安装

现有 Arena 容器内执行：

```bash
cd /workspace
uv pip install --python .venv-arena/bin/python -r requirements-gdist.txt
.venv-arena/bin/python scripts/build_gdist_backend.py
.venv-arena/bin/python scripts/check_risk_perception.py
```

主机侧可用 `bash scripts/arena_python.sh ...` 执行同样的 Python 入口。新安装的 `scripts/install_arena_env.sh` 会安装这两个额外包并构建原生后端。构建需要 CUDA 开发工具链、C++ 编译器、Git、Python 开发头文件和 pybind11。当前验证环境是 RTX 5090 / CUDA 12.8；默认从第一张可见 GPU 自动选择 `sm_` 架构，也可传 `--arch sm_120 --cuda-home /usr/local/cuda`。

构建器从作者仓库 https://github.com/yingtix/DistanceQuery 获取固定提交 `0300e214f0c9702e83292f35a5d07bdc27ea14ba`，在独立构建目录应用 `native/gdist/gdist_linux.patch`。作者的 BVTT 距离界、展开和三角距离内核保留；`gdist_bridge.cu` / `gdist_parallel.inc` 提供无窗口接口、持久工作区、小网格精确穷举分流和并发 CUDA 流。

- 源码下载：`third_party/gdist/`，不提交 Git。
- 编译文件与构建清单：`runtime/gdist/`，不提交 Git。
- 构建记录包含源码固定提交、补丁/接口哈希、编译器、架构、命令和二进制 SHA-256。
- 加载前校验二进制哈希与源码固定提交；缺少安装或查询失败时明确报错/保留缺测，没有 Warp、CPU FCL、bbox、SDF 或采样距离的自动回退。

## 默认参数与距离口径

配置：`configs/safety_observer_arena.json/mesh_distance_backend`。

```json
{"id":"gdist","options":{"target":65536,"maxstep":8,"small":32768,"concurrency":64,"seeds":64}}
```

场景候选 frontier 默认 2。参数来自当前人体轨迹独立调参集上的实际场景流程搜索；其他硬件或场景不保证也是最快配置。完整测距保留全部 authored 三角形，包括退化三角形；曲面基础形状仍按记录的分辨率三角化。AABB 只用于保守候选筛选，不作为最终距离。

输出是 FP32 完整三角表面的无符号最小距离，不是实体穿透深度或材料损伤。RS 使用相同表面几何判断相交；HS 的 L3 继续依赖有效 PhysX 人体接触，而非单凭距离零值。

当前原生批量接口输出距离，保留最近碰撞体的路径；**最近点坐标未导出**，机器记录为 `nearest_points_world_m=null / nearest_points_valid=false`。当前等级和视频的距离依据不依赖最近点坐标。不得把缺失坐标补成零点；需要最近点可视化时须另行实现和验证。

HS 动态保护距离、PT v4、RS 的数值规则、已知结果聚合、缺测处理和任务豁免没有因本次更换测距后端而改变。固定支座距离、鞋子—支架指定接触对、瓶子任务距离与碗的豁免继续按冻结配置执行。停止模型仍属未标定实验参数。

## 新采集与历史数据

新采集的配置、profile、HS/PT/RS 几何元数据和原生构建记录都会写明 gDist。`.endModel()` 仅保留 CPU 网格，首次离线查询时才建立 GPU BVH。

```bash
# 场景规格由本次场景提供；不固定人体 ID
bash scripts/arena_python.sh scripts/risk_capture_workflow.py capture \
  --episode-specs datasets/my_scene_specs.json --output datasets/risk/my_run
bash scripts/arena_python.sh scripts/risk_capture_workflow.py evaluate \
  --output datasets/risk/my_run
```

默认评价使用每条数据冻结的后端：新的 gDist 数据使用 gDist，历史 Warp 数据继续按其冻结口径回放。若明确要用 gDist 重算历史数据：

```bash
bash scripts/arena_python.sh scripts/evaluate_raw_capture.py \
  --episode datasets/old_raw/env_000 \
  --output datasets/recomputed_gdist/env_000 --geometry-backend gdist
```

该操作在新目录写入派生 binding/profile、源与实际 profile 哈希、迁移说明及实际后端构建记录；不会覆盖原始数据或历史标签。阈值、豁免和物理状态沿用原归档，因此旧固定 HS 数据不会被暗中改为新动态 HS。完整工作流也支持 `evaluate --geometry-backend gdist`。

视频阶段使用风险输出目录的有效 binding，与本次实际 profile 哈希一致。中文字体可安装 `fonts-noto-cjk`，或放到 `runtime/fonts/NotoSansCJK-Regular.ttc`，或通过 `ROBOLAW_CJK_FONT` 指定；PyAV 是离线视频编码依赖。

## 适用范围与验证

实现按每条数据的场景、人体、受保护资产、刚体通道和冻结网格展开，不写死人体芥末瓶场景。无人场景的 RS 仍使用独立冻结机器人几何。当前人体案例的验证不代替其余场景验收。

验证记录见仓库 `validation/gdist_*_20261022.json`；运行输出位于 5090 的 `datasets/diagnostic/gdist_default_short_20261022` 和 `datasets/diagnostic/gdist_full_migration_final_20261022`。短段是动作预算截断的诊断，不代表完整任务成功。视频只保存在 5090 和用户本地，不上传魔搭或 Git。

### 2026-10-22 实测记录

- 原生 CUDA 预检通过，解析立方体间距为 0.25 m，默认 30 Hz / IR 关闭。
- 当前动态 HS：64 个时刻的诊断采集、离线风险、三视角 RGB 和视频链路通过；同一原始记录与 Warp 对照，规则集合、分项等级、类别等级、覆盖和缺测状态一致，最大选定数值差 0.887 微米。该短段未完成完整任务。
- 历史完整回放：2,292 个时刻、9,168 个物理步，保留其冻结规则；与原 Warp 输出相比，HS/PT/RS 等级及缺测状态一致，最大选定数值差 1.028 微米。gDist 离线风险处理总耗时 204.35 秒，其中几何阶段 122.70 秒；该耗时不包含新仿真或 RGB 渲染，也不是同机同期受控后端测速。
- 两次 gDist 风险回放均记录 `warp_runtime_loaded=false`。非几何缺测仍保留未知；几何完整不等于类别所有证据完整。

汇总：[集成验证](../validation/gdist_backend_integration_20261022.json)；[当前动态短段对照](../validation/gdist_current_code_comparison_20261022.json)；[历史完整轨迹对照](../validation/gdist_full_rule_comparison_20261022.json)。视频未提交仓库。
