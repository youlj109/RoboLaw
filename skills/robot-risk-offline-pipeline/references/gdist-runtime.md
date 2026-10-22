# 已调优 gDist GPU 完整网格后端

## 身份与可移植构建

使用作者仓库 `https://github.com/yingtix/DistanceQuery` 的提交 `0300e214f0c9702e83292f35a5d07bdc27ea14ba`，配合项目 `arena-gdist-runtime-1` 原生桥接。目录 `../assets/gdist-reference/` 附冻结构建脚本、原生桥接/补丁、Python 后端及 GeometryBatch。先读该目录 SOURCE_NOTICE；原作者源代码由构建时从固定提交取得，不包含 GPU 二进制。

安装目标机器相容的 CUDA 开发工具链、C++17、Python 开发头文件、pybind11 和 numpy。保持参考包相对目录结构后，使用目标 Python 执行 `scripts/build_gdist_backend.py --arch auto --cuda-home <actual_cuda_root>`。构建脚本会初始化/切换一个专用的上游 checkout；不要指向用户正在修改的代码目录。仅在获授权的软件安装/网络范围内执行。

参考机器是 RTX 5090、CUDA 12.8、sm_120；其他机器必须重新构建并记录环境，不复制 `.so` 假称兼容。构建生成 commit、native 源哈希、编译器/命令、架构、二进制 SHA-256。加载前核对源和二进制哈希。

不要 `pip install gdist` 当作本算法。也不要因后端失败自动改用 Warp/FCL/bbox/SDF/采样；失败明确报错或标对应几何未知。

## 已调优起点

```json
{"id":"gdist","version":"arena-gdist-runtime-1","options":{"target":65536,"maxstep":8,"small":32768,"concurrency":64,"seeds":64},"frontier":2}
```

这是当前场景/硬件的调优结果，不是全局最快承诺。`small` 是三角形数乘积的小问题分流阈值，仍精确遍历完整三角形；`seeds` 只是收紧 BVTT 上界的初始候选，不是用采样替代完整测距。`frontier` 控制同一帧候选波次，不是降低采样频率。

参考环境变量名仍叫 `ROBOLAW_WARP_BATCH_FRONTIER`，是历史命名；gDist 也会读取它。显式传 `GeometryBatch(..., frontier=2)` 或记录该环境变量，避免隐藏设置改变调优结果。

## 调度结构

1. 初始化只保存 CPU 完整网格和稳定模型身份；第一次离线查询才在 GPU 建 BVH，后续复用。跨刚体实例共享相同几何；位姿变化只更新相对刚性变换，不重建静态拓扑。
2. 同一时刻 HS、PT、RS 都调用 prepare，把关系和相交查询加入同一个 `GeometryBatch`。执行一次批队列，再解析结果；不要每对调用 distance→`.numpy()`→等待→下一对。
3. CPU 保守 bounds/BVH 剪枝对不可能赢得最小值的候选停止窄相；最终返回值来自完整三角形查询。保留稳定 tie ordering 和来源碰撞体 ID。
4. 原生 `batch_parallel` 使用持久工作区、页锁定结果缓冲、非阻塞多 CUDA stream，按槽推进不同对象对的 BVTT 与精确 narrow phase，再批量收取结果。小问题用精确穷举；全部三角形保留。
5. 相同几何对和完全相同位姿可缓存，缓存键包含两端模型/位姿/查询语义；不可仅按资产名复用上一帧。动态 articulation 每个活动连杆都更新，不能把整扇门当刚体根静态缓存。

当前实现仍有 BVTT 中间长度和结果的必要同步、每个结果 readback 统计。不要声称零 CPU/GPU 同步或已启用 CUDA Graph；当前 gDist 记录的 graph_launches 为 0。若继续优化，保留完整 mesh、身份、关系集合、误差和规则结果。

## 精度和输出

FP32 authored 三角表面无符号最小距离，基础曲面是有记录误差的三角化，不是解析曲面/实体穿透深度。网格表面完全包含而不相交不保证被表面相交检测当成碰撞；PhysX 接触是独立证据。

当前桥接输出距离和最近碰撞体身份，未导出最近点坐标：必须保留 `nearest_points_world_m=null, nearest_points_valid=false`。不可填零、bbox 最近点或随便一对顶点画“真实距离线”。距离 0 不能直接改判 HS L3。

## 性能验收

固定同一份原始轨迹、规则、完整关系集合、频率和几何精度，测冷启动/BVH、暖态 geometry、原始解码、特征、规则、序列化、完整离线耗时，以及每时刻 p50/p95、显存峰值、失败/缺测/缓存命中。GPU 利用率只作诊断。

根据目标 GPU、资产三角数/对数分布再搜索受实现允许的 target、maxstep、small、concurrency、seeds、frontier；保存每组实测与输出一致性。不要仅比较核函数耗时或用理论吞吐宣传端到端提升。发生容量溢出应调整调度/分块，不能静默截断三角形或关系。

当前最新厨房案例：离线总处理约 131.98 秒，geometry 阶段约 38.66 秒；视频仿真 76.4 秒。这是该案例记录，不是 articulation 或其他硬件的速度保证。
