# gDist 可迁移参考构建包

此目录保存 2026-10-23 实际部署的原生桥接、Linux 补丁、构建器、Python 缓存后端和帧级 GeometryBatch，原始项目相对路径及 SHA-256 见 SOURCE_MANIFEST.json。文件没有改写成另一种距离算法。

保持 native/、scripts/、kitchen_safety/ 相对结构可作为独立 gDist 构建参考。编译需要网络可访问的固定作者源提交或另备的干净 source checkout，以及目标机器 CUDA/Python 编译环境。这里不包含作者仓库完整源码、GPU 二进制、仿真器或任务适配器，也不是安装整个 RoboLaw 的脚本。

作者仓库：<https://github.com/yingtix/DistanceQuery>，固定提交 `0300e214f0c9702e83292f35a5d07bdc27ea14ba`。作者 BVTT/三角距离核心由构建器获取，保留原作者许可和归属；本包不重新授权上游。RoboLaw 自有贡献的项目许可随 LICENSE.robolaw 保留。

`geometry_backend.py` 还记录历史 Warp 身份和迁移路由；此参考包只附 gDist，实现新后端时不得走缺失的 Warp 回退。直接使用 gDist 构建器、`get_backend('gdist', options)` 和 GeometryBatch；相同代码行里出现 legacy FCL/Warp 名称不意味着当前实际运行了那些距离后端。

当前后端无最近点坐标输出、无已启用 CUDA Graph，仍有必要 BVTT 同步和读回；不要在迁移报告中声称相反。具体性能合同见 ../../references/gdist-runtime.md。
