# 现有 RoboLaw 入口与通用实现映射

在已有仓库中，读取项目实际根目录的 AGENTS.md。默认策略为 `configs/risk_capture_workflow.json`，入口为 `scripts/risk_capture_workflow.py`。下列路径均相对项目，不要求某个 SSH 别名或主机绝对路径。

```bash
python scripts/risk_capture_workflow.py capture --episode-specs <本批任务规格JSON> --collector-overrides <本批控制参数JSON> --output <新运行根目录> --run-id <本批ID>
python scripts/risk_capture_workflow.py evaluate --output <运行根目录>
python scripts/risk_capture_workflow.py render --output <运行根目录>
python scripts/risk_capture_workflow.py video --output <运行根目录>
```

没有覆盖参数则省略 `--collector-overrides`。Isaac Sim 环境用项目实际 launcher/container 包裹 Python，不能假设当前宿主 Python 可以导入仿真依赖。

- capture 单独结束，不自动开始后续；保存 pending_offline。
- evaluate 先做原始完整性校验，再按冻结后端/规则回放，输出 offline_replay。
- render 必须用相同原始 episode 的测量状态，不重新跑一条“看起来相同”的物理轨迹。
- video 使用本次风险目录的有效 binding/profile，与 RGB 来源相同且源时间对齐。
- 原入口的 audit 是历史独立 clearance 诊断，不是默认 HS/PT/RS 查询口径，也不自动遵守新的风险距离省略清单。不要默认启动它重新算已授权不要计算的距离；用户确实需要该独立诊断时单独说明范围。
- 同一运行的阶段写同一 ledger，不并发调用造成丢失；要并行离线计算与渲染须先实现无竞态状态记录，并确认显存/吞吐收益。

参考代码文件：`kitchen_safety/raw_capture.py`、`frozen_geometry.py`、`risk_binding.py`、`hs_dynamic.py`、`pt_assets.py`、`rs_evidence.py`；风险规则来自第三方参考评价器再经 Arena 动态 HS 与聚合包装。不同工程按契约实现，不直接复制带固定任务变量的适配层当成通用系统。
