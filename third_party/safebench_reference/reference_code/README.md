# SafeBench → Isaac Lab Arena 参考代码

用途：将源端 HS、PT、RS、IR 安全采集与评价接入现有 5090 Arena 厨房 Pick and Place 工程，新增评价只记录，不按风险等级修改动作。

这是必要源码的参考快照，不是已经实现的 Arena 插件。源端文件保持原样，来源为 `safebench:/root/autodl-tmp/SafeBench_gpu661`；逐文件路径、大小、SHA256 和导出时间见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。未打包资产、视频、训练数据、凭据或整个仓库。

## 目录与阅读顺序

| 路径 | 用途 |
| --- | --- |
| `evaluator/safety_risk/evaluator_factory.py` | 选择实际规则实现 |
| `evaluator/safety_risk/iso_ssm_candidate.py`、`iso_traceable.py`、`pt_hs_inspired*.py` | 四类规则和 PT 判断依赖 |
| `evaluator/safety_risk/iso_input.py` | 原始数据到评价特征的映射 |
| `evaluator/safety_risk/online_iso.py`、`observation_cadence.py` | 动作前输入、观察槽与物理执行引用 |
| `evaluator/safety_risk/online_report705.py` | 已记录在线风险的报告汇总 |
| `evaluator/scripts/evaluate_iso_risk.py`、`verify_online_iso.py` | 离线评价及冻结输入重放校验 |
| `evaluator/safety_risk/grip_evidence_binding.py`、`human_presence.py`、`task_success.py` | 抓持绑定、人体清单及源 single-pick 成功判据 |
| `evaluator/safety_risk/configs/` | 两份源 profile：测试基线和 Franka 场景参考 |
| `evaluator/safety_risk/tests/` | 四组原始回归测试，共 18 项 |
| `evaluator/workflows/simbox/core/configs/robots/` | 四份测试所需机器人配置，仅作测试夹具和绑定参考 |
| `sampling_reference/safety_risk/` | 原 PhysX 采样器及接触、相机、对象、关节和环境隔离参考 |
| `arena_templates/` | 新增配置模板及环境探测脚本，目标工程需要主动接入 |

评估器目录包含静态解析得到的仓库内 Python 导入依赖。采样参考目录刻意与它隔开：采样器还依赖旧 Simbox、Isaac Sim 和控制组件，未提供其完整运行环境，不应直接加入目标模块搜索路径。

## 先运行离线测试

进入 `reference_code/evaluator`，使用目标 `arena_python.sh` 对应的容器 Python 解释器。以下是解释器内运行的命令，并非声称 wrapper 支持直接透传 `-m`。

```bash
python -m unittest \
  safety_risk.tests.test_online_iso \
  safety_risk.tests.test_online_observation \
  safety_risk.tests.test_grip_evidence_binding576 \
  safety_risk.tests.test_observation_cadence -v
```

四组离线测试需要 PyYAML；可选受控输入证据依赖 NumPy。使用目标已有依赖；缺少时在独立环境安装，不因此升级现有 Torch、CUDA、Isaac Lab 或 cuRobo。纯规则与测试不需要启动仿真。

`evaluator/requirements-reference.txt` 只列出直接依赖名，不是目标 CUDA 环境锁文件。离线测试结果见 [LOCAL_VALIDATION.md](LOCAL_VALIDATION.md)；它不能代替 Arena 物理采样、三视角及四环境验证。

## 不介入动作时的使用边界

1. 从评估器入口开始复用，Arena 只读适配层负责提供实际数据。不要把旧采样器整体替换进现有执行循环。
2. `online_iso.py` 中 `mode="shadow"` 只标识评估模式；仍需通过目标动作发送边界证明外层没有风险控制分支。
3. `controlled_ir.py` 是保留源码依赖，内含历史受控输入实验的注入类，以及用于重放的 `project_injection`。本次不要实例化 `ControlledIRInjector`，不要开启输入扰动，也不要伪造 `controlled_ir_evidence` 来填补自然感知信息。
4. `sampling_reference` 的旧 collector 源码包含介入组件引用。本包保留它是为了阅读采样顺序、字段和接触口径，不授权目标适配层调用其中的 hold、制动、夹爪调力或风险退出逻辑。
5. 源机器人 YAML 可能包含 drive 等参数，只用于原始测试；不要覆盖目标机器人配置。

## Profile 必须重新绑定

- `iso_profile.experimental.json`：原始单元测试基线。
- `iso_profile.franka_scene569_shared.json`：当前源场景参考，包含原机器人、对象和规则参数，不能原样当作 Arena 机器人配置。
- 保留原 profile，复制为目标的新 profile 后，绑定真实机器人、关节、夹爪、人体、受保护对象、任务阶段和物理时钟。参数保留来源和未校准标记。
- `task_success.py` 的源单任务判据是 single-pick；目标完整 Pick and Place 继续使用原成功判据，并补充释放和最终稳定性证据。

## 交给目标工程助手的执行说明

阅读上一级 Markdown 部署说明和本目录，再核对目标工程实际入口。按照单环境新 run → 四环境 → 11 场景逐步接入。保留 CuRobo/FCL 原规划约束及三视角流水线，不新增风险等级驱动的动作写入。对成功、失败和高风险 attempt 全部归档；分别报告任务完成、数据完整、安全检查与障碍约束确认。

已有 human_default20 批次不能计作新 observer 验证数据，其 `obstruction_unconfirmed` 保留。本包没有给出假定已存在的目标 CLI 选项，也没有包含已验证可运行的 Arena 适配层。
