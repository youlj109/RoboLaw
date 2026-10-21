# 参考包本地验证记录

验证日期：2026-10-19。

- 源端导出的 35 个文件逐一通过 SHA256 校验，内容保持未修改。
- 解包后在 `evaluator/` 目录运行四组原始测试，18 项全部通过。
- 验证环境：本地 macOS ARM64，Python 3.12；缺少的 PyYAML 6.0.3 安装在独立临时目录。未修改目标容器或项目依赖。
- 最初运行因本地缺少 PyYAML 未能导入抓取绑定测试；补齐依赖后重新运行全部四组，结果通过。未修改测试或源代码来规避问题。
- 覆盖冻结输入与回放、缺失数据、因果与执行引用、抓持绑定、观察频率、短接触及陈旧相机。
- 没有运行 Isaac Sim、目标 Arena、RTX 5090 CUDA 扩展或四环境物理采集；`sampling_reference/` 未作为可运行包进行验证。

```text
test_online_iso: 5 passed
test_online_observation: 5 passed
test_grip_evidence_binding576: 4 passed
test_observation_cadence: 4 passed
Ran 18 tests in 0.038s
OK
```

“参考评估器测试通过”不等于“目标 Arena 部署完成”。后续需执行部署说明和验收清单中的目标环境验证。
