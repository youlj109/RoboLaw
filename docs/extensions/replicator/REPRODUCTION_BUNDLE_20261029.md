# Replicator 重跑文件与历史证据分包

**上传状态：121个分包已在本地生成，尚未上传至 ModelScope，等待本机魔搭登录凭据；新分包目前没有可用的云端下载入口。** 完成上传并从公开地址核对后再使用在线恢复命令。持有本地分包的机器可以按下方离线流程验证和恢复；本文不会把本地生成误写成云端已发布。

这批文件补充 Git 忽略的 Replicator 规划输入、历史报告、实际状态、图像和视频，覆盖原始154景及物理重试第1—13轮，并保留源相对路径与原文件字节。代码、配置及最新结果仍从 `ylj` 分支获取；当前采用版本154/154、原布局151/154的历史区别不变。恢复文件不代表新的任务执行、安全验收或稳定成功率测量。

## 文件范围与清单

锁文件为[reproduction_bundle_20261029.json](../../../configs/extensions/replicator/reproduction_bundle_20261029.json)，SHA-256为 `a7d82724f9f6226b656d6080ca32146a6695834e2b475ca6b1e009cf21db94e2`。来源Git提交为 `092749effe29ea4033799abb1ac0dff311edc57e`，包ID为 `replicator-reproduction-20261029-r13-v1`。锁文件固定每个归档及对应逐文件清单的URL、字节数和SHA-256。每份 `<part_id>.manifest.json` 再固定原路径、字节数、SHA-256和文件权限；恢复器只接受清单内的普通文件。

| 选择 | 包数／文件数 | 归档下载字节数 | 恢复内容 |
| --- | --- | ---: | --- |
| `rerun`（默认） | 6／21,952 | 2,002,930,682（约2.00GB） | 小型规划输入／来源报告，以及精确场景、机器人、物体、人体和材质资产 |
| `evidence` | 115／23,986 | 33,470,652,091（约33.47GB） | 默认内容，加大型原生物理状态、接触记录和采集数据；不含媒体包 |
| `media` | 12／30,285 | 7,429,068,136（约7.43GB） | 默认内容，加历史图片和视频；不含大型状态包 |
| `all` | 121／32,319 | 38,896,789,545（约38.90GB） | 全部历史材料 |

表中为归档字节数，全部逐文件清单另占8,675,747字节。默认集合恢复后为5,538,341,390字节，全部文件为103,473,898,726字节；保存缓存和解包内容需要额外磁盘空间。默认6包是 `assets-001`、`assets-002`、`inputs-001` 至 `inputs-004`。

图片、诊断报告和原生状态分别绑定SHA。若需要完整诊断来源检查或重建历史网页，使用 `all`；单独选 `media` 不保证已恢复每个状态依赖。每包以约1GiB未压缩文件为分组目标，超过此大小的单个原生记录独立成包，不切断原文件。精确大小以锁文件为准。

分包不包含完整仿真运行环境、虚拟环境、编译缓存或凭据；固定版本的 IsaacLab-Arena、Isaac Lab、cuRobo 和运行环境仍按[仓库安装说明](../../../README.md)准备。PID、锁文件和缓存等不用于复现的内容不恢复。旧输入ZIP、旧视频ZIP、旧报告及它们的SHA保持原样。

计划发布位置为 [ModelScope 数据集](https://www.modelscope.cn/datasets/YouLinjing/Isaac-Arena-Kitchen-Safety) 的 `robolaw/replicator-reproduction-20261029-r13-v1/` 目录。该路径是预定位置；上方状态未改为已验证发布前，不表示这些新分包已存在于云端。

## 在线恢复：公开上传验证完成后使用

先获取 `ylj` 分支的最新代码，在仓库根目录运行。恢复器仅使用Python标准库，适用于本项目的Linux环境，不需要ModelScope SDK或下载Token。

```bash
# 默认：重跑输入和精确资产
python3 scripts/extensions/replicator/restore_reproduction_bundle.py

# 加载完整历史材料，包括大型状态与媒体
python3 scripts/extensions/replicator/restore_reproduction_bundle.py --profile all

# 只核查已恢复文件；不联网、不写入
python3 scripts/extensions/replicator/restore_reproduction_bundle.py --profile all --verify-only
```

默认缓存目录是 `assets/.downloads/replicator-reproduction/replicator-reproduction-20261029-r13-v1/`。中断后重复同一命令，下载器尝试HTTP Range续传；服务器不支持Range时重新下载该包，仍须通过整包SHA后才能使用。`--no-resume` 可明确重启未完成下载。

仅恢复某些包时用 `--parts <part_id> ...`；它与 `--profile` 互斥。包名取锁文件，不自行猜测依赖。`--verify-only` 需要选中包的已验证逐文件清单已在缓存中；首次运行缺清单会报错，不会悄悄联网补齐。

## 离线恢复与迁移

服务器上的完整分包目录是 `/home/oem/Code/RoboLaw/exports/robolaw_reproduction_20261029_r13_v1/`（仓库相对路径 `exports/robolaw_reproduction_20261029_r13_v1/`）。将需要的 `.tar.gz` 和同名 `.manifest.json` 原样复制到另一台机器，再使用Git中的锁文件验证。目标是已经存在的仓库目录；缓存与目标可以分开。

```bash
python3 scripts/extensions/replicator/restore_reproduction_bundle.py \
  --root /path/to/RoboLaw \
  --cache /path/to/robolaw_reproduction_20261029_r13_v1 \
  --profile all --offline

python3 scripts/extensions/replicator/restore_reproduction_bundle.py \
  --root /path/to/RoboLaw \
  --cache /path/to/robolaw_reproduction_20261029_r13_v1 \
  --profile all --verify-only
```

`--offline` 不访问网络，缓存缺失或校验失败便停止。已有文件与清单一致时跳过；任何同路径不同内容都会报错并保留原文件。先检查并备份本地修改，或在新的checkout中恢复，不要为通过校验改写历史结果。

恢复器拒绝路径穿越、清单外文件、重复路径、归档内的目录／软硬链接，以及目标路径上的符号链接。文件安装使用不跟随链接的目录句柄和拒绝覆盖的原子发布。下载缓存应放在自己控制的目录；这不是对抗其他本地进程恶意修改缓存的权限隔离机制。

恢复本身不会启动规划、渲染或物理执行。实际重跑应写入新的输出目录，保留已完成的历史manifest；历史 `--resume` 只用于其绑定且尚未完成的运行，不用于覆盖已有结果。相关执行入口和原始154景命令见[发布与输入恢复说明](RELEASE_154_PLANNED_SCENES.md)。

## 上传与公开地址核对

用户已授权上传；当前缺的是本机ModelScope SDK的有效登录凭据，不是再次审批。先在本机完成魔搭登录，凭据由SDK读取，不写入仓库、分包或日志。

上传助手默认只校验本地文件，`--upload` 才上传到既有数据集的新目录，不删除远端旧文件。`--verify-public` 直接从公开地址流式读取每个选中分包及逐文件清单，核对大小和SHA；它与 `--upload` 互斥。

```bash
# 无凭据也可运行：核对本地默认重跑集合
python3 scripts/extensions/replicator/publish_reproduction_bundle.py --profile rerun

# 本机魔搭登录完成后：上传全部分包
uv run --no-project --with modelscope-hub python \
  scripts/extensions/replicator/publish_reproduction_bundle.py --upload

# 上传完成后：从公开地址完整复核，不使用登录凭据
python3 scripts/extensions/replicator/publish_reproduction_bundle.py --verify-public
```

上传助手默认选择 `all`；`--profile rerun` 可用于只验证或发布默认集合，但不能据此宣称121包全部已发布。完整公开验证会读取约38.90GB归档，另含逐文件清单。上传调用完成本身不代表公开下载SHA已通过；只有对应公开验证完成后才更新本文的发布状态。

## 校验范围

恢复器已通过25项小型回归测试，包括篡改清单／归档、路径穿越、软硬链接、跨包重复、已有修改保护、并发创建目标及HTTP Range响应校验。另已使用真实分包器和pigz，在临时空目录中验证“生成5个小分包→默认恢复→补齐all→只读复核”，原路径、字节与0755权限均保持。

真实121包的清单与SHA链已核对，32,319条原路径全局唯一；还完整解压读取了全部归档成员并核对逐文件SHA，103,473,898,726字节全部通过。默认6包已在空Git归档目录恢复21,952个文件，并完成逐文件哈希验证。另一次独立新目录检查通过原始154景、当前采用版本154景及全部13轮冻结计划的输入加载：0次读取旧仓库数据、0次向新根目录写入，没有网络、GPU或Kit运行。

该独立检查另外准备了34个上游依赖文件，与既有固定源码镜像的SHA一致；这些仍属于安装流程中的依赖，不能把分包误认为包含完整仿真环境。这些是文件恢复及CPU输入检查，不是重新执行154次物理仿真。新目录目前尚未上传，远端下载验证尚未完成。
