# C01–C08 最终审阅交付登记 · lwx · 2026-10-27

当前共24个变体，每任务human/property/robot各选1条成功执行。selection.json精确对应本次总审阅包，含每组所选数据、匹配基线、配置、现有审计与媒体相对路径。媒体相对路径指向ZIP内布局，并非这些视频已上传Git。

本包在上一版20组基础上更新4组：C04 human h568＋b563；C04 property p291＋新匹配b318（318不是新的财产实采）；C06 human h524＋b515；C08 robot r67＋b67。C04h568最终夹持相对旧h431实测转平20.03度，用户已回复“就这样吧”；C06h524和C08r67也有明确用户认可。固定布局单条成功不证明批采成功率。

18组附所选修订的独立验收与所测seed配对，6组沿用旧版。C01robot缺部分新版独立审计和本轮多seed/源相交证据；C02property、C03human、C07三组未附本轮统一多seed报告，保留其历史成功和审计。详见验收覆盖表.md；不能宣称24组都统一满足最新加严标准。

总包：RoboLaw_C01-C08_lwx_review_20261027.zip
大小：283359405字节。
SHA256：363607a6de4d9005d31bddc23736132abcf415a2c0fbe027d21487f7646e12dd。
解压后打开index.html：24个主视频、18个去重匹配基线、4个新版原速对照、25个HTML页面；逐文件复制SHA256、499处页面引用、ZIP CRC完整性检查均通过，详见package_verification.json。

本机位置：D:/浙江大学/Post_Graduate/EmbodiedAI_2026B/RoboLaw_outputs/lwx_C01_C08_review_20261027/。
远端仓库：/home/youlj/projects/wenxuanli/RoboLaw。完整原始数据、物理子步、旧版和失败仍在selection.json所列datasets路径。视频、HDF5/RGB和二进制人体资产包另行交付，不加入Git；C04H_C06H_runtime_assets530.tar.gz及原路径恢复清单见C06的runtime_assets_manifest.json。

打包时执行代码及主要验收文档版本为30b37e8089085bbabe03065ac31820abe3017c6d；本次提交仅补齐总包登记、C04最新认可和C06中文交付说明，不修改执行代码、数采、资产或任何验收阈值。历史原报告中的pending是生成时状态，当前用户认可由独立确认记录补充，不改写原始审计。
