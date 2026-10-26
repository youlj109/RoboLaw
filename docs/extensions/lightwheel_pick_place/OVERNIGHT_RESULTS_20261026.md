# RoboLaw lgd 夜间结果（2026-10-26）

.git .gitignore .pytest_cache LICENSE README.md THIRD_PARTY_NOTICES.txt assets configs datasets docs exports kitchen_safety reports requirements-collection.txt runtime scenes scripts student_skills tests third_party  B03、B04、B05 全部验收完成。下面区分真实任务成功和安全任务验收。

| 任务 | 已完成 | 未通过 |
| --- | --- | --- |
| B03 双香蕉 | 匹配基线、human 原任务均真实成功；human 独立物理检查通过；120秒静置通过 | human 未证明有效避障；property 第二次放置手指间隙不足；robot 抓取预规划未通过 |
| B04 汤勺 | 基线真实成功，接触/源支撑/全路径/自身碰撞通过；90秒静置通过 | human 真实汤勺路径人体间隙不足；property 抬起时花瓶间隙不足；robot 固定横杆路线未通过 |
| B05 百事 | human、property 均真实成功，并通过独立物理、安全、有效避障、三种子42/43/44对照和90秒静置 | robot 三种子规划通过，但真实采集在释放阶段超时，缺少完整轨迹和原任务最终结果，不能算完成 |

## 可直接在 JupyterLab 下载

- [B03 未完成证据包](B03-review/B03-partial-evidence-20261026.zip)
- [B04 未完成证据包](B04-review/B04-partial-evidence-20261026.zip)
- [B05 property 已通过结果包](B05-review/B05-property-results.zip)
- [B05 human 已通过结果包](B05-review/B05-human-results.zip)
- [B05 全部尝试与缺项](B05-review/current_status_20261026.json)

## 原视频和效果图

- B03：B03-review/native_success/human-task.mp4、human-overview.mp4。
- B04：B04-review/native_success/base-external_camera.mp4、base-external_camera-final.jpg。此原相机能清楚看到汤勺在盘上。
- B05：B05-review/native_success/ 下的 human-*、property-*，保留原始相机视频。human-overview-final.jpg 为真实补充相机最终帧。

B05 human 对照明确复用原成功基线：控制/布局/重置参数一致，并重新证明原轨迹关节余量0.233rad满足新增0.02rad要求；不是冒充新采基线。兼容性证明和新复核报告均入包。

B03 用户批准的障碍布局未挪动；B04/B05 留存场景审核图。未批采200条，未继续调查门和旋钮。>

#
> lgd 分支。GitHub 无凭据，尚未推送；完整历史 bundle 另存，最终版本路径见同目录 lgd-overnight-final-20261026.bundle。原数据 images.h5 和 scene.usda 保留在各报告指向的服务器 episode 目录，不因打包删改。
