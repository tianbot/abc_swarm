# 从 origin/main 重组：范围、取舍与验证

后续指导方针：Python 实现可复用核心算法，关键参数可配置，launch 负责设备适配和实验组合。用户确认四个新节点可并入 nodes；当前结构以下文及根 AGENTS.md 为准。

## 基线与分支

新分支 `teaching/main-reorg` 直接从已同步的 `origin/main` 创建，基线 `1708f332ae900384b9e84889b7ab3979aaae9f5f`（2023-02-15）。不合并 refactor 分支，也不把旧分支整批内容导入。代码提交 `d79ae73` 的直接父提交就是该 main 基线。

前一轮成果保留在 `demo/mini-five-vrpn` 的 `ef58eeb`；完整归档在 `/tmp/abc_swarm_main_reorg_backup/previous-work.tar`，用户未跟踪文件另有副本与 SHA256。选取清单在 `tests/main_baseline.json`，可追溯每个新文件的来源。

用户后续明确：本轮主要增加动捕 demo，不把原来无需动捕的实车实验转换成 Gazebo，也不新建仿真版。中途暂放的 simulation/sim_demo 文件已经移除。

## 原版保持

- 原 `nodes/` 九个脚本逐字节、原路径保留。后续获用户确认，四个新增节点从 scripts 并入 nodes，保持平层，共十三个可执行文件。
- 原 `launch/includes/` 两个文件逐字节、原路径保留。新公共组合放在 `launch/common/`，没有改旧 include 引用。
- 原课程其余 launch 原位、原内容保留，原 `demo_sim_*` 名称和无动捕实车实验不改。
- 两个明确获准的例外：`launch/demo/demo_mc_tf_tt_circle_mini.launch`、`demo_mc_tf_tt_follow_mini.launch` 移到平层 `launch/mocap/`，更名为 `demo_mocap_tf_tt_*`，文件内容不改。它们是旧空地参考，不自动纳入新三车生命周期。
- 原 `config/tf_pid.cfg`、RViz、RST 文档、图片和旧站配置原样保留。基线清单共覆盖 48 个原始资源，除两个路径更名外，SHA256 均保持。
- 根 README.rst 增补新入口；CMake/package.xml/setup.py 只为安装、依赖、共享计算及测试作必要调整。
- 用户两个导弹实验文件未移动、未修改、未提交。

原 `demo_formation_leader_follower_path_tracking.launch` 包含底盘、雷达、AMCL、编队和路径控制，实际是地图定位实车实验。学习它的完整实验组合，不把该文件简单改名成 sim，也不把混合实车内容的整个旧 demo 目录改成仿真目录。

## 新布局

```text
nodes/                         原九个脚本 + 四个新节点，平层
src/abc_swarm/                 三份共享计算，平层
config/                        tf_pid.cfg、mocap.yaml、path_tracking.yaml
launch/demo/                   原课程与原无动捕实车/Gazebo 入口
launch/includes/               原片段，不变
launch/common/                 四个新增公共启动片段，平层
launch/mocap/                  新动捕与原动捕参考，平层
```

四个新节点均在 nodes，新脚本为 `leader_follower_tracker.py`、`pid_tracker.py`、`preset_path_tracker.py`、`demo_runtime.py`，命名不区分仿真和真车。路径、Pure Pursuit 和 leader-follower 计算可共用；本轮没有增加另一套仿真入口或纯 Python 课程。内部理想运动学模型仅放在 tests 中用于控制回归。

## 摘取与舍弃

| 内容 | 决定与依据 |
| --- | --- |
| 三车 LF 三角形、逐车跟随 | 摘取；这是用户已在旧工作分支实测的核心新增能力 |
| 三车 PID | 摘取作为对照，修正后的限速/有效性处理保持，不带五车模板 |
| 单车/三车圆形、圆角矩形路径 | 摘取；使用共享路径计算与跟踪器，分别提供独立三角/蛇形入口 |
| 一文件完整启动、场地配置、统一服务 | 摘取；便于现场和后续 GUI，保留默认禁用与后台复用 |
| 五车、PX4、长蛇、人字形、全局速度工具 | 暂不带入；当前只有三车，非本轮必要能力或尚待专项验证 |
| pid_tracker_rc1、旧新命名并存、额外 demo/examples/legacy | 不带入；避免重回 refactor 混杂布局 |
| 旧监控频率估计、浏览器路径依赖 | 不带入；当前就绪检查由新运行节点直接检查实际数据 |
| 大量上一轮计划/过程记录 | 不带入；本分支只保留必要操作、接口、选择记录及网站草稿 |

新实验统一 `demo_mocap_*` 前缀，详见 `launch/mocap/README.md`。新名字以原版的“demo + 环境 + 实验”写法组织；旧视频文件名除明确授权的 mc 两项外不变。

## 保留的统一能力

六个新的 Mini 实验默认组合 socket 驱动、VRPN、算法及所需键盘/RViz，运动默认禁用。`config/mocap.yaml` 集中 server/port，CLI/环境变量可覆盖；`config/path_tracking.yaml` 保存路径控制参数，两者均不建二级目录。

GUI 可独立保持 `mocap_backend.launch`，运行实验时关闭重复后台、键盘和外部 RViz。统一 SetBool 启停、Trigger 重置、JSON 状态及头车遥控门控保持前一轮已验证的语义；只读定位/里程计检查不是完整硬件健康证明。

## 文档方向

建议新增实验内容进入 docs.tianbot.com，保留原视频 URL，原 RST 暂存归档，不再同时维护两套新增内容。本轮提供 `docs/website/mocap-experiments.md` 页面草稿与 `docs/README.md` 迁移建议；没有改外部网站仓库或发布网站。

旧 RST 构建配置和缺页问题是 origin/main 已存在的状态，保持原样，未把修复旧站纳入本轮。原始 launch 中的既有语法/行为问题也不夹带修改，兼容检查证明未改变基线，不声称重新验收全部旧实验。

## 验证证据

- 全新 `/tmp/abc_swarm_main_verify` catkin 构建通过，**18 项测试，0 错误、0 失败、0 跳过**。覆盖 48 个原始资源、nodes/includes 文件集合、浅目录、公开命名、后台唯一性/复用、配置覆盖、几何路径、闭合圈数、运动学收敛、实际速度回调与跟随拓扑。
- 私有 localhost:11433 ROS master 的合成测试通过，日志 `/tmp/abc_swarm_path_ros_a6jw_m78`。实际启动六个新增 Mini 实验，覆盖单车/三车路径、三角/逐车手动领航、PID 输出、GUI 服务、旧指令不重放、输入超时、定位/里程计中断与共同停车。数据和话题统一为 path_test_*，没有连接现场 master。
- 全新 `/tmp/abc_swarm_main_install` 中，原有课程名称和九个 mocap/backend 文件名可唯一查找；原节点与新脚本、共享模块和参数可发现，新 mocap XML 组合可展开。没有启动节点作资源查找。
- 实际用户工作区选择性构建通过，13 个现行可执行名称与全部动捕入口可发现。旧分支遗留的七个、且仅指向已不存在源码的 catkin 自动生成 relay 已先归档再移除，备份 `/tmp/abc_swarm_main_reorg_backup/retired-devel-relays/`；其他包、用户编辑器配置不动。
- 新 README.rst 可无警告渲染；现行 Markdown 本地链接及 diff 格式检查通过。

## 验收边界

本轮没有启动 Gazebo、真实驱动、VRPN 客户端、键盘窗口、Mini 或飞行器。新分支的实际设备效果和停车仍需现场复验；不能把先前分支的三车反馈直接标成此分支实车验收。新网站页面尚未发布，代码只作本地提交，不推送。

## 后续优化评估

- 已合并节点入口；保留旧脚本正文和可执行名，不增加层级。项目原则写入根 `AGENTS.md`。
- 已消除路径收敛中的硬编码前置点距离，与跟随器共同使用 `front_distance`。
- 已分清参数归属：路径几何、速度、圈数由 demo 的 launch 参数配置；算法前视距离、容差等在 YAML 中。删除 YAML 中被 launch 同名默认值遮蔽的重复字段，不改变默认实验参数。
- 核心算法与设备型号分开；共享 TF/Path/Twist 输入输出不能替代运动模型校核。原 RMTT 控制会输出 linear.y，当前 Mini 控制输出 v/omega，后续应复用适用的控制律或增加必要适配，不靠改话题声称支持新设备。TOM06S 的模型与接口需要查实际驱动后判断。
- 后续优先整理各算法的输入/输出、关键参数、参考坐标和运动模型说明；无需先建插件系统、更多包或更多层级。preset_path_tracker 暂保留完整实验流程，不为了形式拆成更多节点。

后续合并节点的验证：20 项测试通过，包含原九个脚本内容不变、新前置点参数一致性和参数默认值不被重复遮蔽。六个新动捕入口的隔离 ROS 回归通过（日志 `/tmp/abc_swarm_path_ros_wgpngoku`），安装中可执行名称唯一；实际工作区四个 relay 均已指向 nodes。原 48 个资源的 SHA256 与用户自有文件均未改变，未启动实车或 Gazebo。
