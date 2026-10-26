# 三车动捕现场测试清单

## 2026-10-26 完整入队与跟随流程（当前版本）

三角形/蛇形leader–follower默认先ORCA入队，捕获区域与车体包络分离后转纯点镇定，全体位置、航向与实测速度稳定，再进入manual/auto跟随。auto完成路径后重新稳定队形并停车。两个旧launch名称保留，startup=false可对照上一节记录的直接跟随方式。四项操作见[现场使用](mocap-field-use.md)，阶段/参数/GUI接口见[完整流程](formation-workflow.md)。下方早期开发记录是历史快照，当前默认流程以本节为准。

功能提交 `44e6216`。本轮99项catkin测试全部通过，Noetic独立构建、测试、安装完成。安装后四种完整任务组合在私有master 11438及合成时钟下通过：三角manual/auto、蛇形manual/auto，覆盖ORCA→精定位→ready→following，auto额外覆盖settling/completed；检查提前遥控丢弃、超时、TF中断停车及恢复、不回放旧请求、冻结目标、每车唯一输出者、完成与退出零输出。仅发布带已知标记偏移/02航向偏差的合成TF，不发布odom，不启动硬件/VRPN/UI。最终日志 `/tmp/abc_swarm_admission_installed_mission_final.log`，详细目录 `/tmp/abc_swarm_mission_ros_h5u6mvta`。记录中的最小间距来自模型采样，不是实车结果。

安装后旧路径/直接跟随回归、独立点镇定、ORCA及五个协调路径用例通过。日志分别为 `/tmp/abc_swarm_admission_installed_legacy.log`、`/tmp/abc_swarm_admission_installed_point_retry.log`、`/tmp/abc_swarm_admission_installed_orca.log`、`/tmp/abc_swarm_admission_installed_loops.log`。点镇定测试修正了固定3秒启动等待导致的空状态读取，改为有上限的状态到齐等待，控制验收条件未放宽。构建/测试/安装日志为 `/tmp/abc_swarm_admission_build.log`、`/tmp/abc_swarm_admission_tests_final.log`、`/tmp/abc_swarm_admission_install_final.log`。

AI能力表/生成器及正反例同步更新：四个跟随能力默认完整入队，支持显式admission配置或enabled=false对照旧方式；旧配置重新生成时也会采用新默认，须告知自动入队行为。九种能力的交付生成、只读--check与安装后的Noetic解析均通过，并核对两种队形×两种模式的现场标定展开。CI新增核心流程及安装后完整任务回归，尚未运行远端CI。课程检查器、基线和例外未改，原48个资源兼容检查通过。

本轮未启动实车、Gazebo，未推送、未发布网站；完整入队流程仍待现场验收。内部跟随图有五角色回归，但公开launch/AI白名单仍为三台Mini，没有据此扩展客户交付数量或承诺全程避障。

## 2026-10-26 跟随入口的 manual/auto 模式（早期记录）

两个 leader-follower demo 新增 mode=manual/auto，默认manual，拼写错误拒绝。manual启动键盘；auto复用preset_path_tracker作为唯一头车速度发布者，半径默认1m、一圈，后车跟随关系保持三角或逐车蛇形。共用头车片段为 `launch/common/leader_path_tracker.launch.xml`，原自动路径入口也复用它；没有新增Python节点。模式在启动时选择，退出后重启切换。

85项catkin测试、构建安装与安装后四种组合解析通过；AI生成器为manual实验显式传mode=manual，自动实验继续使用原path_triangle/path_snake类型，生成/只读检查/Noetic解析通过。原课程48资源保持兼容，未修改检查器/基线/例外。

私有ROS回归 `/tmp/abc_swarm_path_ros_fyqi_zqp` 覆盖原六入口与新增两种mode=auto：合成TF/odom、启动/编队等待、头车单一发布者、路径完成/重置、反馈中断停车以及manual恢复。日志 `/tmp/abc_swarm_modes_private_ros.log`；构建/测试/安装日志为 `/tmp/abc_swarm_modes_build.log`、`/tmp/abc_swarm_modes_tests.log`、`/tmp/abc_swarm_modes_install.log`。不接入现场master，不启动硬件或UI，没有追加实车/Gazebo测试或远端CI。

## 2026-10-26 收口

本轮不再启动实车。正式使用入口及配置见[现场用法](mocap-field-use.md)，已去掉运行时对 `/tmp` 脚本/标定JSON/临时控制TF的依赖：

- `config/mocap_field_control.yaml`：当日标记架的显式校准、8cm最终位置容差；不默认加载。节点内部换算控制中心，`control_poses`供查看，原始TF保持原样。
- `velocity_source=mocap`：从有效控制位姿约0.2秒窗口测量速度，支持无底盘odom的入位/ORCA与协调；默认odom模式保持兼容。反馈中断后重新积累速度窗口再运行。
- 入位增加转向阶段滞回，最终完成仍检查原位置/航向容差；共用算法没有按设备复制，未新增可执行节点。
- AI契约2新增可选mocap_control，能力表/生成器/正反例/Noetic展开/安装示例同步更新；ORCA包装入口的组合哈希也覆盖其点镇定include。CI新增安装后示例生成检查，旧课程基线和检查器未改。

84项本地/catkin测试、Noetic构建测试安装通过；安装后两轨迹入口的现场YAML和新AI示例生成/只读检查/解析通过。私有ROS：点镇定 `/tmp/abc_swarm_point_ros_dzklb7ye`；ORCA `/tmp/abc_swarm_orca_ros_j2k15w5d`；五个协调用例 `/tmp/abc_swarm_loops_ros_9qji45qi`，包含完全无odom且带已知标记偏移/航向偏差的三角形模型。合成回归不接入现场master、不启动底盘/真实VRPN/UI。

本机验证入口日志：`/tmp/abc_swarm_finalize_point_ros.log`、`/tmp/abc_swarm_finalize_orca_ros.log`、`/tmp/abc_swarm_finalize_loops_ros.log`；构建/测试/安装记录为 `/tmp/abc_swarm_finalize_build.log`、`/tmp/abc_swarm_finalize_catkin_tests_final.log`、`/tmp/abc_swarm_finalize_install_final.log`。未运行远端CI、未推送、未发布网站。

现场证据分开记录：1.5m圆自动录包有completed；三角用户反馈已运行，最后一份自动记录为fault且edge_rounds_completed=0，未证明完整三边。正式合并配置之后的实车、2m及顺时针仍待后续验收，不在本轮追加。

## 2026-10-25 现场准备与 ORCA 首轮记录

- 三车切入多机模式后，UDP echo 与 TCP 连接确认：01=`192.168.2.104`、02=`192.168.2.100`、03=`192.168.2.103`，连接本机 `.106:11411`。这些是当天 DHCP 地址，不是固定设备配置；同一 IP 先后可能对应不同 MAC，不能沿用旧扫描结论。
- 本机 shell 残留 `ROS_IP=192.168.2.102`，首轮 launch 自检失败，未启动控制。测试进程临时 unset ROS_HOSTNAME 并设置 ROS_IP=.106 后可启动；未修改系统配置。
- 启动前10秒静止基线：三车 odom 最大接收间隔约0.12秒、消息最大时间戳延迟约0.063秒，TF 延迟约0.006秒，无其他 cmd_vel 发布者。
- 复用后台执行默认 ORCA 三角点镇定（spacing=0.5，max_linear=0.15），02/03 初始目标误差约0.228/0.496m。运行不足一秒，03 odom 中断，节点转 waiting；独立观察进程结束该轮。最终三车 cmd_vel 和 odom 速度均为零。02 有小幅转动，03 未观察到非零 odom 速度；未完成点镇定，不能据此评估近目标误差或宣称 ORCA 实车通过。
- 后续30秒不发运动指令，02/03 分别出现约0.936/0.684秒的接收间隔；01 最大约0.120秒。TCP 客户端端口相较初次连接均改变，说明期间重新建立过连接，具体重连时间和触发原因尚未定位。底盘驱动日志文件未找到，尚无对应 reset 告警记录，不能将各接收间隔逐一等同于 TCP reset。
- 录包与统计：`/tmp/abc_swarm_field_20261025_153257/`（run.bag、summary.json、stationary.bag、stationary_summary.json）；首次地址错误记录在 `/tmp/abc_swarm_field_20261025_153231/`。这些是本机临时文件，需另行归档才能长期保留。
- 首轮结束已退出算法，保留原驱动/VRPN/RViz。后续根据用户对延迟容忍的现场要求，按下一节记录的参数复测；断流原因尚未定位。

### 现场授权延迟容忍复测

用户明确要求容忍较大延迟后，使用1.5秒 odom 窗口复测，TF/几何/故障停车逻辑保留。本轮运行约20秒，未因 odom 中断结束；03从约0.496m误差收敛至约0.027m，最终航向误差约0.060rad。02误差增大，最终有效包络重叠故障，整组停机；最后三车指令与实测速度均为零。

离线比较正向 odom 速度与动捕位置微分：03移动方向与动捕 yaw 平均相差约0.7°，02约110.7°，稳定移动区间约106～111°。因此02存在显著朝向/参考系不一致的证据，尚需现场核对刚体坐标，不把先前目测的“可能180°”当作结论。修正前未再次运行；没有用修改 ORCA 包络或 PID 来补偿这一问题。

数据：`/tmp/abc_swarm_field_20261025_154131/` 下 run.bag、summary.json、direction_check.json。延迟容忍参数保存为 `config/orca_delayed_feedback.yaml`，默认基础配置不变；该配置本轮只验证了连续执行和03到位，三车任务/避让/圆形与三角形轨迹仍待验收。

本轮代码在 `teaching/main-reorg` 分支，从原 main 重新整理。原九个教学脚本未改，新增节点合在 nodes，动捕实验集中到 launch/mocap。下面是现场复验步骤，不是已经完成的实车结果。

原准备阶段已完成本地检查、安装检查及合成 ROS 回归；2026-10-24 调整为反馈有效后自动运行，现场效果待重新验收。待完成：本分支真实三车测试。GitHub CI 工作流已写好，但未推送运行，main 分支保护也未设置。原来另一分支测试过三角/逐车跟随，不能直接当作当前分支的验收。

## 1. 开始前

在仓库目录核对当前分支和提交，测试记录中记下输出：

```bash
git branch --show-current
```

```bash
git rev-parse --short HEAD
```

分支应为 `teaching/main-reorg`。两份用户自有导弹实验文件仍未跟踪，不属于本次测试内容，不需要清理。

可先运行不需要 ROS 的旧课程检查：

```bash
python3 tests/check_course_compatibility.py
```

应看到 PASS，基线 1708f33、48 个原资源和两处授权改名。

加载环境并编译本包一次：

```bash
source /opt/ros/noetic/setup.bash
cd ~/tianbot_mini_ws
SETUPTOOLS_USE_DISTUTILS=stdlib catkin_make --only-pkg-with-deps abc_swarm -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

下面按驱动、VRPN、算法都运行在这台电脑上、使用本机 ROS master 的情况说明。每个新终端加载环境并使用同一个 master：

```bash
source /opt/ros/noetic/setup.bash
source ~/tianbot_mini_ws/devel/setup.bash
export ROS_MASTER_URI=http://localhost:11311
```

如沿用另一台电脑上的 master，则所有终端使用那个已有地址。ROS_IP 如需指定，应使用当天本机实际网卡地址，不照抄过去的地址。Mini 的设备网络保持原先已跑通的配置，本清单不执行会广播修改设备设置的扫描脚本。

## 2. 确认本次设备配置

- Mini 必须先进入多机/集群模式。2026-10-25 现场确认的操作：按住配网（WIFI）键，点击 Reset，随后松开配网键，状态灯变蓝。原 Mini 教程另有“关机时按住 WIFI 键开机”的操作；按当前硬件实际方式进入。蓝灯只是模式提示，还需确认三车已连上驱动并持续发布 odom。
- 电脑与三台车连接同一个局域网，rmip 指向运行底盘驱动的电脑实际 IPv4，rosserial TCP 端口默认 11411。动捕服务器 IP 与底盘 rmip 是两个不同地址。只看到动捕刚体不代表底盘已联网；无 odom 时先检查工作模式、网络及 TCP 连接。
- 三台 Mini 的 ROS 名称及动捕刚体：`tianbot_mini_01`、`tianbot_mini_02`、`tianbot_mini_03`。
- 场地配置：`config/mocap.yaml`，当前 server 为 `192.168.2.101`，port 为 `3883`；当天不同则改此文件或传 `vrpn_server_ip`、`vrpn_port`。
- 动捕在 world 下持续发布刚体；刚体 x 向车头、y 向左、原点对应底盘控制参考点。
- LF 需要前车实际 odom 速度和有效时间戳，三车都应连接。

**新默认行为：一个 demo 会启动底盘驱动和 VRPN。**先退出旧实验。若后台已在运行，给所选命令追加 `start_driver:=false start_vrpn:=false`，不要重复启动；不再要求先手工启动一套后台。

手动实验默认打开键盘终端；自动路径默认打开 RViz。反馈有效后自动运行，没有手动启用步骤；启动前准备好摆位和完整活动空间。

## 3. 先复验手动领航

三角形：三车同向，02 在 01 后方 0.5 m、左侧 0.5 m，03 在后方 0.5 m、右侧 0.5 m。

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch
```

在另一个已设置环境的终端观察：

```bash
rostopic echo /abc_swarm_demo/status
```

正常时 state 为 running、ready 为 true。若 waiting，看 reason 指出的 frame 或 odom。头车未动时，后车也可能开始向编队位置收敛。

键盘 i 前进、u/o 缓弯、k 停止头车请求。初始请求 0.15 m/s、0.15 rad/s；输入超时 0.3 秒停止头车请求。不要另开直接向头车 cmd_vel 发速度的遥控节点。停止整套实验时在 roslaunch 终端按 Ctrl+C。

退出旧 demo 并确认三车实际静止。再摆成同向纵列、相邻间距 0.5 m，启动逐车跟随：

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_snake.launch
```

同样启动即自动跟随，退出 launch 停止。确认 02 跟 01，03 跟 02；转弯可能切弯，不要求沿相同历史路径。

## 4. 再测自动路径，先单车

停止并退出手动实验。先用 01 验证路径方向、跟踪与结束停车：

```bash
roslaunch abc_swarm demo_mocap_path_tracking.launch shape:=circle
```

首次有效位姿确定起点和起始切线，路径固定在 world。默认逆时针、圆在初始车头左侧，半径 0.8 m、速度 0.15 m/s、一圈。启动前确认整条轨迹及编队范围足够空旷；反馈就绪后自动跟踪，RViz 用于观察运行路径。

完成应显示 completed 并停止输出。再次测试可重启 launch，或在 completed/fault 后重置：

```bash
rosservice call /abc_swarm_demo/reset
```

重置接受后按实时头车位置生成新路径并自动运行，没有再次启用步骤。正在运行时不能重置。

圆角矩形可在退出旧 demo 后测试：

```bash
roslaunch abc_swarm demo_mocap_path_tracking.launch shape:=rounded_rectangle
```

默认外轮廓宽 2.5 m、高 2.0 m、圆角半径 0.8 m。宽高须至少为圆角半径两倍；根据实际场地调整，不能只看头车路径而忽略车体和后车范围。

## 5. 单车通过后接三车

重新按三角形摆位，测试自动领航：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch shape:=circle
```

逐车同向纵列，测试圆角矩形：

```bash
roslaunch abc_swarm demo_mocap_snake_leader_follower_path_tracking.launch shape:=rounded_rectangle
```

同样启动前准备好场地，反馈有效后自动开始。结束时头车先停止，后车继续收敛；满足前置控制点容差或等待 30 秒超时后共同禁用。超时会明确显示 formation settling timed out，需要记录，不能当成正常收敛。

PID 对照为可选项，退出旧实验后使用：

```bash
roslaunch abc_swarm demo_mocap_three_mini_pid_tracking.launch
```

没有运行中切换队形。每次换实验先停止、确认静止、退出旧 launch，再摆车和启动下一个。

## 6. 出现问题时记下这些信息

记录当前 launch 命令及参数、测试形状、哪台车异常、状态 reason、实际轨迹/停车表现。以下命令只读，可分别查看：

```bash
rostopic echo -n 1 /abc_swarm_demo/status
```

```bash
rosnode list
```

```bash
rostopic info /tianbot_mini_01/cmd_vel
```

```bash
rostopic echo /tianbot_mini_02_leader_follower/status
```

```bash
rostopic echo /tianbot_mini_03_leader_follower/status
```

若仍有其他遥控或旧跟踪器发布同一 cmd_vel，先停止并排除重复控制。算法没有避障，实际断网和动捕冻结姿态表现仍需单独现场确认。

## 现场记录（测试后填写）

日期：　　提交：　　场地/配置变更：

| 项目 | 结果与问题 |
| --- | --- |
| 三车数据就绪、摆位 | 未测 |
| 手动三角形 | 已完成（2026-10-24 用户报告，简单跟随） |
| 手动逐车跟随 | 已完成（2026-10-24 用户报告，简单跟随） |
| 单车圆形、正常结束 | 未测 |
| 单车圆角矩形 | 未测 |
| 自动三角形 | 未测 |
| 自动逐车圆角矩形 | 未测 |
| 统一停止、重置、异常原因 | 未测 |
| PID 对照（可选） | 未测 |

更多参数与接口见 [动捕实验说明](mocap-demos.md)、[统一运行接口](demo-runtime.md)。CI 设置见 [课程兼容保障](course-compatibility.md)，不需要为现场试车先配置 GitHub。

2026-10-24 启动流程修正验证：38 项本地测试及六入口私有 ROS 合成回归通过；已移除手动启用步骤，并修正 rosserial odom 小幅未来时间戳误判。当前现场旧进程未重启，不能把这些离线结果记为新的实车验收。


2026-10-24 用户反馈：简单三角形和蛇形跟随已完成实测（用户报告）。未据此确认自动圆形/圆角矩形、点镇定或避障。下一项请使用独立的[三车点镇定测试](point-stabilization.md)，不在现有跟随实验运行中切换。
