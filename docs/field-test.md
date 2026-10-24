# 三车动捕现场测试清单

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
