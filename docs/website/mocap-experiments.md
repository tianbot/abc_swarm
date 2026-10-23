# Mini 三车动捕集群实验

> Tianbot Docs 页面草稿，尚未发布。建议放在原 swarm 课程旁，保留原课程 URL。本页用于新增设备实验，不替代原视频教程。

本实验使用三台 Tianbot Mini 和动捕系统，观察三角形编队、逐车跟随以及预设路径下的集群运动。NOKOV 是动捕合作伙伴，ROS 当前通过 VRPN 获取位姿。没有动捕的原课程继续使用已有激光地图等实验，不要求重新录制视频。

需要：三台 Mini、运行原 Noetic 工作区的电脑、能通过 VRPN 持续输出三刚体位姿的动捕环境。默认刚体与车名为 tianbot_mini_01、02、03，按现场名称可分别设置；坐标轴 x 向前、y 向左，原点对应底盘控制参考点。

服务器地址和端口在包内 `config/mocap.yaml` 配置。首次更新后编译并加载工作区。默认一个 launch 启动驱动、定位、算法和交互；运动始终先禁用，已有驱动/VRPN 时显式传 `start_driver:=false start_vrpn:=false`。

## 从手动领航开始

三角形摆位：02 在 01 后方 0.5 m、左侧 0.5 m，03 在后方 0.5 m、右侧 0.5 m，各车同向。

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch
```

观察统一状态，确认持续有效定位、实际前车里程计和摆位后开始：

```bash
rostopic echo /abc_swarm_demo/status
```

```bash
rosservice call /abc_swarm_demo/set_enabled "data: true"
```

默认键盘终端用于头车低速领航。i 前进、u/o 缓弯、k 停止；输入有超时，旧键盘版本需要持续按键。停止整个实验：

```bash
rosservice call /abc_swarm_demo/set_enabled "data: false"
```

要逐车跟随，停止并退出旧实验，摆成同向纵列、相邻间距 0.5 m，启动：

```bash
roslaunch abc_swarm demo_mocap_three_mini_leader_follower_snake.launch
```

两者分别是 01 同时领航两车，以及 01→02→03。逐车转弯可能切弯，不等于复走同一路径。

## 再测试预设路径

先测单车，检查空间、方向、路径预览与停车：

```bash
roslaunch abc_swarm demo_mocap_path_tracking.launch shape:=circle
```

三车三角形走圆形：

```bash
roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch shape:=circle
```

逐车跟随走圆角矩形：

```bash
roslaunch abc_swarm demo_mocap_snake_leader_follower_path_tracking.launch shape:=rounded_rectangle
```

路径根据头车准备时的位姿生成，预览后固定。默认圆半径 0.8 m，圆角矩形外轮廓宽 2.5 m、高 2.0 m、圆角半径 0.8 m，头车速度 0.05 m/s、一圈。后车轨迹和车体会超出头车参考线，须按整个编队的范围准备场地；参数可以通过 launch 覆盖。

用同一 set_enabled 服务开始、停止。新一轮先停止，再重置，等新的 ready 和预览：

```bash
rosservice call /abc_swarm_demo/reset
```

## 观察与验收

比较直行和转弯时的队形、内外侧车辆运动、路径完成和实际停车。实验没有避障，不保证车体参考点误差和朝向同时为零。就绪只表示输入数据可用，不能代替电机/电源/急停检查。

代码分支的构建和合成反馈检查不等于现场验收。正式发布本页面前应补充三车设备实测记录、场地条件、效果图和已知限制；本轮没有自动启动真实车辆或发布网站。
