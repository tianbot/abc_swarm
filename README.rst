ABC Swarm
=======================================

ABC Swarm is the developer manual for ROS2GO, Tianbot Mini and RoboMaster TT ROS Swarm Kit.

原课程与新增动捕实验
--------------------

本分支从 ``origin/main`` 重新整理，保留原课程的文件布局和命令。
``nodes/`` 保留原九个教学脚本，并集中放置四个新增可执行节点，
不另设 scripts 入口目录。原 ``launch/includes/`` 不变，配置文件保持一层。

* 原课程：继续按视频操作，原有无动捕实车和 Gazebo demo 不新增、不搬迁。
* 新增动捕实验：进入 ``launch/mocap/``，文件统一使用 ``demo_mocap_*`` 命名。
* 公共启动片段：新增部分放在 ``launch/common/``，与原 includes 分开。

加载已有 Noetic 工作区，更新后编译一次：

.. code-block:: bash

   source /opt/ros/noetic/setup.bash
   cd ~/tianbot_mini_ws
   catkin_make --only-pkg-with-deps abc_swarm -DPYTHON_EXECUTABLE=/usr/bin/python3
   source devel/setup.bash

三车三角形手动领航：

.. code-block:: bash

   roslaunch abc_swarm demo_mocap_three_mini_leader_follower_triangle.launch

三车逐车跟随：

.. code-block:: bash

   roslaunch abc_swarm demo_mocap_three_mini_leader_follower_snake.launch

头车自动走圆形路径，后车保持三角形：

.. code-block:: bash

   roslaunch abc_swarm demo_mocap_formation_leader_follower_path_tracking.launch shape:=circle

一个文件组合驱动、VRPN、算法和所需交互，运动默认禁用。
已有后台时传入 ``start_driver:=false start_vrpn:=false``。
地址和端口只需在 ``config/mocap.yaml`` 配置，也可通过启动参数覆盖。
准备好后使用统一服务开始、停止，具体摆位和路径范围先看文档。

* `现场测试清单 <docs/field-test.md>`_
* `动捕实验说明 <docs/mocap-demos.md>`_
* `统一启动和 GUI 接口 <docs/demo-runtime.md>`_
* `本轮取舍和验证 <docs/main-reorg.md>`_
* `文档与网站迁移建议 <docs/README.md>`_

本轮不启动 Gazebo 或真实设备。新分支的运行效果需要现场复验，
检查通过不等于已完成实车验收。

开发指导方针
------------

Python 实现可复用核心算法，关键参数可配置；launch 负责驱动、定位、
设备话题/frame 映射、角色、参数和实验组合。相同算法不按 Mini、RMTT、
TOM06S 分别复制。运动模型或控制输出不同的设备，使用必要的适配或对应控制律。
项目开发约定记录在 `AGENTS.md <AGENTS.md>`_。

旧课程兼容检查
--------------

冻结对比原 main 的 ``1708f33``，不会随未来 main 自动重设基线。
在源码仓库根目录即可检查，不需要启动 ROS 或设备：

.. code-block:: bash

   python3 tests/check_course_compatibility.py

GitHub 工作流会运行兼容检查、算法回归以及 Noetic 构建和合成 ROS 测试。
推送后还需要在 main 设置必过检查，才会阻止未通过检查的合并。
操作步骤见 `兼容保障与 CI <docs/course-compatibility.md>`_。
