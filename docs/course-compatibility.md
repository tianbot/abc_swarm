# 如何保证旧 main 和视频课程不被改坏

已经对比。冻结版本为 `1708f332ae900384b9e84889b7ab3979aaae9f5f`，即 2023-02-15 的原 main。之后 main 即使合入新功能，也不自动更换这个课程基线。

## 三件事分别解决什么

- **兼容检查**：核对旧课程的文件、内容、位置、执行权限和启动名称。
- **单元/集成测试**：检查算法、参数传递、构建安装及合成 ROS 反馈下的行为。
- **CI 和分支保护**：GitHub 自动运行这些检查，再要求检查通过才能合并。

检查通过不等于真实车辆已经验收。当前规则主要防止开发或整理目录时意外破坏原课程。

## 当前冻结范围

原 main 的 48 个 nodes、launch、config、RViz、RST/图片等资源保持原内容。原九个教学脚本路径、名称和权限保持。新增算法与实验允许增加文件，不把当前 13 个节点或九个动捕入口的数量永久写死。

只有两个已经由用户批准的改名：

| 原位置 | 当前位置 |
| --- | --- |
| `launch/demo/demo_mc_tf_tt_circle_mini.launch` | `launch/mocap/demo_mocap_tf_tt_circle_mini.launch` |
| `launch/demo/demo_mc_tf_tt_follow_mini.launch` | `launch/mocap/demo_mocap_tf_tt_follow_mini.launch` |

内容仍须与原版一致。新目录不能放一个与旧课程同名的 launch，避免 ROS 按文件名启动时歧义。

CMake、包依赖、根 README 等可以为新增功能作必要调整；Noetic 构建、测试和安装检查负责验证这些改动。原资源内容暂时采用严格冻结，若要修复旧 bug，应按下文单独审核，不把“完全不改旧文件”当作永久禁止修复。

## 本地最简单的操作

在完整 Git 源码仓库根目录运行，不需要 ROS、动捕或车辆：

```bash
python3 tests/check_course_compatibility.py
```

成功会显示 PASS、冻结提交和资源数量。失败会指出文件，比如 Original content changed、Missing original resource、Ambiguous original roslaunch name。浅克隆缺少原始提交也会失败，不会跳过。此时先取得完整历史；不要自动生成新基线。

检查器直接从冻结 Git 提交读取原文件，再核对 JSON 清单与当前文件。只改清单 SHA256、删掉清单条目或把清单指向当前版本，都无法使检查通过。修改检查器本身仍然需要代码审核，CI 无法代替维护者审查自己的规则。

开发时需要完整 Noetic 检查：

```bash
source /opt/ros/noetic/setup.bash
cd ~/tianbot_mini_ws
catkin_make --only-pkg-with-deps abc_swarm -DPYTHON_EXECUTABLE=/usr/bin/python3
catkin_make run_tests_abc_swarm
catkin_test_results build/test_results/abc_swarm
```

当前机器如遇既有 setuptools/distutils 环境冲突，按原构建记录给 catkin 命令加 `SETUPTOOLS_USE_DISTUTILS=stdlib`；不修改系统 Python 环境来绕过。

## GitHub 会自动做什么

工作流位于 `.github/workflows/course-compatibility.yml`。推送或创建/更新 PR 后运行：

1. **Course compatibility**：冻结 Git 比较；检查器自身的破坏性回归测试；Python 语法、改动空白及无硬件的控制计算测试。普通 Ubuntu 执行，不安装 ROS。
2. **Noetic build and tests**：使用 Noetic/Focal 容器构建、测试、安装，检查安装后的可执行入口；最后运行 localhost:11433 上的合成 ROS 回归，真实驱动、VRPN、键盘和 RViz 显式关闭。

没有 Gazebo 仿真、真实车辆、飞行器或现场网络连接。工作流采用 GitHub 托管 runner，不把机器人电脑设为 PR 的执行环境。

本地已经运行相同的检查器和 Noetic 代码检查步骤。工作流尚未推送，不能写成“GitHub CI 已运行通过”；第一次远端运行还需要确认容器/软件源获取正常。没有 secrets、写权限、自动提交或部署步骤。

Checkout 使用完整历史，依据 [官方 checkout 参数说明](https://github.com/actions/checkout)。工作流定义方式见 [GitHub Actions 官方文档](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax)。

## 你需要做什么

现在不必安装 CI 服务，也不必逐文件审查 Python 代码。

1. 下次推送该分支后，创建指向 main 的 PR，确认上述两项检查在 GitHub 实际变绿。
2. 由仓库管理员在 main 的分支保护或规则集中，启用“必须通过 PR 合并”和“必须通过状态检查”，选择 **Course compatibility**、**Noetic build and tests**。需要保障管理员操作时，也将规则应用于管理员/禁止绕过。不要使用不同分支的同名成功检查作为替代。
3. 以后合并前，检查兼容检查通过、PR 变更说明合理。仓库的 PR 模板会提醒开发者填写兼容影响和验证结果。涉及旧课程、兼容清单、检查器或工作流时，由你确认是否有意改变规则。

设置入口和权限以 [GitHub 分支保护官方说明](https://docs.github.com/en/repositories/configuring-branches-and-merges-in-your-repository/managing-protected-branches/about-protected-branches) 为准。只有工作流文件，没有分支保护，并不能阻止直接合并。

## 什么时候需要你特别确认

若必须修改旧文件，开发者先停止扩大改动，单独说明：旧命令、修改前后行为、是否影响视频中的编辑操作、回归证据和回退方式。由你确认后，再在独立 PR 中审查明确例外及行为测试。

不能为了变绿直接改 `base_commit`、重算旧文件哈希、删除清单条目、删掉 CI 步骤或设置 continue-on-error。后续新增节点和 demo 不需要改变旧课程基线。

## 本轮本地验证记录

- 冻结提交直接比较通过：48 个原资源内容不变，两处授权改名不改变内容。
- 检查器的七种临时副本回归通过，包括“改代码并一起重写清单”仍被拒绝。
- Noetic 构建及 28 项测试通过；安装后的脚本正文、原课程资源和文件名查找检查通过。
- 安装包上的六个新动捕 demo 合成 ROS 回归通过，日志 `/tmp/abc_swarm_path_ros_6kkh560v`；没有连接实车。
- 工作流 YAML、全部 shell 片段和文档链接已作本地检查。GitHub Actions 尚未远端运行，main 分支保护尚未配置。
