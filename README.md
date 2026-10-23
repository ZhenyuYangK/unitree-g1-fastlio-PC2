# Unitree G1 EDU+ 29DOF · PC2 建图、导航与讲解

面向 **G1 EDU+ 29DOF 的 PC2** 维护的原生 ROS 2 项目，包含 MID360、FAST-LIO、NDT 重定位、Nav2、速度安全桥、航点管理和多点任务。

- 本仓库：[unitree-g1-fastlio-PC2](https://github.com/ZhenyuYangK/unitree-g1-fastlio-PC2)。
- 功能对齐来源：[unitree-fastlio-ubuntu22.04](https://github.com/ZhenyuYangK/unitree-fastlio-ubuntu22.04)，本轮固定到 `afaf7bc5daefc98cfa045c0b50d05e17b7ac2053`（2026-10-20）。
- [进度核对与分阶段迁移](docs/PC2_MIGRATION_PLAN.md) · [代码结构与整理规则](docs/CODE_STRUCTURE.md)。

**P0 基线冻结与结构整理已完成，P1–P7 待实施。** 本地源码已有航点、多点任务与暂停巡航，不能按远端旧 README 的 v8.7 描述判断本地进度。本轮整理日期为 2026-10-23。P0 的快照、缺失文件说明与校验结果见 [P0 验收记录](docs/migration/P0_ACCEPTANCE.md)。

## 平台与验证范围

| 用途 | 平台 |
| --- | --- |
| 本次检查主机 | 用户提供的 Ubuntu 22.04 主机；用于源码核对和文档整理 |
| PC2 适配目标 | 按现有源码维护 Ubuntu 20.04 / ROS 2 Foxy / aarch64 / Python 3.8；上机时核实实际系统和依赖版本 |
| 功能参考版本 | Ubuntu 22.04 / ROS 2 Humble / x86_64 / Python 3.10 |

对齐的是功能和接口。保留 PC2 的 Foxy、DDS 和 aarch64 适配，不直接替换成电脑的 Humble 环境或预编译文件。本轮未运行编译、ROS 节点或真机控制，后续由操作者在 PC2 编译验收。

## 当前能力

| 模块 | 本地已有 | 待从 Ubuntu 版迁移 |
| --- | --- | --- |
| 建图 | MID360、FAST-LIO、PCD 保存和二维地图转换 | 统一环境、进程归属检查、保存结果确认 |
| 定位 | 人工初始位姿 + NDT；固定起点入口 | 时间同步、新鲜状态、无先验全局定位、多候选确认和诊断 |
| 导航 | Foxy Nav2、`/cmd_vel_nav`、Dry Run 与真机桥 | 定位失效联锁、机体系速度反馈、`/Odometry_nav`、统一速度参数 |
| 航点与任务 | 录点、任务校验、顺序导航、取消和恢复标记 | 单点任务结果持久化、双语讲解、人工放行、状态服务 |
| 巡航 | `g1_cruise.sh` 的多圈、暂停、继续和停止 | 接入新定位状态，保留 PC2 既有巡航能力 |
| 语音与网页 | 有厂商语音头文件；没有业务语音包和网页目录 | `g1_audio_bridge`、网页建图/导航/编辑、多设备操作权 |

当前 `start-auto` 仍是读取固定起点文件的旧入口，**不代表已实现无先验全局定位**。新命令语义将在迁移 P3 中处理。

## 代码目录

Git 仓库根目录就是包含 `src/`、`script/`、`maps/` 的工作空间。本次本地路径为 `g1_slam_ws_source/g1_slam_ws/`。

```text
.
├── README.md
├── docs/                         # 进度、迁移方案和结构说明
├── script/                       # 现有建图、导航、航点与巡航入口
├── src/
│   ├── FAST_LIO_ROS2/            # 建图与里程计
│   ├── Livox-SDK2/              # 雷达 SDK 源码
│   ├── livox_ros_driver2/       # ROS 2 驱动
│   ├── g1_relocalization/      # NDT 定位
│   ├── g1_nav2_bringup/        # Foxy Nav2 配置与启动
│   ├── g1_loco_bridge/         # 速度门控与 Unitree 后端
│   ├── g1_task_interfaces/     # 当前 ExecuteMission Action
│   ├── g1_task_manager/        # 航点与多点任务
│   ├── g1_unitree_ros2_probe/  # Unitree 诊断工具
│   └── unitree_ros2_interfaces/
├── maps/                         # 现场地图、转换工具
├── task_data/                    # 现场航点和任务；新增内容不进入 Git
├── third_party/                  # 厂商头文件、库及许可证
├── backup/                       # 本地备份；已归档未被引用的历史副本
└── build/ install/ log/ runtime/ # 本地生成内容，不进入 Git
```

`g1_audio_bridge/`、`web_console/` 和统一环境脚本属于后续迁移交付，当前尚不存在。结构规划和各模块职责见 [CODE_STRUCTURE.md](docs/CODE_STRUCTURE.md)。

## 获取与上机准备

```bash
git clone https://github.com/ZhenyuYangK/unitree-g1-fastlio-PC2.git
cd unitree-g1-fastlio-PC2
export G1_WS="$PWD"
```

本轮修改保存在本地，尚未提交或推送；上述 clone 获取的是远端当前版本。

P0 已保存本地源码、现场数据及历史归档快照，位置为 `backup/p0_20261023/`；基线与差异证据位于 `docs/migration/`。按项目设置，`docs/`、`doc/` 不跟踪，文档与核对证据只保存在本地，clone 不包含这些记录。在当前工作空间执行以下只读校验，不需要 ROS 环境：

```bash
python3 script/g1_verify_p0.py --local
```

省略 `--local` 仅校验源码和本地核对证据。后续阶段修改源码后，P0 校验报告差异属于预期；保留 P0 快照并为新阶段单独记录验收。

当前本地启动脚本和 CMake 仍有旧机器的 `/home/g1test/...` 路径。迁移 P1 会统一环境加载和路径配置；在 P1 完成前，换路径部署需要逐项核对。不要把当前检查主机已有的 Humble 环境当作 PC2 的运行环境。

上机前准备同一区域的三维地图、PGM 和地图 YAML，以及匹配 `map_id` 的航点与任务。默认文件为：

- `maps/map.pcd`、`maps/map_multilayer.pgm`、`maps/map_multilayer.yaml`；
- `task_data/maps/laboratory_map_v1/waypoints.yaml`；
- `task_data/missions/missions.yaml`。

地图、航点和任务需要单独备份、部署；`.gitignore` 不会自动取消远端已跟踪地图的跟踪。厂商库只使用对应架构版本，既有 `build/`、`install/` 含原机路径，不可直接作为新路径的部署结果。

## 现有运行入口

以下命令说明当前源码入口，在目标 PC2 环境、路径及依赖准备完成后使用。编译与分阶段验收要求见迁移方案。

```bash
# 建图：启动、保存、保存后结束
bash script/g1_mapping.sh start
bash script/g1_mapping.sh save
bash script/g1_mapping.sh finish

# 地图转换：先查看工具参数，按实际地图设置地面、边界和障碍高度
python3 maps/pcd2pgm_multilayer.py --help

# 人工定位与规划；在 RViz 中给定初始位置和朝向
bash script/g1_navigation.sh start
bash script/g1_navigation.sh continue

# Dry Run 导航桥
bash script/g1_navigation.sh start-dry
bash script/g1_navigation.sh continue-dry

# 航点和任务离线查看
bash script/g1_waypoints.sh list
bash script/g1_waypoints.sh missions

# PC2 已有巡航路线预览
bash script/g1_cruise.sh plan
```

巡航的多圈、暂停和停止说明保留在 [g1_cruise_README.md](script/g1_cruise_README.md)。其中原机器的安装路径和历史验证记录仅供追溯，不代表本轮已复验。

真机桥入口为 `script/g1_navigation_real.sh`，`start` 后仍需显式 `enable`。定位失效自动关门等新版能力尚待迁移；现有入口的就绪检查不能等同于新版实时定位联锁。

## 原项目图片

保留 PC2 远端原 README 的三张图片，作为历史运行展示；图片不代表本轮迁移验收。

建图运行：

<img width="2468" height="1322" alt="截图 2026-09-20 15-37-42" src="https://github.com/user-attachments/assets/7c641cc5-257c-4467-8006-543a2e9ebd2a" />

大范围点云地图：

<img width="1320" height="910" alt="截图 2026-09-23 20-26-41" src="https://github.com/user-attachments/assets/b920b017-1370-4dbe-a483-06c6b594d13e" />

导航运行：

<img width="1562" height="857" alt="截图 2026-09-20 15-25-19" src="https://github.com/user-attachments/assets/4dba04f2-d4e9-4c2f-b56a-b40f8c8f0dfa" />

## 迁移顺序

P0 基线与结构整理 → P1 PC2 环境和路径 → P2 新鲜定位状态与联锁 → P3 全局自动定位 → P4 速度反馈与 Nav2 → P5 语音和任务 → P6 网页控制 → P7 PC2 整体验收。

每阶段的源码范围、依赖、Foxy 适配点、验收和回退条件均在 [PC2_MIGRATION_PLAN.md](docs/PC2_MIGRATION_PLAN.md)。P0 之外均为待迁移，不能把参考仓库通过的检查记为 PC2 已通过。

## 许可证

第三方组件遵循各自目录中的许可证。各业务包的 `package.xml` 也有各自声明；发布和分发时逐项核对，根目录目前未补充统一许可证。
