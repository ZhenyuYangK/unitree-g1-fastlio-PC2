# Unitree G1 EDU+ 29DOF · PC2 建图、导航与讲解

本仓库是 **Unitree G1 EDU+ 29DOF 建图、导航与讲解项目的 PC2 平台版本**，包含 MID360、FAST-LIO、NDT 重定位、Nav2、速度安全桥、航点管理和多点任务。本仓库与 `unitree-fastlio-ubuntu22.04` 属于同一项目，分别面向 PC2 和 Ubuntu 22.04 x86_64 运行平台维护，按各自的系统、ROS 2 版本、架构及依赖进行适配。

- PC2 版本（本仓库）：[unitree-g1-fastlio-PC2](https://github.com/ZhenyuYangK/unitree-g1-fastlio-PC2)，面向 Ubuntu 20.04 / ROS 2 Foxy / aarch64。
- 本项目的 Ubuntu 22.04 x86_64 版本：[unitree-fastlio-ubuntu22.04](https://github.com/ZhenyuYangK/unitree-fastlio-ubuntu22.04)，使用 ROS 2 Humble；在该平台运行时请使用此版本。



## 平台与验证范围

| 用途 | 平台 |
| --- | --- |
| 本次检查主机 | Ubuntu 22.04 / Humble / x86_64 / Python 3.10；用于源码迁移、本机编译及隔离合成测试 |
| PC2 适配目标 | 按现有源码维护 Ubuntu 20.04 / ROS 2 Foxy / aarch64 / Python 3.8；上机时核实实际系统和依赖版本 |
| Ubuntu 22.04 x86_64 版本 | Ubuntu 22.04 / ROS 2 Humble / x86_64 / Python 3.10 |

两个平台版本按同一项目的功能和接口进行同步，PC2 版本保留 Foxy、DDS 和 aarch64 适配。P1 离线检查、P2/P3 定位与全局匹配测试、P4 FAST-LIO/速度桥/导航转换器及 P5 接口/语音桥编译和隔离测试已通过；P6 已在隔离 Python 3.8.20 实际运行测试，并核验 aarch64 安装包；其他阶段的 Python 3.8 语法检查通过。

## 当前能力

| 模块 | PC2 版本当前实现 | 同步与验证 |
| --- | --- | --- |
| 建图 | MID360、FAST-LIO、PCD 保存和二维地图转换；P1 已统一环境、进程归属和保存确认 | PC2 上机检查；后续整体验收 |
| 定位 | 无先验全局 XY/yaw 搜索、多候选三帧验证、有限重试和诊断；保留人工/固定初值与实时失效联锁 | PC2 多位置/朝向实测及性能验收 |
| 导航 | 机体系速度与协方差、`/Odometry_nav`、Foxy Nav2、统一低速/目标档、实际后端和运行参数核验；定位失效关门 | PC2 速度符号、低速导航及性能实测 |
| 航点与任务 | 新鲜定位录点、严格任务校验、双语讲解、末站人工放行、状态服务、单点/Action 结果持久化、取消和恢复标记 | PC2 讲解及任务现场检查 |
| 巡航 | 多圈/循环、途中暂停和恢复、剩余停留；业务包实现，共用任务锁/恢复标记及定位状态检查 | 已接入网页；PC2 现场回归待执行 |
| 语音 | 可取消的 Speak Action、真实/模拟模式、默认保留音量、进程归属管理 | PC2 发声与 PlayStop 效果确认 |
| 网页 | 本地原生控制台：地图/任务编辑、建图、定位/导航、讲解任务、连续巡航及手机操作权恢复 | PC2 双浏览器、负载与真实控制验收 |

`start-auto` / `start-auto-dry` 现为无先验全局定位；旧固定坐标功能使用 `start-fixed` / `start-fixed-dry`，仍读取 `script/auto_start_pose.env`。手动入口 `start` / `start-dry` 保留。

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
│   ├── g1_relocalization/      # 全局匹配、候选验证与 NDT 跟踪
│   ├── g1_nav2_bringup/        # Foxy Nav2 配置与启动
│   ├── g1_loco_bridge/         # 速度门控与 Unitree 后端
│   ├── g1_task_interfaces/     # ExecuteMission/Speak 与状态/放行服务
│   ├── g1_audio_bridge/        # 可取消语音 Action 与 Unitree AudioClient
│   ├── g1_task_manager/        # 航点、任务、双语讲解与巡航
│   ├── g1_unitree_ros2_probe/  # Unitree 诊断工具
│   └── unitree_ros2_interfaces/
├── web_console/                  # P6 网页、PC2 依赖锁、API/浏览器测试
├── maps/                         # 现场地图、转换工具
├── task_data/                    # 现场航点和任务；新增内容不进入 Git
├── third_party/                  # 厂商头文件、库及许可证
├── backup/                       # 本地备份；已归档未被引用的历史副本
└── build/ install/ log/ runtime/ # 本地生成内容，不进入 Git
```

P1 已增加 `g1_env.sh`、`local.env.example`、构建/自检、雷达配置、进程归属和地图保存辅助工具；P5 已增加 `g1_audio_bridge/`、任务放行/状态接口，并将巡航 Python 移入业务包；P6 已增加根目录 `web_console/` 和 PC2 运行入口。结构规划见 [CODE_STRUCTURE.md](docs/CODE_STRUCTURE.md)。

## 获取与上机准备

```bash
git clone https://github.com/ZhenyuYangK/unitree-g1-fastlio-PC2.git
cd unitree-g1-fastlio-PC2
export G1_WS="$PWD"
```

仓库按 P0–P6 分阶段维护；上述 clone 获取远端 main，具体更新以提交记录为准。

P0 已保存本地源码、现场数据及历史归档快照，位置为 `backup/p0_20261023/`；基线与差异证据位于 `docs/migration/`。按项目设置，`docs/`、`doc/` 不跟踪，文档与核对证据只保存在本地，clone 不包含这些记录。在当前工作空间执行以下只读校验，不需要 ROS 环境：

```bash
python3 script/g1_verify_p0.py --local
```

省略 `--local` 仅校验源码和本地核对证据。后续阶段修改源码后，P0 校验报告差异属于预期；保留 P0 快照并为新阶段单独记录验收。

P1 已移除业务入口、CMake 和配置中的原机器路径。公共入口默认 Foxy；加载 ROS 前会拒绝已混入的 Humble 环境。导出的同名配置优先于 `local.env`；工作空间默认由脚本位置确定，`G1_WS` 可覆盖。在 PC2 上部署时：

```bash
cp -n script/local.env.example script/local.env  # 保留已有配置
# 编辑 local.env：填写现场网卡、主机/雷达 IP、Domain 和所需 underlay
bash script/g1_build.sh
bash script/g1_check.sh > pc2-dependencies.json
```

默认新建 `build_pc2/`、`install_pc2/`、`log_pc2/`，原 `build/install/log` 保留；路径、架构或 ABI 改变时选择新的构建目录。厂商来源、校验和与依赖要求见 [PC2_DEPENDENCIES.md](third_party/PC2_DEPENDENCIES.md)。本机检查不需要 ROS：

```bash
bash script/g1_check.sh --offline
python3 script/test_p1_platform.py
```

建图与导航共用管理锁；停止操作只处理身份、命令行和进程组归属均通过核验的进程。旧 PID 文件若仍指向活进程，会拒绝操作并要求由原启动方核实。地图保存必须取得 `success=true` 回执并发现新的非空文件，`finish` 在保存失败时保留节点；会话和保存回执位于 `runtime/mapping/`。重启归档旧日志，当前日志从空文件开始。

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

# 无先验全局定位（保持静止），确认成功后才启动规划/Dry Run
bash script/g1_navigation.sh start-auto
bash script/g1_navigation.sh start-auto-dry

# 旧固定坐标入口；使用已有 script/auto_start_pose.env
bash script/g1_navigation.sh start-fixed-dry

# 人工定位与规划；在 RViz 中给定初始位置和朝向
bash script/g1_navigation.sh start
bash script/g1_navigation.sh continue

# Dry Run 导航桥
bash script/g1_navigation.sh start-dry
bash script/g1_navigation.sh continue-dry

# 航点和任务离线查看
bash script/g1_waypoints.sh list
bash script/g1_waypoints.sh validate

# PC2 已有巡航路线预览
bash script/g1_cruise.sh plan
```

巡航的多圈、暂停和停止说明保留在 [g1_cruise_README.md](script/g1_cruise_README.md)。其中原机器的安装路径和历史验证记录仅供追溯，不代表本轮已复验。

导航默认 `G1_NAV_PROFILE=safe`，前进上限 0.20 m/s；在 PC2 低速验收后可显式选择 `target`（1.0 m/s）。两档转速上限均为 0.40 rad/s，线加速/减速 0.20/0.30 m/s²，角加减速 4.0 rad/s²，禁止横移和倒退。修改速度档须停止并重启整个导航栈；Nav2 与两种桥的配套 YAML 由同一生成器写入 `runtime/navigation/`，桥脚本按同一档位启动。直接使用源码桥 YAML 会选择目标档。

FAST-LIO `/Odometry` 的 twist 属于 `body`，Nav2 统一使用旋转后的 `/Odometry_nav`（`nav_base`）；转换器与静态 TF 共用倒装雷达旋转，保留原采样时间戳。详见 [导航包说明](src/g1_nav2_bringup/README.md)。

真机桥入口为 `script/g1_navigation_real.sh`，`start` 和 `enable` 都核对本工作空间进程、Nav2 active、新鲜定位/TF、速度反馈、已加载参数及实际 `unitree_ros2` 后端；启动后仍需显式 `enable`。导航真机与 Dry Run 配置均启用定位联锁：状态接收/发布时间超过 1 秒、成功扫描超过 3 秒或状态无效时关门并清空速度。恢复定位后必须重新 enable 并发送新的 Twist。

## 项目运行展示

以下三张图片展示本项目 PC2 版本的历史运行情况；图片不代表本轮平台适配验收。

建图运行：

<img width="2468" height="1322" alt="截图 2026-09-20 15-37-42" src="https://github.com/user-attachments/assets/7c641cc5-257c-4467-8006-543a2e9ebd2a" />

大范围点云地图：

<img width="1320" height="910" alt="截图 2026-09-23 20-26-41" src="https://github.com/user-attachments/assets/b920b017-1370-4dbe-a483-06c6b594d13e" />

导航运行：

<img width="1562" height="857" alt="截图 2026-09-20 15-25-19" src="https://github.com/user-attachments/assets/4dba04f2-d4e9-4c2f-b56a-b40f8c8f0dfa" />

## P2 实时定位验收

`/g1_relocalization/status` 使用 reliable / transient-local / depth=1 的 String JSON。仅 `LOCALIZED` 或 `TRACKING` 且 `valid=true` 可就绪，消费者同时核验状态时间和成功扫描时间；TF 刷新、旧日志和单独的 fitness 都不能替代定位证据。

```bash
# 已加载 PC2 环境后，只检查定位与 TF，不启用运动
python3 script/g1_wait_localization.py --once --timeout 5 --tf-frame nav_base
# 完成 PC2 构建并加载对应 install 后，运行合成定位与 Dry Run 联锁测试
colcon test --build-base build_pc2 --install-base install_pc2 \
  --packages-select g1_relocalization g1_loco_bridge --event-handlers console_direct+
colcon test-result --test-result-base build_pc2 --verbose
```

录点、单点导航、多点任务及巡航继续复用任务包的健康判断。`--max-fitness-age` 的旧参数不能放宽状态 1 秒/成功扫描 3 秒的上限。人工初值需三帧独立同步扫描确认；P3 已将 `start-auto` 切换为全局搜索。时间与状态契约、测试结果、PC2 检查点和成组回退见 [P2_ACCEPTANCE.md](docs/migration/P2_ACCEPTANCE.md)。

## P3 全局自动定位

全局模式先采集静止扫描，再做全图 XY 与完整 360° yaw 粗搜索、多模式细化和 NDT 配准。默认验证最多 3 个候选，所有候选比较同一批后续新扫描，连续 3 帧确认，最多 3 轮采集。主候选稳定且三帧窗口平均覆盖率**严格大于 0.91** 可直接确认；`global_direct_accept_coverage: 1.0` 禁用该规则，此时须满足综合分和覆盖率分差，未验证的竞争位置会阻止确认。

`G1_LOCALIZATION_TIMEOUT_SEC` 默认 `180.0` 秒，从定位节点收到首帧同步数据开始覆盖采集、搜索、验证与重试。失败或超时返回非零，保留定位节点供 RViz 人工后备，随后使用 `continue` / `continue-dry`。诊断位于 `runtime/navigation/localization/session_*/`，包含输入 PCD、地图/有效参数 SHA-256、候选评分与逐帧验证记录。

地图地面模型、雷达高度、倒装方向及搜索范围须按现场核实。`global_matcher_check --map ...` 只检查地图索引；加独立扫描可检查搜索，仍须通过实时新帧确认。P3 本机证据与 PC2 检查点见 [P3_ACCEPTANCE.md](docs/migration/P3_ACCEPTANCE.md)。

## P5 讲解、人工放行与任务结果

P5 配套升级了任务接口、语音桥和任务执行器。旧无语音任务保持定时自动推进，新任务支持
`zh/en/both`、`speaker_id`、`wait_sec`、失败终止和 `auto/manual` 推进；末站人工等待也需放行。
真实任务先核验真实语音桥，讲解/等待时关门，取消先完成运动收尾。`g1_navigation_real.sh start`
管理真实语音进程，默认 `G1_AUDIO_VOLUME=-1` 保留当前音量；外部语音实例由原启动者停止。

```bash
# 离线校验旧任务；tour 替换为已配置的任务 ID
bash script/g1_waypoints.sh missions
bash script/g1_waypoints.sh mission tour --check-only
# 加载 PC2 环境后模拟任务，保留人工等待服务
bash script/g1_waypoints.sh mission tour --simulate --time-scale 0.01
# 同一工作空间/ROS Domain 的另一终端：
bash script/g1_waypoints.sh status
bash script/g1_waypoints.sh continue
```

放行标识绑定当前运行及站点；巡航的途中 `pause/resume` 继续独立保留，不能用任务 `continue`
替代。任务/巡航共用锁和停止未确认恢复标记，真实任务报告及 Action UUID 结果保存于
`runtime/missions/`。接口增加了字段，PC2 须用全新构建和安装目录一起重建三个配套包及客户端。

配置、执行与恢复方法见 [任务包说明](src/g1_task_manager/README.md)；本机证据、PC2 待验收项和
回退见 [P5_ACCEPTANCE.md](docs/migration/P5_ACCEPTANCE.md)。本机模拟通过不代表机器人播放完成，
声音与 PlayStop 效果仍待上机确认。

## P6 网页控制台

工作空间根目录即网页默认数据源。使用 PC2 系统 Python 创建环境：

```bash
/usr/bin/python3 -m venv --system-site-packages web_console/.venv
web_console/.venv/bin/python -m pip install --require-hashes -r web_console/requirements.txt
bash web_console/run.sh
```

默认 local 模式，访问 http://127.0.0.1:8080 。mock 模式独立演示；ros_readonly 显示 ROS 状态并支持配置编辑/建图管理；ros_control 显式启用导航准备、初始位姿、单点、讲解任务和连续巡航。ROS/局域网模式须设置 G1_WEB_TOKEN；所有模式共用 300 秒单设备操作权和版本冲突检查。

网页连续巡航接入原有多圈/循环与途中 pause/resume，独立于任务到站 continue。源码、本机验证、目标机检查与回退见 [P6_ACCEPTANCE.md](docs/migration/P6_ACCEPTANCE.md)，安装和使用见 [网页 README](web_console/README.md)。

## 迁移顺序

P0 基线与结构整理 → P1 PC2 环境和路径 → P2 新鲜定位状态与联锁 → P3 全局自动定位 → P4 速度反馈与 Nav2 → P5 语音和任务 → P6 网页控制 → P7 PC2 整体验收。

每阶段的源码范围、依赖、Foxy 适配点、验收和回退条件均在 [PC2_MIGRATION_PLAN.md](docs/PC2_MIGRATION_PLAN.md)。P1–P6 已完成源码与各阶段本机检查；Ubuntu 22.04 x86_64 版本或本机通过的检查不代表 PC2 已通过。

## 许可证

第三方组件遵循各自目录中的许可证。各业务包的 `package.xml` 也有各自声明；发布和分发时逐项核对，根目录目前未补充统一许可证。
