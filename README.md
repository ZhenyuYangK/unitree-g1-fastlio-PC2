# Unitree G1 EDU+ 29DOF · PC2 建图、导航与讲解

面向 **Unitree G1 EDU+ 29DOF 机器人 PC2** 的室内建图、导航与讲解系统。项目使用 Livox MID360 激光雷达，通过 FAST-LIO 建立三维地图，结合全局重定位与 Nav2 实现自主导航，并提供航点管理、多点讲解任务、连续巡航和网页控制台。

本仓库运行环境为 **Ubuntu 20.04 / ROS 2 Foxy / aarch64 / Python 3.8**。Ubuntu 22.04 / ROS 2 Humble / x86_64 环境请使用 [Ubuntu 22.04 版本](https://github.com/ZhenyuYangK/unitree-fastlio-ubuntu22.04)。

## 功能

| 功能 | 说明 |
| --- | --- |
| 激光建图 | MID360 点云与 IMU 融合，保存三维 PCD 地图，转换为二维导航地图 |
| 定位 | 无先验全局自动定位、人工初始位姿、固定初始位姿与 NDT 跟踪 |
| 自主导航 | 基于 Nav2 的路径规划与避障，支持单点目标和速度限制 |
| 航点与讲解 | 录制航点、编排多点任务，支持中英文讲解、到站停留和人工放行 |
| 连续巡航 | 按指定航点顺序执行单圈、多圈或循环巡航，支持途中暂停、恢复与停止 |
| 网页控制台 | 查看地图与状态、编辑航点和任务、管理建图、导航与巡航，支持手机访问 |
| 运动控制 | 真实机器人与 Dry Run 模式，定位失效时关闭运动安全门 |

## 安装与配置

以下命令均在仓库根目录执行。PC2 需已安装 ROS 2 Foxy、Nav2、CycloneDDS RMW、colcon，以及 PCL、Eigen、OpenSSL 和 Python 开发依赖。详细依赖见 [PC2 依赖说明](third_party/PC2_DEPENDENCIES.md)。

```bash
git clone https://github.com/ZhenyuYangK/unitree-g1-fastlio-PC2.git
cd unitree-g1-fastlio-PC2
cp -n script/local.env.example script/local.env
```

编辑 `script/local.env`，填写现场网络和设备配置：

| 配置项 | 用途 |
| --- | --- |
| `G1_NETWORK_INTERFACE` | 与雷达、机器人通信的网卡 |
| `G1_HOST_IP` / `G1_LIDAR_IP` | PC2 主机与 MID360 的 IP 地址 |
| `ROS_DOMAIN_ID` | ROS 通信域，各操作终端保持一致 |
| `G1_PCL_SETUP` / `G1_UNITREE_ROS2_SETUP` | 可选依赖环境路径 |
| `G1_MAP_PCD` / `G1_MAP_PGM` / `G1_MAP_YAML` | 三维地图、二维地图及地图配置路径 |
| `G1_WAYPOINTS_FILE` / `G1_MISSIONS_FILE` | 航点与任务文件路径 |

然后在 PC2 上编译：

```bash
bash script/g1_build.sh
```

脚本自动加载配置与 ROS 环境，默认构建到 `build_pc2/`、安装到 `install_pc2/`。地图与任务数据需按现场准备并单独备份；三维地图、二维地图及航点应对应同一区域，航点与任务的 `map_id` 保持一致。

## 使用方法

### 1. 建图与地图转换

```bash
bash script/g1_mapping.sh start       # 启动雷达与 FAST-LIO
bash script/g1_mapping.sh save        # 保存当前地图，继续建图
bash script/g1_mapping.sh map-path    # 查看本次 PCD 保存路径
bash script/g1_mapping.sh finish      # 保存成功后结束建图
```

默认保存到 `maps/map_<时间戳>.pcd`。使用转换工具生成二维 PGM 和 YAML 地图：

```bash
python3 maps/pcd2pgm_multilayer.py --help
# 将输入路径替换为实际保存的 PCD；地面平面、地图边界和障碍高度参数需按现场设置
python3 maps/pcd2pgm_multilayer.py maps/map_20261025_120000_000000.pcd --output-prefix map_multilayer
```

将 `script/local.env` 中的 `G1_MAP_PCD` 指向保存的 PCD，`G1_MAP_PGM` 和 `G1_MAP_YAML` 指向转换后的文件。建图结束后再启动导航。

### 2. 定位与导航

自动定位时保持机器人静止，定位成功后脚本启动 Nav2：

```bash
bash script/g1_navigation.sh start-auto
```

需要人工定位时，执行 `start`，在 RViz 中使用 **2D Pose Estimate** 设置位置与朝向，再执行 `continue`：

```bash
bash script/g1_navigation.sh start
# 在 RViz 中设置初始位姿，等待定位成功
bash script/g1_navigation.sh continue
```

定位与规划就绪后，启动真实机器人桥接；机器人运动模式由操作者设置：

```bash
bash script/g1_navigation_real.sh start    # 启动桥接与语音，安全门保持关闭
bash script/g1_navigation_real.sh enable   # 放行真实运动
```

随后可在 RViz 中使用 **Nav2 Goal** 发送单点目标，或通过网页、任务与巡航命令执行路线。默认前进速度上限为 **0.20 m/s**；定位失效时安全门关闭，恢复定位后需重新启用。

仅运行定位与规划流程时，可用 `start-auto-dry` 替代 `start-auto`，使用 Dry Run 桥，不发送真实运动命令。固定初始位姿使用 `start-fixed`，位姿配置位于 `script/auto_start_pose.env`。

```bash
bash script/g1_navigation.sh status         # 查看导航状态
bash script/g1_navigation_real.sh disable   # 关闭真实运动安全门
bash script/g1_navigation_real.sh stop      # 停止真实桥接与托管语音进程
bash script/g1_navigation.sh stop           # 停止导航
```

### 3. 航点与讲解任务

定位正常且机器人静止时，在当前位置录制航点：

```bash
bash script/g1_waypoints.sh record home --name 起点
bash script/g1_waypoints.sh list
bash script/g1_waypoints.sh validate
```

默认航点文件为 `task_data/maps/laboratory_map_v1/waypoints.yaml`，任务文件为 `task_data/missions/missions.yaml`。可通过网页编辑任务的航点顺序、讲解内容、停留时间与人工放行方式；配置格式见 [任务使用说明](src/g1_task_manager/README.md) 和 [讲解任务示例](src/g1_task_manager/config/tour_missions.example.yaml)。

```bash
bash script/g1_waypoints.sh missions                     # 列出任务
bash script/g1_waypoints.sh mission tour --check-only     # 检查配置
# 导航、定位与真实桥接就绪后执行；tour 替换为实际任务 ID
bash script/g1_waypoints.sh mission tour --execute
```

在另一终端查看状态或放行当前站点：

```bash
bash script/g1_waypoints.sh status
bash script/g1_waypoints.sh continue
```

任务到站后关闭运动安全门，再进行讲解与停留；人工推进的站点需执行 `continue`。前台任务可用 `Ctrl+C` 请求取消并等待收尾。

### 4. 连续巡航

将 `point_a`、`home` 替换为已保存的航点 ID。执行前需准备好导航、定位与真实桥接。

```bash
bash script/g1_cruise.sh plan --points point_a home
bash script/g1_cruise.sh start --points point_a home --laps 3 --execute
# 持续循环时，将 --laps 3 替换为 --loop
```

巡航保持前台运行，在另一终端操作：

```bash
bash script/g1_cruise.sh pause
bash script/g1_cruise.sh status
bash script/g1_cruise.sh resume
bash script/g1_cruise.sh stop
```

`pause/resume` 用于巡航途中暂停与恢复；任务的 `continue` 用于到站人工放行。更多参数见 [巡航使用说明](script/g1_cruise_README.md)。

### 5. 网页控制台

使用 PC2 的系统 Python 安装并启动：

```bash
/usr/bin/python3 -m venv --system-site-packages web_console/.venv
web_console/.venv/bin/python -m pip install --require-hashes -r web_console/requirements.txt
bash web_console/run.sh
```

浏览器访问 **http://127.0.0.1:8080**。默认 `local` 模式可查看地图、编辑航点与任务配置；`mock` 模式可体验模拟界面。

需要通过局域网查看 ROS 状态或控制机器人时，设置访问口令与运行模式：

```bash
read -rs -p '网页访问口令: ' G1_WEB_TOKEN
export G1_WEB_TOKEN
G1_WEB_HOST=0.0.0.0 G1_WEB_MODE=ros_control bash web_console/run.sh
```

手机或其他电脑访问 `http://PC2的IP:8080`，输入口令并获取操作权。`ros_control` 支持导航、任务和巡航控制；只查看 ROS 状态时使用 `ros_readonly`，该模式仍允许配置编辑和建图管理。详细操作见 [网页使用说明](web_console/README.md)。

## 运行展示

建图：

![FAST-LIO 建图](https://github.com/user-attachments/assets/7c641cc5-257c-4467-8006-543a2e9ebd2a)

三维点云地图：

![三维点云地图](https://github.com/user-attachments/assets/b920b017-1370-4dbe-a483-06c6b594d13e)

导航：

![Nav2 导航](https://github.com/user-attachments/assets/4dba04f2-d4e9-4c2f-b56a-b40f8c8f0dfa)
