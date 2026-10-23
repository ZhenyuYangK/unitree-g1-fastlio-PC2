# G1 PC2 网页控制台

P6 已迁入上游固定版本 `afaf7bc5daefc98cfa045c0b50d05e17b7ac2053` 的前端和 HTTP/WebSocket 契约，适配本仓库根工作空间、Python 3.8 和 Foxy。网页运行在 PC2 本地。源码及本机隔离检查已完成，Foxy/aarch64 和硬件现场检查见 [P6 验收记录](../docs/migration/P6_ACCEPTANCE.md)。

## 安装和启动

从包含 `src/`、`script/`、`web_console/` 的工作空间根目录执行。先按根 README 配置 `script/local.env` 并构建 ROS 包。

```bash
/usr/bin/python3 -m venv --system-site-packages web_console/.venv
web_console/.venv/bin/python -m pip install --require-hashes -r web_console/requirements.txt
bash web_console/run.sh
```

访问 http://127.0.0.1:8080 。默认 local 模式读取当前工作空间文件。启动器加载 P1 公共配置；ROS 模式额外加载 Foxy 和 PC2 install。虚拟环境必须使用系统 `/usr/bin/python3`，以便访问相同版本的系统 rclpy。不要复制开发电脑的虚拟环境。

运行依赖及全部传递依赖在 `requirements.txt` 固定版本和 SHA-256，由 `requirements.in` 按 Python 3.8/aarch64 解析生成。开发依赖的完整锁为 `requirements-dev-pc2.txt`；`requirements-dev.txt` 为开发依赖输入。

| 模式 | 数据与配置 | 建图管理 | 运动/任务 |
| --- | --- | --- | --- |
| local（默认） | 实际地图/航点/任务，支持写入 | 显式调用 CLI，无 ROS 点云预览 | 禁用 |
| mock | 内置示例，独立于现场文件 | 禁用 | 五站模拟、模拟单点和取消/放行 |
| ros_readonly | 实际文件、定位、路径、位姿与任务遥测 | 显式调用 CLI，含点云预览 | 只读 |
| ros_control | 同上 | 同上 | 导航准备、人工初值、单点、讲解任务、连续巡航 |

local 和 ros_readonly 都允许配置编辑、建图生命周期操作。“只读”指运动和任务控制通道。

局域网/ROS 模式需要访问口令：

```bash
read -rs -p '网页访问口令: ' G1_WEB_TOKEN
export G1_WEB_TOKEN
G1_WEB_HOST=0.0.0.0 G1_WEB_MODE=ros_readonly bash web_console/run.sh
# 完成现场准备后，显式选择控制模式：
# G1_WEB_HOST=0.0.0.0 G1_WEB_MODE=ros_control bash web_console/run.sh
```

其他设备访问 `http://PC2的可达IP:8080`。只体验模拟界面可执行 `G1_WEB_MODE=mock bash web_console/run.sh`。

## 文件和操作权

`G1_WS` 默认为本仓库根目录，复用任务包公共路径函数；`G1_WAYPOINTS_FILE`、`G1_MISSIONS_FILE`、`G1_MAP_YAML`、`G1_MAP_PGM` 可覆盖现场文件，相对路径按 G1_WS 解析。缺失/无效数据会显示错误。默认地图为 `maps/map_multilayer.yaml`，默认航点和任务为 `task_data/maps/laboratory_map_v1/waypoints.yaml` 与 `task_data/missions/missions.yaml`。

地图坐标按 YAML 的 origin、旋转和分辨率转换。编辑支持地图选点、严格任务验证、双语讲解、站点顺序和人工/自动推进；保存要求配置版本匹配，与 CLI 共用文件锁和 mission.lock，备份到 `backup/web_console/`。运行任务、巡航暂停、建图和恢复标记均阻止不兼容的写入。

一个设备持有操作权，租约 300 秒，每 5 秒续约。暂时断线保留凭据，恢复后重新核验；过期后清除凭据并等待显式获取，不能自动抢占其他设备。控制命令通过 HTTP，WebSocket 只推送 2 Hz 状态；命令不因断线自动重发。

## 导航、任务和连续巡航

运行总览提供全局定位、人工初值、继续启动、准备真实桥/任务执行器、单点导航，以及带双语讲解的任务开始/取消/到站放行。Foxy 使用 recoveries_server；定位有效性复用 P2 公共健康规则；真实桥检查实际 real_backend/backend_name。任务结果按 Action UUID 查询和持久化。

“巡航任务”页的“连续巡航 · PC2”调用 P5 `g1_cruise.sh`，支持指定导航点顺序、多圈/无限循环、单点超时和到站停留。途中暂停/恢复与到站人工放行分别操作；暂停期间继续持有任务锁，保留剩余停留时间。接口接收暂停/停止请求后，仍需等待状态 PAUSED/CANCELED 及停止确认。浏览器或网关断开后自主任务继续；连续巡航使用 P5 的本地控制 socket，恢复时核对协调器本次运行标识；旧协调器缺少 run_id 时使用原 CLI 管理。

网页不发送 Twist，不直接启用运动门；运动由既有执行器和安全桥负责。STOP_UNCONFIRMED 或 recovery_required.json 按根 README/CLI 恢复流程处理，不以删除文件代替停止确认。

## 验证

```bash
web_console/.venv/bin/python -m pip install --require-hashes -r web_console/requirements-dev-pc2.txt
PYTHONPATH=web_console/backend:web_console/tests web_console/.venv/bin/python -m pytest -q web_console/tests
node web_console/tests/geometry_test.js
```

可选浏览器工具使用目标机支持的 Playwright 与已有 Chrome。`browser_control_smoke.py`、`browser_mapping_smoke.py`、`browser_navigation_smoke.py`、`browser_pc2_smoke.py` 均使用隔离服务或临时工作空间。ROS 合成脚本在独立 Domain/lo 上运行；本机 Humble 测试结果不能替代 PC2 Foxy 验收。

接口见 [api.md](docs/api.md)，流程见 [navigation.md](docs/navigation.md)、[mapping.md](docs/mapping.md)，现场检查见 [real_robot_validation.md](docs/real_robot_validation.md)。本轮不安装自启服务。
