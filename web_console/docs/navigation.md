# 网页定位与导航

面向 PC2 / Foxy / Python 3.8；网页复用 P1–P5 的定位脚本、Nav2、安全桥和任务执行器。源码及本机隔离测试完成，PC2 现场验收待执行。

## 启动

先编译更新后的任务管理器，再启动网页网关。以下命令只构建和启动网关；定位、运动桥和任务均由页面中的显式操作启动。

```bash
source unitree_ros2/setup.sh
cd "$G1_WS"
colcon build --packages-select g1_task_manager --executor sequential
cd ..
read -rs -p '网页访问口令: ' G1_WEB_TOKEN
export G1_WEB_TOKEN
G1_WEB_MODE=ros_control bash web_console/run.sh
```

浏览器打开 http://127.0.0.1:8080。局域网监听沿用 `G1_WEB_HOST=0.0.0.0`；正式部署使用 HTTPS 反向代理并设置 `G1_WEB_COOKIE_SECURE=1`。

模式边界：

| 模式 | 导航观察 | 导航操作 |
| --- | --- | --- |
| local | 现场地图和配置，无机器人位姿 | 不启停导航，不执行任务 |
| mock | 示例位置、路径和模拟任务 | 模拟巡航/单点/取消/放行，无 ROS |
| ros_readonly | 真实定位、TF、路径、任务状态和就绪检查 | 运动与任务只读；保留已有建图与配置编辑能力 |
| ros_control | 同上 | 显式启停导航、初始位姿、桥准备、单点与巡航执行 |

## 使用流程

1. 取得操作权，进入“系统接入”。先结束建图，然后点击“自动定位并启动导航”。页面展示异步操作结果、定位/Nav2 就绪状态与日志。此时不会自动启动真实运动桥。
2. 自动定位失败或场景有歧义时，基础进程保留。点击“设置初始位姿”：可填 X/Y 和角度，也可在地图按住目标位置、拖动朝向箭头，松开后在窗口中预览并提交。页面输入角度为度，接口与 ROS 使用弧度。提交不等于定位成功。定位有效后点击“定位后继续启动 Nav2”。
3. 点击“准备运动与语音桥”。复用 `g1_navigation_real.sh start`，并准备真实模式任务 Action 服务；校对执行器的导航点/任务文件路径，重复或配置不匹配的实例会拒绝。准备完成后确认控制门关闭；机器人运动模式仍由现场操作者按原方式切换。
4. 回到“运行总览”。地图显示机器人位置、朝向与规划路径。任务站点顺序为虚线，Nav2 路径为绿色实线；超过新鲜度限制后不再显示为当前路径。可选择跟随机器人，或点击“显示全图”退出跟随。
5. 单点导航：选择已有导航点，预览位置和朝向，点击“前往该点”，在确认窗口中开始。通过同一个任务 Action 的 `waypoint:<id>` 请求生成临时单站任务，复用任务锁、定位检查、运动门及取消收尾；不修改现场任务文件。到点后自动结束，不播报、不等待人工放行。
6. 巡航：选择任务并开始。按任务配置导航、播报和推进。人工推进站点持续等待“继续下一站”，人工推进的末站也需要放行。需要语音的任务必须先确认真实语音桥就绪。
7. 取消时绑定本次任务 Action UUID，等待收尾结果。只有 `gate_closed=true` 且 `goals_terminal=true` 才显示停止收尾已确认。结束任务后可下载任务报告。
8. 任务结束后，在“系统接入”点击“结束导航”。先确认关门，再停止本网页/脚本持有且身份匹配的进程；外部启动、未纳入 PID 管理的实例不会被收编。应由原启动者结束这些实例。

自动初始化与本地 `start-auto` 共用定位逻辑。页面可显示“采集定位点云”“全局搜索”“比较定位候选”“重新采集定位”和第几次尝试；候选分数是几何匹配分，不是位置正确的概率。`VERIFYING_CANDIDATES` 和 `RETRYING` 都保持 `valid=false`，不会放行导航。只有最终 `AMBIGUOUS` / `FAILED` 才结束就绪等待，失败回执保留新鲜定位状态中的具体原因。状态消息的新鲜度与成功定位扫描的新鲜度分别判断，尚未定位时也能显示实时进度。

默认最多 3 轮、同时验证最多 3 个空间不同候选，每轮候选验证最长 20 秒，总预算由 `G1_LOCALIZATION_TIMEOUT_SEC` 控制（默认 180 秒，自第一帧同步数据起，覆盖采集/搜索/验证/重试）。采集帧数、新扫描稳定性及原有绝对质量门槛保留；多个位置均可匹配时，还要求综合评分差及覆盖率差分别达到 0.08，覆盖率差可通过 `global_confirmation_coverage_gap` 配置。可信候选超出验证数量时不能直接忽略后放行。阈值需要实测校验，静止重复结构仍可能需要人工初值。

诊断保存在 `runtime/navigation/diagnostics/relocalization/`：`inputs_*.json` 含地图/配置哈希、配置原文及入口覆盖参数，`global_*.pcd/.json` 含每轮搜索输入和候选，`verification_*.jsonl` 含同批新扫描的候选比较过程。仅启动重定位节点时，可通过 `diagnostics_directory` 指定已有目录；空值不保存诊断文件。

导航期间任务执行器自行按阶段管理运动门。网页没有任意阶段“暂停/恢复”，也不会发送裸速度指令。未接入的紧急停止占位按钮已移除，普通任务取消保留原有语义。

## 断线、恢复与互斥

- 浏览器失联或租约到期，自主任务由机器人侧继续执行，人工等待阶段持续等待；页面不自动重发命令。状态流超过两秒未更新时禁用网页操作。
- 网关重启后丢弃旧租约，重新取得操作权。网关持久保存请求回执、Action UUID、已知 run_id 及阶段；按 UUID 查询原 Action 结果，并与当前任务状态核对。不取消其他客户端的目标。
- 网关在收到 run_id 前也允许使用本次持久化 goal_id 请求取消。接受结果未知时保留运行记录，禁止重复开始。
- 机器人侧 Action 终态保存在 `runtime/missions/actions/<goal_id>.json`；网关重启或 Action 服务重启后可从该记录核对终态。缺少结果时显示未知，不推断已经停止。
- 建图、导航准备、单点和巡航互斥；生命周期操作持有任务锁，防止 CLI 在导航启停期间开始任务。准备期间锁定配置写入。真实任务开始必须匹配页面预览时的配置版本。
- 进程使用 PID 与内核启动时间共同识别。旧的无身份令牌记录、PID 被复用、命令返回后进程未退出，均不能显示停止成功；旧实例由原启动者核对后处理。

`STOP_UNCONFIRMED` 继续使用根 README 的现场恢复流程，网页不自动删除恢复标记。完成遥控停止、关门、原任务退出、目标终止核对，按原流程处理恢复标记并重启任务管理器后，还需要在停掉网页网关的条件下归档 `runtime/navigation/web/run.json`，再重新启动网关。归档保留证据，不把记录删除视为停止确认。

## 文件与接口

- `runtime/navigation/web/commands.sqlite3`：真实网页命令持久回执；重复 UUID 不会再次执行，结果未知的命令拒绝重发。
- `runtime/navigation/web/run.json`：本次 Action 归属、阶段、事件与结果。
- `runtime/navigation/web/operations/`：导航长操作状态及命令日志。
- `runtime/navigation/logs/`：定位、Nav2、运动桥、语音桥及任务执行器日志。
- `runtime/missions/`：原任务执行器的报告和恢复标记。

具体字段与权限见 [API 契约](api.md)。导航脚本的 PCD、PGM、YAML 应属于相同现场地图；地图自动转换/切换仍单独实现。

## 验证

离线网关测试和任务单元测试不连接机器人。浏览器测试使用临时地图和注入的合成适配器：

```bash
PYTHONPATH=web_console/backend web_console/.venv/bin/python -m unittest discover -s web_console/tests -v
PYTHONPATH=src/g1_task_manager /usr/bin/python3 -m unittest discover -s src/g1_task_manager/test -v
PYTHONPATH=web_console/backend:web_console/tests web_console/.venv/bin/python web_console/tests/browser_navigation_smoke.py
```

ROS 导航联调脚本在导入 ROS 前固定回环接口和独立 Domain 181，只创建合成状态和服务；不启动雷达/运动/语音节点：

```bash
source unitree_ros2/setup.sh
PYTHONPATH="web_console/backend:web_console/tests:$PYTHONPATH" web_console/.venv/bin/python web_console/tests/ros_navigation_smoke.py
```

2026-10-17 已通过离线网关、浏览器、只读 ROS 回环、导航 ROS 回环与任务执行器模拟集成验证。实际定位效果、真实运动、语音播放及断线现场表现仍按 [现场验证方案](real_robot_validation.md) 分阶段验收。首轮运动继续使用该文档的临时 0.20 m/s 限速桥，不能把默认 1.0 m/s 配置作为首轮测试速度。

## PC2 连续巡航

巡航任务页支持导航点顺序、多圈/循环、途中暂停/恢复和停留剩余时间。底层使用 P5 g1_cruise.sh，暂停继续持有 mission.lock；人工讲解任务的 continue 仅用于到站放行。GET /api/v1/cruise 可核对状态，POST /api/v1/cruise/start 和 /pause、/resume、/stop 需要操作权；控制请求绑定本次 run_id，旧进程标识被拒绝。失联且历史状态未终止时显示 STOP_UNCONFIRMED。
