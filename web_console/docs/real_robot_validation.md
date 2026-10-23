> 本文保留参考版本的详细操作顺序；其中原机的已有测试/历史记录只作参考。PC2 实际通过项仅以本仓库 P6/P7 验收记录为准。

# 当前开发电脑与 G1 的现场验证方案

适用：PC2 / Ubuntu 20.04 / aarch64 / Foxy / Python 3.8。以下为待执行的现场流程；本机 Python 3.8、Humble 和浏览器合成检查见根目录 docs/migration/P6_ACCEPTANCE.md。上游原机结果不作为本设备验收证据。

这份方案用于逐阶段执行。只有本阶段通过后才进入下一阶段。本文中的运动、语音、录点及配置修改命令由现场负责人执行；准备本文时只执行了通信查询和离线检查，没有发送运动、模式切换、导航目标或语音指令。

2026-10-17 更新：网页 `ros_control` 已接入定位/Nav2 准备、初始位姿、桥准备、单点与巡航开始/放行/取消，已通过合成数据和 ROS 回环验证，尚无新增真机运动通过记录。完整页面操作见 [导航使用说明](navigation.md)。下文原有 `ros_readonly` + CLI 验证流程继续有效；首次网页真机验证也须先完成定位核对和临时限速桥准备。

现场网页建图的启动、保存、结束与验收步骤见 [建图使用说明](mapping.md)。建图和定位/Nav2 互斥，请分别验收。

## 0. 已完成的现场基线

2026-10-16 在当前电脑检查到：

| 项目 | 实测结果 | 能说明什么 |
| --- | --- | --- |
| 有线网卡 | enp4s0 UP，192.168.123.99/24 | 本机机器人网段接口可用 |
| Wi-Fi | wlo1 UP，10.252.32.190/17 | 可尝试从同一可达网络访问网页；仍须实际验证设备互通 |
| ROS | Foxy / 按 P1 核实的 DDS/RMW / 现场 Domain | 使用 PC2 实际配置 |
| GetFsmId | code=0，value=801 | 机器人只读查询成功 |
| GetFsmMode | code=0，value=0 | 机器人只读查询成功；不据此判定已进入可行走模式 |
| GetBalanceMode | code=7301 | 此查询未取得 LocoState；当前只读探针将此码列为可接受，不代表平衡/步态已验证 |
| 依赖、PCD、PGM/YAML | g1_check.sh --offline 通过 | 本机依赖与地图文件可用 |
| 导航点和任务 | 5 个点，1 个任务，引用校验通过 | 实际任务是 five_point_patrol，当前是中文讲解、逐站人工放行 |
| 本机业务进程 | 检查时未见定位/Nav2/运动桥运行 | 下一步需要启动定位链路 |

机器可读记录：`web_console/.preview/robot_validation/baseline.json`。以上结果不是定位、实际运动或真实播放通过记录。

## 1. 终端安排与统一环境

- A：定位/Nav2 启动、状态与日志。
- B：网页网关，保持前台运行。
- C：执行任务，保留此终端以便 Ctrl+C 取消。
- D：任务 status/continue、配置检查；真实运动前用它前台运行限速桥。
- E：真实桥的准备/状态、语音检查，以及 D 被占用后的 status/continue。

所有终端均从项目根目录开始，使用同一组现场文件：

```bash
cd "$G1_WS"
export G1_WS="$PWD"
export G1_NETWORK_INTERFACE=enp4s0
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export G1_WAYPOINTS_FILE="$G1_WS/task_data/maps/laboratory_map_v1/waypoints.yaml"
export G1_MISSIONS_FILE="$G1_WS/task_data/missions/missions.yaml"
export G1_MAP_PCD="$G1_WS/maps/map.pcd"
export G1_MAP_YAML="$G1_WS/maps/map_multilayer.yaml"
export G1_MAP_PGM="$G1_WS/maps/map_multilayer.pgm"
export G1_VERIFY_DIR="$PWD/web_console/.preview/robot_validation"
source unitree_ros2/setup.sh
```

若之前人为设置过 CYCLONEDDS_URI，先核对其绑定的是 enp4s0；不要沿用回环测试用的 lo。使用新的终端通常可避免残留测试变量。机器人有线 DDS 与浏览器 Wi-Fi 访问是两个连接，不要求手机接入机器人有线网段。

终端 A 准备本轮原文件副本，不覆盖已有备份：

```bash
mkdir -p "$G1_VERIFY_DIR"
cp -n "$G1_WAYPOINTS_FILE" "$G1_VERIFY_DIR/waypoints.before.yaml"
cp -n "$G1_MISSIONS_FILE" "$G1_VERIFY_DIR/missions.before.yaml"
bash script/g1_waypoints.sh path
bash script/g1_waypoints.sh validate
bash script/g1_waypoints.sh mission five_point_patrol --check-only
```

通过：路径与网页后端一致，显示 5 个点、实际任务与有效引用。这里不使用示例任务 five_point_tour。重复现场验收时请更换 G1_VERIFY_DIR 的目录后缀，各终端保持相同路径。

## 2. 定位和 Nav2：先建立真实状态源

终端 A：

```bash
bash script/g1_navigation.sh start-auto
```

该入口启动雷达、FAST-LIO、自动定位和 Nav2，不启动真实速度桥、不自动切换机器人模式。启动时保持机器人静止，等待定位确认。使用 start-auto，避免先启用同名 Dry Run 桥，后面切真机时发生冲突。

启动成功后检查：

```bash
bash script/g1_navigation.sh status
python3 script/g1_wait_localization.py --once --timeout 5
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /recoveries_server
ros2 lifecycle get /bt_navigator
timeout --signal=INT 6 ros2 run tf2_ros tf2_echo map nav_base
```

通过标准：

- 等待工具退出码 0，状态为 LOCALIZED 或 TRACKING，状态发布时间和成功扫描时间都新鲜。
- 四个 Nav2 节点均为 active [3]。
- RViz 中地图、点云和机器人所处物理位置对齐；持续的 map→nav_base 变换正常，静止时位置不明显跳变。
- tf2_echo 用 timeout 结束的 124 仅表示到了观察时限，不能当作 TF 失败；需检查超时前是否持续收到有效变换。

遇到 AMBIGUOUS、FAILED、定位漂移、雷达/IMU 缺失时停在本阶段，用 logs 排查，不发送导航目标：

```bash
bash script/g1_navigation.sh logs
```

## 3. 网页与手机：验证真实地图和状态

终端 B，先在本机使用，再验证另一台设备：

```bash
read -rs -p "网页访问口令: " G1_WEB_TOKEN
export G1_WEB_TOKEN
export G1_WEB_HOST=0.0.0.0
export G1_WEB_PORT=8080
G1_WEB_MODE=ros_readonly bash web_console/run.sh
```

本机：`http://127.0.0.1:8080`。同一可达 Wi-Fi 的设备：本次检查地址为 `http://10.252.32.190:8080`；地址变化以 `ip -brief address` 为准。

通过标准：

1. 登录后显示“ROS”，实际实验室 PGM、讲解点1～5与 five_point_patrol 对应的真实任务名。
2. 网页显示定位有效，与终端 A 的新鲜状态一致。
3. 两台设备可以同时查看；一个取得操作权后，另一个的编辑入口禁用。
4. 手机访问失败时，先验证能否到达电脑 Wi-Fi 地址与 8080 端口，再检查本机防火墙和 Wi-Fi 客户端隔离；不要把浏览器连通失败判成机器人 DDS 失败。

当前版本的边界：

- ros_readonly 的运动与任务通道只读，导航点和任务文件允许编辑，建图生命周期可以操作。
- 网页已接入机器人实时位姿和 Nav2 路径；TF 超过 0.5 秒不显示为当前位姿，路径超过 3 秒标记过期。虚线表示任务站点顺序，绿色实线为规划路径。
- 没有运行任务时 GetMissionState 服务可能不存在，任务状态未知/过期是预期结果，不等于机器人停止已确认。
- ros_readonly 中真实“开始、继续、取消”继续禁用；ros_control 已实现这些操作。紧急停止接口仍未实现。下文基线运动流程使用既有 CLI，网页替代流程见文末新增验收项。

### 网页控制新增验收项

完成第 2 阶段定位、Nav2 和第 6 阶段临时 0.20 m/s 真机桥准备后，保持其他任务空闲，使用 ros_control 网关：

1. 就绪状态需与 CLI 一致；页面运动桥显示实际配置的 0.20 m/s 上限。“准备运动与语音桥”应复用已存在的限速桥，禁止先停止它再用默认参数重启。
2. 机器人静止时验证初始位姿预览、位置与朝向；活动任务或 Nav2 目标存在时拒绝重设位姿。
3. 选择附近已验证导航点，核对目标预览后单点导航；观察真实位置和路径，确认到点、关门及目标终止后显示完成。
4. 导航期间取消；核对只取消本次 UUID，页面必须等待门关闭与目标终止。服务失联不能显示停止已确认。
5. 按既有语音验证步骤核对实际播放，再执行真实讲解巡航；人工推进站点和末站放行规则保持一致。
6. 两设备争抢操作权、页面刷新、断线重连和受控重启网关：任务不重复启动，人工等待持续等待，重新取得操作权后可放行或取消原任务。
7. 注入定位失效、状态断流和停止未确认，核对禁用开始及恢复提示。只读、模拟或服务请求成功不能记作真机运动/播放通过。

## 4. 配置回写、多设备同步和删除保护

此阶段保持没有运行任务。建议只使用以下测试 ID，保留原来的 5 个讲解点不变。

| 操作 | 通过标准 |
| --- | --- |
| 在地图新增 web_verify_tmp，填写可识别名称、坐标和 yaw，保存 | 命令行 show 能读到完全相同的数据，第二个浏览器自动出现该点 |
| 编辑该点的名称和坐标 | 后端文件变化，地图位置变化，刷新浏览器仍保留 |
| 新建任务 web_verify_edit，引用该点，设置中文讲解与人工放行 | missions 和 check-only 能看到相同路线、文本、等待参数 |
| 在网页尝试删除仍被 web_verify_edit 引用的点 | 拒绝删除，错误说明任务引用，原数据保留 |
| 按下方方法制造旧窗口版本冲突 | 保存被拒绝，不覆盖后端的新版本 |
| 删除 web_verify_edit，再删除 web_verify_tmp | 删除成功，第二个页面自动同步 |
| 查看备份 | backup/web_console/ 每次保存前备份两个原 YAML |

命令行核对：

```bash
bash script/g1_waypoints.sh show web_verify_tmp
bash script/g1_waypoints.sh missions
bash script/g1_waypoints.sh mission web_verify_edit --check-only
```

版本冲突验证：在浏览器打开 web_verify_tmp 编辑窗口但暂不保存；终端用 update web_verify_tmp 重新采样同一个测试点；等另一页面收到更新后，尝试保存旧窗口。必须提示版本冲突，不能覆盖新数据；点“重新载入后端”再编辑。update 会按当前真实位置重新采样，因此仅用于这个测试点，不用于原 5 个讲解点：

```bash
bash script/g1_waypoints.sh update web_verify_tmp
```

测试顺序：先做版本冲突，再删除测试任务和测试点。采样要求定位有效、机器人静止；失败采样不能作为版本冲突测试通过。

## 5. 真实 ROS 状态与放行联调：模拟运动

在尚未启动真实速度桥、也没有其他任务服务时，终端 C 执行：

```bash
bash script/g1_waypoints.sh mission five_point_patrol --simulate
```

这是现有任务执行器的 ROS 模拟模式，不向机器人发送导航或语音，但创建真实 status/continue 服务，适合核对网页状态接入。

终端 D 每站执行，最后一站也放行：

```bash
bash script/g1_waypoints.sh status
bash script/g1_waypoints.sh continue
```

通过标准：网页站点序号、任务阶段和终端状态对应；每站进入 WAITING_FOR_CONTINUE 后持续等待；人工放行才能前进；末站放行后 C 输出 SUCCEEDED。此阶段必须按模拟结果记录，不能记作机器人运动或真实播放成功。可通过 `/api/v1/state` 的 `data.mission.simulated=true` 区分；页面顶层 ROS 标识仅说明网关连接 ROS。

同时验证：模拟任务运行时网页禁止保存配置；任一时刻只有一个任务持有工作空间任务锁。C 结束后确认该模拟进程已退出，再进入真实任务。

## 6. 首次真实运动前：使用临时限速桥

当前源码与 Nav2 运行时配置允许最高线速度 1.0 m/s，本轮初次验证用 0.20 m/s 的桥输出上限。不要直接按默认最高速度开始首次导航。

此步骤要求现场有人操作遥控器/实体停止手段，机器人模式由你按既有操作流程切换。只读探针成功不代替模式确认。不要同时从 RViz 或其他终端发送导航目标。

终端 D 前台运行桥，用启动参数覆盖速度上限，不改正式 YAML。执行前确认没有同名桥；若已经存在，先确认任务空闲，再由原启动者停止它：

```bash
ros2 run g1_loco_bridge g1_cmd_vel_dry_run --ros-args \
  --params-file "$G1_WS/src/g1_loco_bridge/config/g1_loco_bridge_nav2_real.yaml" \
  -p max_linear_x:=0.20 \
  -p min_effective_linear_x:=0.20 \
  -p max_angular_z:=0.40 \
  -p min_effective_angular_z:=0.40
```

D 保持前台。参数是启动时读取，不能用“ros2 param set 返回成功”代替运行中限速已生效。本轮使用的是该前台桥，不要在结束它后让 start 脚本重新生成默认 1.0 m/s 的桥继续测试。

终端 E 启动脚本会复用已存在的真实桥，并准备真实语音桥：

```bash
export G1_AUDIO_VOLUME=70
bash script/g1_navigation_real.sh start
bash script/g1_navigation_real.sh status
ros2 param get /g1_cmd_vel_dry_run dry_run
ros2 param get /g1_cmd_vel_dry_run max_linear_x
ros2 param get /g1_cmd_vel_dry_run min_effective_linear_x
ros2 param get /g1_cmd_vel_dry_run cmd_vel_topic
python3 -m g1_task_manager.audio_readiness --timeout 8
```

通过：dry_run=False、max_linear_x=0.20、min_effective_linear_x=0.20、输入 /cmd_vel_nav；真实语音桥 dry_run=false；D 日志显示初始安全门关闭，机器人没有因“启动桥”自行行走。若复用了已存在的语音桥，G1_AUDIO_VOLUME 不改变该已运行实例的音量。

**任务执行器会按阶段开关安全门，本方案不手动执行 g1_navigation_real.sh enable。** 网页没有可靠安全门状态字段，不用网页“实时连接”作为门状态确认。

## 7. 真实语音：机器人保持静止

终端 E：

```bash
ros2 action send_goal /g1_audio/speak g1_task_interfaces/action/Speak \
  "{text: '真机语音验证，机器人保持静止。', speaker_id: 0, wait_sec: 6.0}" \
  --feedback
```

通过：实际听到对应中文，机器人不移动，Action 返回 success=true。仅命令返回成功而没有听到声音，不能记作通过。wait_sec 是配置等待，不是播放完成回调；按现场句子长度标定，避免放行时仍在播报。

如要验收英语，单独验证：

```bash
ros2 action send_goal /g1_audio/speak g1_task_interfaces/action/Speak \
  "{text: 'Robot audio verification.', speaker_id: 1, wait_sec: 6.0}" \
  --feedback
```

当前现场五点任务只配置中文。需要双语巡航时，在网页新建测试任务设置 both，并填全两种语言；不把中文任务通过写成双语已验收。

## 8. 近距离单点：先验证整个真实闭环

先选择可通行、无障碍、与起点相距约 0.5～1 m 的位置，方向与预期到站朝向一致。用遥控器将机器人移到该目标并静止，终端 E 采样：

```bash
bash script/g1_waypoints.sh record web_verify_goal --name "近距离验证点"
```

采样成功后，网页应自动出现新点。用遥控器回到确认过的起点，面向目标，重新确认定位有效。不要把未知距离的 point_01 当作首次近距离目标。

在网页创建任务：

- ID：web_verify_single；名称：近距离单点验证。
- 唯一站点：web_verify_goal。
- 导航超时：120 秒；额外停留：0；推进：人工放行。
- 中文讲解：“已到达近距离验证点”；等待先设 6 秒，按实际播放调整。

终端 C 先检查，再由现场负责人执行实际运动：

```bash
bash script/g1_waypoints.sh mission web_verify_single --check-only
bash script/g1_waypoints.sh mission web_verify_single --execute
```

执行期间终端 E 查询：

```bash
bash script/g1_waypoints.sh status
```

通过标准：

1. 开始前就绪检查通过，终端状态为真实执行，非模拟。
2. 机器人前往确认过的近点，实际速度符合本轮限速，没有突发跳变或错误方向。
3. 到站后停止、播报对应文本，再进入 WAITING_FOR_CONTINUE；等待 10 秒仍保持原站。
4. 网页显示真实任务、当前点和阶段；因没有位姿接入，不要求网页机器人图标移动。
5. 终端 E 执行下面的 continue 后，C 才输出最终 SUCCEEDED：

```bash
bash script/g1_waypoints.sh continue
```

最终任务报告必须同时满足 `gate_closed=true` 与 `goals_terminal=true`；不能只看网页状态、进程退出或机器人看似静止。报告保存到 `runtime/missions/`。网关的状态服务可能在任务结束后消失，最终结果以 C 和报告为准。

## 9. 取消与重连验证

### 9.1 人工等待阶段取消

重新执行 web_verify_single，到人工等待后在 C 按 Ctrl+C。通过：机器人保持停止，C 最终报告 CANCELED 且 gate_closed=true、goals_terminal=true；不能产生后续导航。真实语音的取消不等于一定中断机载 TTS 播放，播放中断效果单独记录。

### 9.2 行进阶段取消

单点完成、等待阶段取消通过后，才在同一确认过的短路径进行行进取消。C 启动任务，实际开始行进后按 Ctrl+C；负责人观察机器人停止，并核对本任务目标终态和最终关门结果。任何未确认均记为未通过，不直接重跑任务。

### 9.3 浏览器失联与恢复

只在机器人已停止、任务处于人工等待时断开观察设备 Wi-Fi，或关闭浏览器。通过：后台仍保持本任务人工等待，不自动前进；页面重连后恢复同一 run_id/当前点，无重复启动；任务继续由 E 的 continue 控制。

断线网页必须显示状态过期并禁用编辑。浏览器失联不会取消由 CLI 执行的自主任务。不要用行进时拔机器人网线作为本轮首个断线测试；动态 DDS/定位故障停止需单独设计并记录，未测不能宣称通过。

## 10. 五点真实巡航

仅在近距离单点、语音与取消检查通过，且逐段确认路线可通行后执行。本轮仍使用 D 的限速桥。

终端 C：

```bash
bash script/g1_waypoints.sh mission five_point_patrol --check-only
bash script/g1_waypoints.sh mission five_point_patrol --execute
```

终端 E 每站：

```bash
bash script/g1_waypoints.sh status
bash script/g1_waypoints.sh continue
```

每站记录：实际到达点、朝向、中文内容、等待是否足够、人工放行是否生效，以及网页阶段是否一致。只在状态为 WAITING_FOR_CONTINUE、现场确认后放行，不连续盲发 continue。第 5 站也要放行，最终 C 输出 SUCCEEDED 且报告关门/目标终态均为 true。

运行中尝试在网页打开编辑入口，应禁用或明确拒绝写入；本轮不要用外部文本编辑器绕过锁改正在运行的配置。

## 11. 异常处理与结束

异常时优先使用现场遥控器/实体停止手段，并在 C 请求取消。终端 E 可显式请求关闭安全门：

```bash
ros2 service call /g1_loco_bridge/enable_real_nav std_srvs/srv/SetBool "{data: false}"
```

必须看到 success=true。调用超时、服务不存在、进程退出或脚本退出码 0 都不能代替成功确认。`g1_navigation_real.sh disable` 当前会吞掉部分命令失败，因此恢复核对优先查看上述直接服务响应。

若出现 STOP_UNCONFIRMED 或 recovery_required.json：保留报告和恢复标记，确认原任务退出、运动门关闭、属于本任务的 Nav2 goal 已终止，再按根 README 的“停止未确认后的恢复”处理。不要删除标记来绕过本阶段，也不要取消其他客户端的全部目标。

正常结束顺序：

1. C 最终结果已确认、机器人静止。
2. E 请求关门并检查 success=true。
3. D 按 Ctrl+C 结束本轮手动前台启动的限速桥。脚本没有登记这个前台桥的 PID，不能只靠脚本 stop 清理它。
4. E 执行下列脚本，清理由脚本启动的语音桥和业务链路；B 按 Ctrl+C 关闭网关。

```bash
bash script/g1_navigation_real.sh stop
bash script/g1_navigation.sh stop
```

归档日志和任务报告后再启动下一轮，因为启动器会生成运行时配置并清理该会话部分日志。仅在任务结束后，在网页先删除 web_verify_single 等测试任务，再删除 web_verify_goal 等测试点，保留原 5 点和原任务；不需要整文件覆盖恢复已合法修改的生产数据。

在再次启动导航前保存本轮产物（同名归档已存在时使用新的 G1_VERIFY_DIR）：

```bash
cp -a "$G1_WS/runtime/missions" "$G1_VERIFY_DIR/missions_runtime"
cp -a "$G1_WS/runtime/navigation/logs" "$G1_VERIFY_DIR/navigation_logs"
```

## 12. 验收记录

以下项目按实际结果填写“通过 / 失败 / 未测”，网页能力与机器人能力分别记账：

| 编号 | 项目 | 结果与证据 |
| --- | --- | --- |
| V01 | 机器人只读通信 | 已通过；baseline.json |
| V02 | 依赖、地图和任务引用校验 | 已通过；5 点 / five_point_patrol |
| V03 | 雷达、定位新鲜度、RViz 对齐 | 待现场执行 |
| V04 | Nav2 四节点 active、TF 正常 | 待现场执行 |
| V05 | 手机访问、登录、真实地图与状态 | 待现场执行 |
| V06 | 配置回写、备份、刷新持久化 | 自动化已通过；本轮现场待执行 |
| V07 | 多设备同步、租约、引用/冲突保护 | 自动化已通过；本轮现场待执行 |
| V08 | ROS 模拟任务与网页阶段对应 | 待现场执行；不能计作实际运动 |
| V09 | 限速桥、初始关门、真实语音 | 待现场执行 |
| V10 | 近距离单点运动、到站、播报、放行 | 待现场执行 |
| V11 | 等待取消、行进取消、最终关门与目标终态 | 待现场执行 |
| V12 | 人工等待时网页断线/恢复 | 待现场执行 |
| V13 | 五点巡航、各站放行、末站完成 | 待现场执行 |
| V14 | 网页真实启动/放行/取消/急停 | 未实现，不纳入已通过范围 |
| V15 | 网页实时位姿、Nav2 路径 | 未实现 |
| V16 | PC2 Foxy 原生部署、动态故障停止专项 | 源码已迁移，现场待验收 |

建议为每轮保留：代码版本/工作区差异、实际参数、waypoints/missions 副本、Nav2/定位/运动/语音日志、完整任务报告、网页截图及现场负责人观察。访问口令不要写入验收记录。
