# G1 已有导航点巡航与暂停

新增 `script/g1_cruise.sh`，为单文件 Bash 入口（内嵌 Python），无需配套 Python 文件、无需编译。复用本工作空间的 `g1_task_manager` 源码中的导航客户端、安全门与任务收尾代码。原有导航点管理双文件脚本继续使用。

## 安装

将 g1_cruise_scripts.tar.gz 放到 PC2 主目录：

```bash
export WS=/home/g1test/Desktop/irongroup/yzy/g1_slam_ws_source/g1_slam_ws
export G1_WS="$WS"
tar -xzf "$HOME/g1_cruise_scripts.tar.gz" -C "$WS"
bash "$WS/script/g1_cruise.sh" help
```

要求现有 `src/g1_task_manager`、系统 Python 3.8、PyYAML、ROS2 Foxy、Nav2 和真实安全桥。优先使用已有 `script/g1_run_verified.sh` 加载执行环境；没有该脚本时加载 Foxy 和本工作空间 install/setup.bash。继承终端中的 DDS 配置，不改变构建类型。

## 预览和执行

```bash
# 预览所有已保存的导航点：按 ID 字母序排列，默认一圈
bash "$WS/script/g1_cruise.sh" plan

# 推荐先用已经验证的往返顺序
bash "$WS/script/g1_cruise.sh" plan --points point_a home

# 导航定位和真实桥接已运行、机器人定位正确后，前台执行一圈
bash "$WS/script/g1_cruise.sh" start --points point_a home --execute
```

start 保持前台运行，请保留这个终端。运行前应结束已有任务，不用另行启动任务服务；巡航直接复用底层执行代码。它与原 g1_taskctl 共用 `runtime/missions/mission.lock`，暂停时仍占用该锁。不要同时在 RViz 或其他客户端发送目标，那些客户端不遵守本项目文件锁。

默认真实安全门服务固定为 `/g1_loco_bridge/enable_real_nav`；TF 年龄上限 0.5 秒、fitness 上限 2.0、fitness 有效期 15 秒。脚本不调用运动模式切换 API。

## 在第二个 PC2 终端暂停、继续、停止

每个终端都设置相同的 WS、G1_WS，并使用相同用户：

```bash
export WS=/home/g1test/Desktop/irongroup/yzy/g1_slam_ws_source/g1_slam_ws
export G1_WS="$WS"

bash "$WS/script/g1_cruise.sh" pause
bash "$WS/script/g1_cruise.sh" status
bash "$WS/script/g1_cruise.sh" resume
bash "$WS/script/g1_cruise.sh" stop
```

这些命令只控制本脚本启动的巡航，不控制原有 `g1_mission roundtrip`。采用本机 UNIX socket 通信，不新增 ROS 服务；必须在同一台 PC2、同一工作空间下运行。

| 状态 | 含义 |
| --- | --- |
| STARTING / CHECKING | 初始化或检查下一目标，安全门应在检查前关闭 |
| NAVIGATING | 已进入当前目标的发送/导航阶段 |
| DWELLING | 已到达，关门停留 |
| PAUSING | 已收到暂停请求，正在完成关门与目标取消 |
| PAUSED | 安全门关闭、巡航拥有的目标已终止，暂停完成 |
| RESUMING | 接受继续请求，随后重新检查定位 |
| STOPPING | 正在停止并收尾 |
| SUCCEEDED | 全部圈数与停留完成 |
| CANCELED | 用户停止且收尾完成 |
| FAILED | 导航、定位或其他错误；不会自动重试 |
| STOP_UNCONFIRMED | 停止状态未确认，不能继续；按原项目恢复流程处理 |

`pause` / `stop` 最多等待约40秒给出确认，超时只表示请求尚未确认，不能视为停止成功。`resume` 返回请求接受，不代表已完成定位检查。仅 `PAUSED` 可继续；停止请求不可撤销。

途中暂停：先关门，再取消本巡航发送的 Nav2 目标。继续时重新检查 TF/fitness，向尚未完成的同一导航点重新发送目标，Nav2 重新规划；该次目标超时重新计时。若目标已成功结束，则下一步访问后续点。

到点停留时暂停：保存剩余停留时间；继续后完成剩余停留，再去下一点。暂停不会占用目标执行超时。

`stop` 或巡航终端 Ctrl+C 都执行关门/取消收尾，结束本次巡航；结束后不能 resume，需要重新 start。不要用 Ctrl+Z 或 kill -9 代替暂停/停止。

## 圈数与顺序

```bash
# 全部已有点，一圈（最终停在最后一个点，不自动返回起点）
bash "$WS/script/g1_cruise.sh" start --execute

# 指定顺序，三圈
bash "$WS/script/g1_cruise.sh" start --points point_a home --laps 3 --execute

# 持续循环，需显式停止；首次实机验证不要直接使用循环模式
bash "$WS/script/g1_cruise.sh" start --points point_a home --loop --execute

# 每段超时90秒，每点停留5秒
bash "$WS/script/g1_cruise.sh" start --points point_a home --timeout 90 --dwell 5 --execute
```

`--points` 省略时使用所有点的 ID 字母序；这不是按距离规划最短路线。显式顺序允许重复点。启动时读取一次点文件生成本次路线快照；运行中修改点文件不会改变本次路线。不会改写 waypoints.yaml 或 missions.yaml。

可指定其他点文件：`plan --file /完整路径/waypoints.yaml` 或同样给 start 传 `--file`。每个点必须来自该文件，地图和定位应对应。

## PC2 首次验证

1. `plan --points point_a home`：核对顺序、坐标。
2. 在导航定位与桥接正常时 `start --points point_a home --execute`。
3. 行进途中第二终端执行 pause；确认机器人停止、输出 PAUSED，且 gate_closed 与 goals_terminal 均为 true。
4. 执行 resume；确认继续前往未完成目标。
5. 再执行 stop；确认 CANCELED、两个收尾标志为 true。
6. 再跑完整一圈，确认 SUCCEEDED；通过后再使用多圈或持续循环。

脚本不在执行环境中直接操作你的机器人，上述是待你在 PC2 完成的验收。

## 记录与异常

`runtime/cruise/status.json` 保存最近状态，每次巡航另有时间戳报告文件。实时状态通过 `status` 向正在运行的进程查询；若进程不可达而缓存仍是运行/暂停状态，会显示 STOP_UNCONFIRMED，而不是把旧状态当作当前停止确认。

整个巡航（包括暂停）保留 `runtime/missions/recovery_required.json`；只有正常收尾并确认关门与目标结束后才清理。硬杀进程、断电或收尾失败会保留恢复标记。本脚本不会自动删除此前任务遗留的保护标记。

本脚本的暂停与停止是软件控制接口。脚本进程被强制杀死或失联时不能保证执行清理，应使用现有机器人停止手段确认状态。

## 本地检查范围

已通过 Bash 语法、Python 3.8 语法、真实上传点文件的路线预览，以及基于现有 execute_mission 的模拟执行：有序多圈、途中暂停后重试当前点、到点停留暂停、停止、定位失败不重试、关门/取消未确认时拒绝继续、循环退出、失联状态判定。

当前执行环境没有 ROS2 Foxy，也禁止创建 UNIX socket，因此没有完成真实 Nav2/安全桥通信或跨进程 socket 联调；这两部分须在 PC2 验证。
