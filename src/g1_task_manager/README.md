# PC2 任务、讲解和人工放行

P5 配套包为 `g1_task_interfaces`、`g1_audio_bridge` 和 `g1_task_manager`。
接口有新增字段和服务，须在 PC2 的全新构建/安装目录一起重建，再加载该安装环境。
默认任务服务器仅检查配置；真实执行由服务器的 `execute_real` 参数控制。

## 配置和离线检查

旧任务仍支持 `waypoint`、`timeout_sec`、`dwell_sec`，默认定时自动推进。
新步骤可增加 `advance_mode: auto|manual` 和 `speech`。整个文件先严格校验，未知字段、
重复 YAML 键、无效数字、缺少语言或航点引用都会阻止出发。

```yaml
- waypoint: home
  timeout_sec: 60.0
  dwell_sec: 1.0
  advance_mode: manual
  speech:
    mode: tts
    language: both
    failure_policy: abort
    content:
      zh: {speaker_id: 0, text: 欢迎来到实验室。, wait_sec: 5.0}
      en: {speaker_id: 1, text: Welcome to the laboratory., wait_sec: 5.0}
```

此片段放在任务的 `steps` 下；`language` 可选 `zh`、`en`、`both`，双语按中文、英文顺序执行。
`speaker_id` 为非负 int32，`wait_sec` 为有限正数，当前 `failure_policy` 只支持 `abort`。
五站示例在 [tour_missions.example.yaml](config/tour_missions.example.yaml)，使用前须替换
`map_id`、航点、文本和现场校准的等待时长，再保存到单独的现场任务文件。

在工作空间根目录运行（`tour` 替换为实际任务 ID）：

```bash
bash script/g1_waypoints.sh validate
bash script/g1_waypoints.sh missions
bash script/g1_waypoints.sh mission tour --check-only
# 另一个任务文件可在 mission 后指定 --missions-file /absolute/path/missions.yaml
```

## 模拟、状态与放行

```bash
bash script/g1_waypoints.sh mission tour --simulate --time-scale 0.01
# 第二终端，相同工作空间和 ROS Domain：
bash script/g1_waypoints.sh status
bash script/g1_waypoints.sh continue
```

模拟保留 ROS 状态/放行服务，不连接 Nav2、定位、安全门和机器人音频。
真实流程为：逐段就绪检查 → 导航 → 关门 → 中文/英文讲解 → 额外停留 → 人工等待。
`manual` 在末站也必须放行；没有自动出发倒计时。每次放行绑定本轮 `run_id` 和当前
`wait_id`，重复、过期及其他阶段的请求被拒绝。Ctrl+C 请求取消并等待收尾。

## 真实任务与 Action

导航规划和定位准备好后，`bash script/g1_navigation_real.sh start` 会启动或复用
`dry_run=false` 的语音桥；安全门保持关闭。默认 `G1_AUDIO_VOLUME=-1` 保留当前音量，
可在 `script/local.env` 显式设置为 0..100。`stop` 只停止本工作空间管理的进程，
外部语音实例由原启动者停止。旧无语音任务的执行器不要求语音服务。

```bash
# 按原有现场流程准备好导航、定位和真实安全桥后：
bash script/g1_waypoints.sh mission tour --execute

# 或加载 PC2 环境后启动 Action 服务器（默认配置为 execute_real=false）：
ros2 run g1_task_manager g1_task_manager_node --ros-args -p execute_real:=true
# 另一个终端：
ros2 run g1_task_manager g1_mission tour
ros2 run g1_task_manager g1_mission waypoint:home
```

含讲解的真实任务先确认唯一的标准语音 Action 服务端和 `dry_run=false`，预检不发声。
到站讲解、停留和人工等待期间关门；语音失败终止任务。取消语音的响应等待放在运动
收尾之后；停止须确认关门和本次 Nav2 目标终止。TTS 成功回执加 `wait_sec` 只表示
请求成功和配置等待结束，真实播放完成及 PlayStop 中断效果须在机器人上确认。

| 接口 | 用途 |
| --- | --- |
| `/g1_task/execute_mission` · ExecuteMission | 多点任务；`waypoint:<ID>` 生成临时单点任务 |
| `/g1_task/get_state` · GetMissionState | 查询当前执行器的运行、站点及放行标识 |
| `/g1_task/continue` · ContinueMission | 提交当前 `run_id` / `wait_id` |
| `/g1_task/cancel` · Trigger | 请求 Action 服务器取消，等待最终结果确认 |
| `/g1_audio/speak` · Speak | 可取消的文本讲解及配置等待 |

状态/放行服务在执行器存活时可用。真实任务报告保存在 `runtime/missions/*_<任务ID>.json`；
Action 结果按 UUID 保存于 `runtime/missions/actions/<UUID>.json`，服务器保留结果 24 小时。
单点 Action 与多点任务共用执行器，不修改现场任务文件。
停止未确认或异常退出保留 `runtime/missions/recovery_required.json`；先按现场流程确认
机器人停止、关门和目标终止，再人工归档处理标记，不能自动删除后重试。

## PC2 巡航

`script/g1_cruise.sh` 转发到 `g1_task_manager.cruise`，保留多圈、无限循环、途中暂停后
重试未完成目标、停留剩余时间和本机控制 socket。它与任务共用 `mission.lock` 和恢复标记，
暂停期间也持锁。巡航 `pause/resume` 与任务到站 `continue` 是不同操作。
使用示例见 [巡航说明](../../script/g1_cruise_README.md)。
