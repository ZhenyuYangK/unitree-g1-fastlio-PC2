# G1 Web Gateway API v0.1

状态为“已实现”的接口可在本次框架运行。“占位”接口固定拒绝，不运行机器人命令；“规划”路径尚未注册。

## 传输和访问

- 页面、`/api/v1`、`/ws/state` 同源；不启用任意 CORS。
- 本机 local / mock 可以无口令；非回环监听地址以及 ros_readonly / ros_control 模式必须设置 G1_WEB_TOKEN。
- `POST /api/v1/session` 用口令换取 12 小时 HttpOnly、SameSite=Strict Cookie；口令不放 URL 或浏览器存储。
- HTTP 写请求和 WebSocket 检查浏览器 Origin；请求体拒绝额外字段。
- 演示启动器提供 HTTP。正式 HTTPS 终止和访问权限属于真机控制接入阶段，配置 HTTPS 后设置 G1_WEB_COOKIE_SECURE=1。
- 无账号角色管理、登录限流或持久会话；本框架不作为公网入口。

## 已实现接口

| 方法/路径 | 用途 | 模式 |
| --- | --- | --- |
| GET /api/v1/health | 服务存活、模式与 schema_version | 无需会话；不代表机器人就绪 |
| GET /api/v1/session | 查询当前会话是否认证 | 所有模式 |
| POST /api/v1/session | {token} 登录 | 所有模式 |
| DELETE /api/v1/session | 注销当前会话 | 所有模式 |
| GET /api/v1/state | 获取最新状态信封 | 所有模式 |
| GET /api/v1/catalog | 地图标识、导航点、任务目录 | 所有模式；配置缺失返回 503 |
| GET /api/v1/openapi.json | 认证后的机器可读 HTTP 契约 | 所有模式 |
| POST /api/v1/control/acquire | {owner} 取得控制权，返回 lease_id、expires_in | 所有模式；local/ROS 用于配置编辑 |
| POST /api/v1/control/renew | 续租 | 已持有租约的设备 |
| POST /api/v1/control/release | 释放 | 同上 |
| POST /api/v1/missions/start | 开始任务；真实模式需 catalog revision | mock / ros_control |
| POST /api/v1/missions/continue | 放行当前站点 | mock / ros_control |
| POST /api/v1/missions/cancel | 取消本次运行，真实模式绑定 Action UUID | mock / ros_control |
| POST /api/v1/operations | 异步建图及导航白名单动作 | 建图：local / ros_readonly / ros_control；导航：ros_control |
| POST /api/v1/navigation/go-to | {request_id,waypoint_id,revision} 单点导航 | mock / ros_control |
| POST /api/v1/navigation/initial-pose | {request_id,x,y,yaw} 提交初始位姿，yaw 为弧度 | ros_control |
| GET /api/v1/navigation | 导航就绪、阻塞原因及生命周期状态 | 所有模式 |
| GET /api/v1/navigation/logs | 最近导航操作与业务日志 | 认证后；mock 返回空列表 |
| GET /api/v1/navigation/report?goal_id=... | 下载该 UUID 对应的任务报告，限制在 runtime/missions 内 | ros_control |
| GET /api/v1/operations/{operation_id} | 查询持久化操作状态 | 已认证设备 |
| GET /api/v1/mapping | 建图状态、可执行动作、进程、最近操作与成果 | 所有模式 |
| GET /api/v1/mapping/logs | 操作/雷达/FAST-LIO 最近日志 | 已认证设备 |
| GET /api/v1/mapping/artifact?session_id=... | 当前会话的已保存 PCD 下载 | 已认证设备；忙碌或未保存时拒绝 |
| WS /ws/state | 每 0.5 秒最新状态；重连先发全量快照 | 所有模式；不接收控制命令 |

额外已实现的现场数据接口：

| 方法/路径 | 用途 | 模式 |
| --- | --- | --- |
| GET /api/v1/map | PGM 对应的像素尺寸、分辨率、origin、版本和图片地址 | local / ros_readonly / ros_control |
| GET /api/v1/map/image?version=... | 实际 PGM 转换的 PNG；ETag/304；旧图片版本返回 409 | local / ros_readonly / ros_control |
| POST /api/v1/waypoints | 新增 {id,name,x,y,yaw} | local / ros_readonly / ros_control |
| PUT /api/v1/waypoints/{id} | 更新 {name,x,y,yaw}，保留采样元数据 | local / ros_readonly / ros_control |
| DELETE /api/v1/waypoints/{id} | 删除未被任务引用的点 | local / ros_readonly / ros_control |
| POST /api/v1/missions | 新增 {id,name,steps} | local / ros_readonly / ros_control |
| PUT /api/v1/missions/{id} | 更新 {name,steps}，steps 沿用现有任务 YAML 结构 | local / ros_readonly / ros_control |
| DELETE /api/v1/missions/{id} | 删除任务配置 | local / ros_readonly / ros_control |

GET /catalog 返回 source=workspace、revision、files、导航点及任务的原始 steps；响应带 ETag。所有配置写入须同时携带 X-Control-Lease 与 If-Match（catalog 的 revision）。缺版本返回 428，版本变化返回 409，不覆盖打开编辑窗口后的其他修改。成功返回 {saved,revision,catalog,backup,message}。

保存先在临时文件调用现有 WaypointStore/load_missions 校验，备份两个原文件，再原子替换被编辑的单个 YAML。修改任务和导航点共享一个配置版本；任务锁、CLI 写入锁或 recovery_required 标记存在时拒绝保存。POST 重试使用相同 ID 会冲突，不自动重发。地图像素不提供编辑接口。

控制权凭证通过 `X-Control-Lease` 请求头传递，不出现在快照中。租约有效期为 300 秒（5 分钟），页面每 5 秒续租；每次续租成功后重新计时 5 分钟。

前端续租请求串行执行。临时网络错误、超时或服务端错误时保留原凭证，暂停操作并自动重试；手机返回前台、页面恢复或网络恢复时立即核验。只有后端明确返回 `CONTROL_REQUIRED`、主动释放或网关实例变更才清除凭证，过期后不自动重新申请。旧凭证请求的迟到结果不影响新取得的权限。页面显示“操作剩余时间”，状态未同步或权限待核验时不显示可操作倒计时。

## 命令示例

开始：

```json
{
  "request_id": "045de9a2-203c-4228-855b-76222f9fc66f",
  "mission_id": "five_point_tour"
}
```

人工放行：

```json
{
  "request_id": "79134957-5c34-44c5-88fb-a7ce1d9c58b9",
  "run_id": "从最新任务状态获取",
  "wait_id": "从最新 WAITING_FOR_CONTINUE 状态获取"
}
```

取消：`{request_id, run_id}`。所有 request_id 必须为 UUID v4。

成功返回 HTTP 202：`{accepted: true, simulated: true, request_id, operation_id, run_id, message}`。表示已接受，最终执行结果通过状态查询。页面不自动重试命令。

mock 的同一租约和 request_id 重试在 10 分钟、最多 256 条的内存缓存内返回原响应。ros_control 使用 SQLite 持久记录：跨网关重启/租约更换，同一 UUID/参数返回原回执；参数不同返回 REQUEST_ID_CONFLICT。PENDING 或结果未知的命令不会重发，返回 COMMAND_UNCONFIRMED。重试仍需有效租约。真实开始任务/单点导航必须携带 catalog revision；缺失返回 428，已变化返回 409。

## 状态信封

```json
{
  "type": "state",
  "schema_version": 1,
  "instance_id": "每次后台实例重新生成",
  "sequence": 42,
  "server_time": 1790000000.0,
  "data": {
    "mode": "mock",
    "simulated": true,
    "localization": {"state": "TRACKING", "valid": true, "stale": false},
    "pose": {"x": 2.0, "y": 2.0, "yaw": 0.0},
    "mission": {
      "active": true,
      "simulated": true,
      "run_id": "...",
      "mission_id": "five_point_tour",
      "mission_name": "五站双语讲解巡航",
      "state": "WAITING_FOR_CONTINUE",
      "current_step": 1,
      "total_steps": 5,
      "current_waypoint": "point_01",
      "next_waypoint": "point_02",
      "wait_id": "...",
      "message": "等待放行",
      "gate_closed": true,
      "goals_terminal": true,
      "distance_remaining": null
    },
    "control": {"held": true, "owner": "浏览器 A", "remaining_sec": 12.0},
    "map": {"available": true, "simulated": true, "id": "demo_map"},
    "capabilities": {"mission_control": true, "process_control": false, "waypoint_write": false, "teleop": false, "video": false},
    "logs": []
  }
}
```

- 前端用 instance_id 识别网关重启，丢弃旧租约；同一实例内丢弃 sequence 较旧的数据。
- local 模式默认加载现场地图和配置，pose=null，定位/任务未知。ros_readonly 接入 ROS 定位、TF、路径及任务状态，运动命令仍返回 501。ros_control 显式接入真实高层操作。
- data.configuration 包含 available、revision、error、write_blocker；前端根据版本变化重取目录。data.map 含地图资源版本，原图或 YAML 变化后自动刷新。waypoint_write / mission_write 表示文件编辑能力。
- 未接入可靠机器人来源时 gate_closed/goals_terminal 为 null，不显示确认成功。
- ROS 定位检查接收间隔、stamp_ns、last_success_stamp_ns 和状态；超过 1 秒未更新或成功扫描超过 3 秒则 valid=false。
- ROS 任务服务超过 2 秒没有有效响应则 mission.stale=true。状态过期不意味着任务已经停止。
- WS 认证失败关闭码 4401，Origin 不匹配为 4403；登录过期后重新认证。

## 错误

```json
{"error": {"code": "STALE_WAIT", "message": "任务或等待标识已过期，请重新查询状态"}}
```

401：会话/口令错误；403：来源不符；404：任务不存在；409：租约、任务互斥或标识冲突；422：请求体或配置校验失败；428：缺少 If-Match 版本；501：功能未接入；503：现场配置不可用。

## 导航状态与长操作

ROS 快照增加以下字段（schema_version=1 的兼容性扩展）：

- pose：新鲜的 map→nav_base 平面坐标、yaw、frame_id、stamp_ns；TF 超过 0.5 秒或不可用时为 null。
- path：available、stale、frame_id、stamp_ns、points；只接收 map 坐标，最多 800 个点并保留首尾，超过 3 秒标记过期。
- navigation：ready、blockers、checks、active_nav_goal、lifecycle。checks 包括四个 Nav2 生命周期节点、运动桥、语音桥、真实任务管理器；参数/生命周期信息超过 2 秒失效。任务执行器需 execute_real=true 且现场文件路径一致，重复节点/Action 服务端不视为就绪。
- navigation.lifecycle：available、active、busy、phase、processes、operation、blocker、can_start/can_continue/can_prepare/can_stop。
- mission：真实网页任务附带 goal_id、controllable、report_url（有终态报告时）。run_id 尚未取得时可用 goal_id 取消。其他客户端任务仅观察，不收编为可取消目标。
- capabilities：navigation_control、initial_pose、single_goal 按模式启用；后端仍独立校验权限和运行状态。

POST /operations 新增 navigation.start（自动定位+Nav2）、navigation.start-manual、navigation.continue、navigation.prepare（真实运动/语音桥及任务执行器）、navigation.stop。导航动作拒绝 name、自定义命令或路径，使用同一持久化 operation_id 查询机制。脚本退出码不能代替新鲜定位/Nav2、关门及进程退出确认。

任务和导航长操作持久化路径、停止未确认恢复方式及完整交互见 [导航使用说明](navigation.md)。浏览器失联后机器人侧任务继续，人工等待阶段持续等待；重连不自动重发。

## 后续接口（当前未注册）

- POST /waypoints/record，使用机器人实时位置采样。
- 独立的运动桥开门接口；当前由任务执行器按阶段管理。
- POST /audio/speak 与终止结果。
- 禁止将 mock 结果用于真机确认。

## 异步建图

POST /operations 请求体：{request_id: UUIDv4, operation: mapping.start|mapping.save|mapping.finish|mapping.stop, name?: 地图名称}，需 X-Control-Lease。name 用于启动时生成独立文件名，只接受汉字/字母/数字/下划线/短横线，最长 48 字符，不接受文件路径或任意命令。

成功返回 HTTP 202：{accepted:true,simulated:false,request_id,operation_id,message}。GET /operations/{id} 返回 QUEUED / RUNNING / SUCCEEDED / FAILED / INTERRUPTED、时间、错误原因及退出码。请求 UUID 与参数持久去重；复用 UUID 修改参数返回 409。

状态快照新增 data.mapping 和 data.mapping_preview，capabilities.mapping_control 表示可用建图入口。mapping 含 phase、active、busy、can_start/can_save/can_finish/can_stop、blocker、processes、session、operation。预览为独立坐标系的点云 XY 投影，每帧最多 5000 个采样点，stale 表示断流；不能作为导航栅格或障碍判断。

保存必须确认 Trigger.success=true、PCD 更新与内容完整。finish 先保存再停止，失败保留已有进程。命令和日志位于 runtime/mapping，网页操作持久记录位于 runtime/mapping/web。结果未确认时以实际进程和文件核对为准。网关不因浏览器断线停止采集，不自动替换导航地图。

## P6 PC2 扩展

状态 data.cruise 为连续巡航，独立于讲解任务 data.mission。Foxy 的导航检查名称为 recoveries_server。定位 valid 复用 P2 health.py：状态接收/发布时间最多 1 秒、成功扫描最多 3 秒，拒绝未来时间和无效 int64。

| 路径 | 请求/响应 | 权限 |
| --- | --- | --- |
| GET /api/v1/cruise | state、live、active、run_id、controllable、route、lap/laps、dwell_remaining、停止确认 | 已认证；local/mock 返回不可用 |
| POST /api/v1/cruise/start | request_id、points 数组、revision、laps(默认1)、loop(false)、timeout_sec(60)、dwell_sec(2) | ros_control、租约、版本、新鲜定位与导航就绪；与任务/建图互斥 |
| POST /api/v1/cruise/pause | request_id、run_id | ros_control、租约、当前运行标识 |
| POST /api/v1/cruise/resume | 同上；只允许已确认 PAUSED，定位须有效 | 同上 |
| POST /api/v1/cruise/stop | 同上；结果以 CANCELED/停止确认为准 | 同上 |

连续巡航控制返回 202 表示请求接受，不表示已暂停/停止。真实命令使用原有持久化去重日志；同 UUID、不同内容返回冲突，网关重启后未确认命令不再次执行。
