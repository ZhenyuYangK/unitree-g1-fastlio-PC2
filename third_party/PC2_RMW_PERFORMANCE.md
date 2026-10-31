# PC2 雷达发布积压的 RMW 修复

现场现象及原始记录见 `runtime/inspection/startup_diagnosis_20261030.md`。雷达发布积压导致 FAST-LIO、TF 和导航反馈过期；Nav2 可以进入 active，但启动脚本或运动桥后续的就绪检查失败。

## 修复内容

PC2 原先的 `rmw_cyclonedds_cpp` 编译缓存未设置 `CMAKE_BUILD_TYPE`，编译标志没有优化选项。使用现场已有的同一份 Foxy RMW 源码，以 `Release`（`-O3 -DNDEBUG`）构建独立安装目录。保留现有 CycloneDDS 运行库，不修改雷达频率、消息格式、TF/定位/速度反馈新鲜度阈值或运动安全门。

原生 C++ 基准使用相同的 20,000 点 CustomMsg，预热后采样 10 次，并验证反序列化结果。现场旧版本的序列化中位耗时约 81.26 ms，Release 版本约 16.29 ms。之前 Python 基准还包含 Python 消息转换成本，应与原生基准分开解读。

PC2 时间 2026-10-31 的 120 秒连续观测通过，期间加入 RViz 进程负载。1009 条 `/Odometry_nav` 的延迟中位数 49.85 ms、P99 98.42 ms、最大 126.77 ms；552 次 `map -> nav_base` 查询的延迟中位数 121.55 ms、P99 287.52 ms、最大 483.23 ms。反馈和 TF 均没有超过原有阈值，600 条定位状态均为 TRACKING。首轮采样末尾与停机重叠的记录已作废，上述结果来自独立重测。

原始证据位于 PC2 的 `runtime/rmw_release/`：`benchmark_comparison.log`、`monitor_result.json`、`final_validation.log`、`build_receipt.json`。本地工作空间另保存 `runtime/inspection/rmw_release_acceptance_20261031.json`。本次 RViz 仍报告独立的 GLSL 地图着色器链接错误，因此只能确认加入图形进程后的数据链路性能，不能宣称地图渲染已通过验收。

## 可复现构建

在 PC2 工作空间根目录执行。源码目录与 DDS cmake 目录必须对应现场已核实的 Foxy 版本；下面为本次设备的实际路径：

```bash
bash script/g1_build_rmw_release.sh \
  /home/g1test/cyclonedds_ws/src/ros2/rmw_cyclonedds/rmw_cyclonedds_cpp \
  /home/g1test/cyclonedds_ws/install/cyclonedds/lib/cmake/CycloneDDS
```

脚本只编译和安装，不自动切换环境。产物为 `runtime/rmw_release/install`；`build_receipt.json` 记录源码校验和、编译标志和库校验和。旧的 RMW 和系统 DDS 库保持原样。源码、DDS 路径或安装目录改变时，选择新的输出目录，不覆盖其他构建缓存。

## 配置与回退

本次验证通过后，PC2 的 `script/local.env` 使用：

```bash
G1_RMW_SETUP=runtime/rmw_release/install/local_setup.bash
G1_DDS_LIB_DIR=/usr/local/lib
```

不要删除这份安装目录。新终端通过项目环境入口加载配置；旧终端若已导出旧的 `G1_RMW_SETUP`，其环境变量优先于文件，需要先清除旧值：

```bash
unset G1_RMW_SETUP
source script/g1_env.sh
g1_source_environment
python3 script/g1_platform.py rmw
```

输出的 RMW 路径应指向本工作空间的 `runtime/rmw_release/install/lib/librmw_cyclonedds_cpp.so`。实际运行进程的 `/proc/PID/maps` 也应加载该库；仅修改文件不能改变已经运行的进程。桥和导航应通过项目脚本停止后再重新启动。

本轮配置和脚本备份已在用户确认修复成功后清理。如需回退，只把 `G1_RMW_SETUP` 恢复为 `/home/g1test/cyclonedds_ws/install/local_setup.bash`，保留其他现场配置；停止业务并在新终端重新加载环境。无需删除优化版本。

项目构建保护会检测 RMW 路径及校验和的变化。以后重编译应用时，需为 `G1_BUILD_DIR`、`G1_INSTALL_DIR`、`G1_LOG_DIR` 选择三个全新的目录，完成应用构建后再使用新安装目录；不要修改旧构建的保护记录来绕过检查。现有应用的运行验证与重新构建应用是两个独立步骤。

## 基准和现场验证

原生基准不创建 ROS 节点，也不调用机器人接口：

```bash
cmake -S script/rmw_benchmark -B runtime/rmw_release/benchmark \
  -DCMAKE_BUILD_TYPE=Release
cmake --build runtime/rmw_release/benchmark --parallel 2
runtime/rmw_release/benchmark/benchmark
```

现场验证需检查完整启动、持续数据新鲜度、重复停止和启动，以及真实桥的只读通信和后端参数。SSH 没有图形显示时使用：

```bash
G1_ENABLE_RVIZ=0 bash script/g1_navigation.sh start-auto
bash script/g1_navigation_real.sh start
bash script/g1_navigation_real.sh status
```

桥的 `start` 保持运动安全门关闭。上面的验证不包含运动安全门放行、真实导航运动或播放语音。长时间运动测试仍需现场补充，不能从短时间静止测试推断永久无故障。

性能退化时，先记录 `runtime/navigation/logs/livox.log` 中的 `input_age_ms`、`publish_max_ms`、队列长度，并核对实际加载的 RMW。此次修复针对已定位到的发布序列化瓶颈；过载或断流仍应由现有的新鲜度检查和安全门拦截。

重复停止/启动时，ROS 发现列表可能短暂保留已退出节点。导航入口会最多查询 5 次、间隔 1 秒等待它们消失；持久存在的冲突节点和查询失败仍拒绝启动，不会停止外部进程。

发现列表等待修复后的两轮完整启停均通过，日志 `restart_trial_after_graph_fix.log` 中包含 `COLD_START_ATTEMPT_2_PASS`、`COLD_START_ATTEMPT_3_PASS` 和 `AFTER_GRAPH_FIX_RC=0`；其中一轮实际触发旧节点残留等待并恢复。4 项发现列表保护测试及 4 项导航启动模式测试在本地和 PC2 均通过。最终导航、运动桥、语音桥和本次 RViz 进程均已停止，运动安全门全程没有启用。

## 确认成功后的清理

本轮备份、RMW 编译目录、基准可执行产物、临时测试脚本及重复日志已清理。`runtime/rmw_release/install` 是现场正在使用的安装目录，必须保留；构建校验回执、最终性能摘要和成功验收日志仍保留。清理清单见现场 `runtime/rmw_release/cleanup_receipt.json`。需要重新构建时使用上文源码脚本。
