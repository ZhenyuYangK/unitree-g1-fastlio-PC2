# P1 PC2 依赖与构建约定

目标为 Ubuntu 20.04 / aarch64 / ROS 2 Foxy / Python 3.8。检查主机的 Humble 产物不能用于 PC2。

配置 `script/local.env`（模板 `script/local.env.example`），然后在 PC2 执行：

```bash
bash script/g1_build.sh
bash script/g1_check.sh > pc2-dependencies.json
```

环境入口只加载显式配置的 PCL、Unitree ROS2 与 DDS/RMW underlay。空路径表示使用系统包；已填写却不存在的路径直接报错。工作空间使用 `install_pc2/local_setup.bash`，不会加载旧 `install/setup.bash` 中冻结的原机器 underlay 链。默认构建目录为 `build_pc2`、`install_pc2`、`log_pc2`；改变路径、架构、Python、编译器、PCL 或 DDS/RMW 后配置全新的目录，脚本不会删除旧产物。

| 层 | PC2 必需依赖 | 核验方式 |
| --- | --- | --- |
| 工具链 | CMake >= 3.16、GCC 的 C++17、make、colcon、Python 3.8、PyYAML、iproute2、util-linux | inventory、构建与离线脚本测试 |
| 算法 | Eigen3、PCL（common/io/filters/registration）、OpenMP、Python 开发头、APR | CMake find_package / pkg-config |
| ROS | rclcpp、rclpy、rosidl generators/runtime、std/geometry/nav/sensor/visualization messages、tf2、pcl_ros、pcl_conversions、ament_cmake_auto | 包清单及 CMake |
| 导航 | Foxy Nav2 的 map_server/controller/planner/recoveries/bt_navigator/lifecycle_manager、DWB、NavFn；启用图形时需要 RViz2 | `g1_check.sh`，启动时生命周期检查 |
| 雷达 | 仓库 Livox SDK2 1.3.3 源码与 livox_ros_driver2 | 本机编译安装到 install_pc2/livox_sdk2；不用原机器二进制 |
| G1 控制 | 仓库 unitree_api 接口与 ROS2 example 的 g1/common/nlohmann 头文件 | vendor_manifest.json 中 SHA-256；CMake 相对路径或 UNITREE_ROS2_EXAMPLE_INCLUDE cache |
| DDS/RMW | 与 Foxy 和现场系统一致的 rmw_cyclonedds_cpp + CycloneDDS | 实际 ldd 解析路径、版本、ELF 架构、SHA-256；自定义组合必须成对配置 |

`vendor_manifest.json` 记录已知来源、固定提交、本地版本证据及 68 个必需厂商文件的 SHA-256。未随原始快照提供的上游版本不补写猜测值。`bash script/g1_check.sh --offline` 在无 Foxy 主机上只读核验这些文件并输出本机信息。

G1 桥和 probes 使用 `unitree_api` 的 Request/Response 通道，**不链接原生 Unitree SDK2 静态库，也不注入其 bundled DDS 库**。`third_party/unitree_sdk2_native/` 的 aarch64 与 x86_64 已解压库作为可选历史资产分别记录校验和；当前构建不需要缺失的压缩归档和 B2/H2 示例。显式需要原生 SDK 时使用 `bash script/restore_vendor_libraries.sh --arch aarch64 --check` 校验；恢复模式只接受本地压缩包内单个、尺寸及 SHA-256 都匹配的库，不联网取未知二进制。

release 调试须同时指定 `G1_RMW_SETUP` 与 `G1_DDS_LIB_DIR`，自检要求实际加载的 RMW 位于该 setup 下、ldd 实际使用的 DDS 位于配置目录。`g1_check.sh` 会输出 RMW package.xml 版本、两个库的 SHA-256 和链接解析结果；版本兼容性最终还须通过 PC2 上建图、人工定位与收发测试确认。
