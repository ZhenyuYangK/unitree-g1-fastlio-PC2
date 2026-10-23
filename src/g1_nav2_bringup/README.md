# g1_nav2_bringup

Planning-only Nav2 bringup for the current G1 FAST-LIO project.

- global frame: `map`
- odom frame: `camera_init`
- FAST-LIO frame: `body`
- navigation base: `nav_base` (static 180-degree X correction from `body`)
- odometry topic: `/Odometry_nav` (converted from FAST-LIO `/Odometry`)
- local obstacle source: `/cloud_registered_body`
- isolated velocity output: `/cmd_vel_nav`

This package intentionally does not connect velocity output to Unitree SDK2.

`navigation_odometry` rotates pose orientation, body linear/angular velocity and
all twist covariance blocks into `nav_base`. It preserves the sensor timestamp,
parent frame, position and parent-frame pose covariance, and publishes no TF.
The converter and static TF use the same quaternion in `planning_only.launch.py`
(default 180 degrees about X). This assumes co-located origins. A translated
navigation origin requires a lever-arm velocity/covariance model, not a renamed
frame. Recheck signs after changing the lidar mounting or IMU extrinsics.

The launch and configuration target **Foxy**: `nav2_recoveries/recoveries_server`,
singular `goal_checker_plugin`, `default_bt_xml_filename`, and the `SequenceStar`
BT node. The recovery tree uses spin/wait and costmap clearing without backup.
Recovery TF uses `camera_init` and `transform_tolerance`, matching the local map.
Both controller and BT navigator consume `/Odometry_nav`.

The source YAML describes the target limits: 1.0 m/s forward, 0.40 rad/s yaw,
linear acceleration/deceleration 0.20/0.30 m/s², angular acceleration/deceleration
4.0 rad/s², with no lateral or reverse commands. Positive lateral acceleration
parameters support DWB's single zero lateral sample; both lateral velocity bounds
remain zero. Navigation scripts default to the **safe** profile, with a 0.20 m/s
forward cap and the same yaw/acceleration limits.

`tools/generate_nav2_params.py SOURCE OUTPUT --profile safe|target` generates
Foxy parameters. The navigation script also supplies `--bridge-dry`,
`--bridge-real` and `--manifest` to generate matching bridge files and a hash
record. `script/g1_navigation_readiness.py` checks active nodes, the loaded
parameters, actual backend and fresh converted odometry before enabling a bridge.
Changing `G1_NAV_PROFILE` requires stopping and restarting the navigation stack;
reusing a different running profile is rejected. Direct use of source bridge
YAML selects the target limits.

The local P4 acceptance record is `docs/migration/P4_ACCEPTANCE.md`. Host tests use
Humble/x86_64, synthetic inputs and DryRunBackend. Foxy/aarch64 compilation and
real forward/turn signs and navigation remain PC2 acceptance steps.
