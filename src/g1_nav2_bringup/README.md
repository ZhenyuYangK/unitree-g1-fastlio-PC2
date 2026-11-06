# g1_nav2_bringup

Planning-only Nav2 bringup for the current G1 FAST-LIO project.

- global frame: `map`
- odom frame: `camera_init`
- FAST-LIO frame: `body`
- navigation base: `nav_base` (static 180-degree X correction from `body`)
- odometry topic: `/Odometry_nav` (converted from FAST-LIO `/Odometry`)
- local and global obstacle source: `/cloud_registered_body`
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
singular `goal_checker_plugin` and `default_bt_xml_filename`.
The `G1GuardedNavigate` BT action delegates planning and DWB ownership to
`g1_navigation_guard`. A static-only reference planner freezes the original route
budget; travelled distance plus a replacement route may exceed it by at most 10 m.
When no allowed route is executable, navigation stops and waits. A confirmed
one-second blockage requests one Chinese reminder; two continuous clear seconds
release the same goal. No spin or costmap-clearing recovery runs during this wait.
Confirmed obstacle waiting is excluded from task navigation timeouts. Narration,
pause and stop cancel the navigation leg and cannot generate a delayed reminder.
Generated bridge parameters require a fresh navigation permit (0.3 s timeout)
in addition to the existing manual and localization gates.
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

The obstacle-wait software checks run on Foxy/aarch64 using an isolated DDS domain,
synthetic sensors, real NavFn/DWB/BT nodes and a mock speaker. Motion bridge checks
use DryRunBackend. Physical clearance, braking and audible speech still require
on-robot acceptance. Earlier P4 records remain in `docs/migration/P4_ACCEPTANCE.md`.
