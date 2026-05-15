# jo_navigation — Nav2 Local Framework Architecture

This document describes the Nav2 environment configured by `jo_navigation/config/nav2_local.yaml` and launched by `jo_navigation/launch/navigation_local.launch.py`.

The stack is a local, odometry-frame navigation setup for the Jo tracked robot. It does not use a static global map as the main planning reference: both local and global costmaps are rolling windows in `odom`, fed by filtered LiDAR points and by tracked dynamic obstacles from `onboard_detector` / GLIM.

---

## Table of Contents

1. [Runtime Launch Model](#1-runtime-launch-model)
2. [Frames, Topics, And Global Assumptions](#2-frames-topics-and-global-assumptions)
3. [High-Level Navigation Data Flow](#3-high-level-navigation-data-flow)
4. [Lifecycle Nodes](#4-lifecycle-nodes)
5. [BT Navigator](#5-bt-navigator)
6. [Planner Server](#6-planner-server)
7. [Smoother Server](#7-smoother-server)
8. [Controller Server](#8-controller-server)
9. [Velocity Smoother](#9-velocity-smoother)
10. [Local Costmap](#10-local-costmap)
11. [Global Costmap](#11-global-costmap)
12. [DynamicObstacleLayer](#12-dynamicobstaclelayer)
13. [Behavior Server And Recoveries](#13-behavior-server-and-recoveries)
14. [Waypoint Follower](#14-waypoint-follower)
15. [Map Server And Map Saver](#15-map-server-and-map-saver)
16. [Integration With onboard_detector And GLIM](#16-integration-with-onboard_detector-and-glim)
17. [Tuning Reference](#17-tuning-reference)
18. [Debugging Guide](#18-debugging-guide)

---

## 1. Runtime Launch Model

`navigation_local.launch.py` is the launch entry point for this configuration. By default it loads:

- `controller_server`
- `smoother_server`
- `planner_server`
- `behavior_server`
- `bt_navigator`
- `waypoint_follower`
- `velocity_smoother`
- `lifecycle_manager_navigation`
- optionally `rviz2` when `rviz:=true`

The launch file supports two execution modes:

- **Non-composed mode** (`use_composition:=False`, default): each Nav2 server is launched as a separate process.
- **Composed mode** (`use_composition:=True`): servers are loaded as composable nodes into `nav2_container`.

The lifecycle manager owns activation of:

```text
controller_server
smoother_server
planner_server
behavior_server
bt_navigator
waypoint_follower
velocity_smoother
```

`navigation_local.launch.py` uses `nav2_common.launch.RewrittenYaml` to rewrite parameters at launch time. This matters for `use_sim_time`: although `nav2_local.yaml` sets many nodes to `use_sim_time: True`, the launch argument defaults to:

```text
use_sim_time:=false
```

So, when launched normally, runtime `use_sim_time` is false unless the user explicitly passes `use_sim_time:=true`.

Velocity topics are remapped as:

```text
controller_server output: cmd_vel      → cmd_vel_nav
velocity_smoother input: cmd_vel       → cmd_vel_nav
velocity_smoother output: cmd_vel_smoothed → /cmd_vel
```

Therefore the controller never commands `/cmd_vel` directly. It commands `cmd_vel_nav`; the velocity smoother clamps and republishes the final command on `/cmd_vel`.

---

## 2. Frames, Topics, And Global Assumptions

The local navigation stack is built around the odometry frame:

| Item | Value |
|---|---|
| Global planning frame | `odom` |
| Local costmap frame | `odom` |
| Robot base frame | `base_link` |
| Odometry topic | `/odometry/filtered` |
| Filtered LiDAR obstacle cloud | `/velodyne_points_filtered` |
| Tracked dynamic obstacles | `/onboard_detector/tracked_dynamic_obstacles` |
| GLIM rejection obstacles | `/glim_ros/rejection_obstacles` |
| Final velocity command | `/cmd_vel` |

Because both costmaps use `global_frame: odom` and `rolling_window: true`, the system behaves as a **local navigation environment**, not a map-anchored global navigation stack. The planner still has a "global" costmap, but that global costmap is a larger rolling local window around the robot.

This is appropriate when:

- localization is provided by local odometry / EKF;
- obstacle information is live and local;
- the robot is not relying on a prebuilt static map;
- dynamic avoidance is more important than long-horizon map planning.

---

## 3. High-Level Navigation Data Flow

```text
RViz / action client goal
        │
        ▼
bt_navigator
        │
        ├─ planner_server
        │     └─ NavfnPlanner / A* over global rolling costmap
        │
        ├─ smoother_server
        │     └─ SimpleSmoother, available to BT requests
        │
        └─ controller_server
              └─ GracefulController follows path using local rolling costmap
                    │
                    ▼
                cmd_vel_nav
                    │
                    ▼
              velocity_smoother
                    │
                    ▼
                  /cmd_vel
```

Both local and global costmaps receive:

```text
/velodyne_points_filtered
    → static / geometric obstacle layer

/onboard_detector/tracked_dynamic_obstacles
    → jo_sim::DynamicObstacleLayer

/glim_ros/rejection_obstacles
    → jo_sim::DynamicObstacleLayer fallback / supplementary obstacles
```

The key architectural idea is that obstacle points and dynamic obstacle objects are treated separately:

- point clouds mark ordinary occupied space;
- tracked obstacles add a semantic, velocity-aware footprint and risk region;
- GLIM rejection obstacles can preserve risk where onboard tracking temporarily drops an object, but with reduced velocity inflation.

---

## 4. Lifecycle Nodes

The Nav2 servers are lifecycle-managed. With `autostart:=true` the lifecycle manager automatically configures and activates all listed nodes.

The lifecycle manager does **not** include `map_server` or `map_saver` in `navigation_local.launch.py`. They have parameters in `nav2_local.yaml`, but they are not launched by this local navigation launch file.

Practical consequence:

- local navigation does not wait for a map server;
- `map_server.yaml_filename` being empty is not a blocker in this launch path;
- costmaps are populated from live rolling sensor data.

---

## 5. BT Navigator

`bt_navigator` is configured with:

| Parameter | Value | Meaning |
|---|---:|---|
| `global_frame` | `odom` | Frame used by navigation actions |
| `robot_base_frame` | `base_link` | Robot frame |
| `odom_topic` | `/odometry/filtered` | Source for odometry feedback |
| `bt_loop_duration` | `10` ms | Behavior tree tick period |
| `default_server_timeout` | `20` s | Timeout used when waiting for action servers |

`nav2_local.yaml` does not explicitly set a custom `default_nav_to_pose_bt_xml` / behavior tree XML parameter. Therefore, when a goal does not specify a custom behavior tree, `bt_navigator` follows Nav2's default behavior for the installed distribution.

The package nevertheless contains two behavior trees:

- `behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`
- `behavior_trees/navigate_dynamic.xml`

They are useful when explicitly selected by a launch/action configuration, but they are not bound directly by `nav2_local.yaml`.

### Provided Replanning And Recovery Tree

`navigate_to_pose_w_replanning_and_recovery.xml` contains a `NavigateToPoseWReplanningAndRecovery` tree with:

- planner, controller, goal checker and progress checker selectors;
- `ComputePathToPose` under a `RateController` at 1 Hz;
- contextual global costmap clearing if planning fails and recovery is considered useful;
- `FollowPath` with contextual local costmap clearing if control fails and recovery is considered useful;
- global recovery sequence with:
  - clear local costmap;
  - clear global costmap;
  - spin;
  - wait;
  - backup.

So yes, this package's provided XML contains both:

```text
local_costmap/clear_entirely_local_costmap
global_costmap/clear_entirely_global_costmap
```

### Provided Dynamic Tree

`navigate_dynamic.xml` is a more dynamic-obstacle-oriented tree. Compared with the standard replanning tree, it:

- replans at 2 Hz instead of 1 Hz;
- checks `IsPathValid` before `FollowPath`;
- preempts path following if the path becomes invalid, allowing replanning around newly lethal dynamic obstacles.

This file is conceptually aligned with moving-obstacle avoidance, but again it must be explicitly selected to be used.

---

## 6. Planner Server

`planner_server` uses one planner plugin:

```yaml
planner_plugins: ["GridBased"]

GridBased:
  plugin: "nav2_navfn_planner::NavfnPlanner"
  tolerance: 0.1
  use_astar: true
  allow_unknown: true
```

The planner is therefore Navfn operating in A* mode over the global rolling costmap.

Important behavior:

- `expected_planner_frequency: 8.0` declares the expected planning rate.
- `planner_patience: 15.0` allows relatively long planning attempts before failure.
- `allow_partial_planning: True` permits partial plans when a full path cannot be generated.
- `allow_unknown: true` allows paths through unknown cells.

Because the global costmap is rolling and `track_unknown_space: true`, the planner can operate in local unknown space while still respecting live obstacles and dynamic risk regions.

---

## 7. Smoother Server

The smoother server exposes one smoother:

```yaml
smoother_plugins: ["simple_smoother"]

simple_smoother:
  plugin: "nav2_smoother::SimpleSmoother"
  tolerance: 1.0e-10
  max_its: 1000
  do_refinement: True
```

This provides a Nav2 smoothing action server. Whether it is actually used depends on the behavior tree or client request. The provided `navigate_to_pose_w_replanning_and_recovery.xml` shown in this package does not explicitly include a `SmoothPath` action, so planning can work without the smoother being invoked by that tree.

---

## 8. Controller Server

The controller server runs at:

```yaml
controller_frequency: 25.0
```

It uses:

```yaml
controller_plugins: ["FollowPath"]
FollowPath:
  plugin: nav2_graceful_controller::GracefulController
```

The Graceful Controller is tuned to prefer forward, smooth path tracking while still allowing limited reverse motion when necessary.

### Progress Checker

```yaml
required_movement_radius: 0.5
movement_time_allowance: 10.0
```

The robot must move at least 0.5 m within 10 s. If not, Nav2 considers progress blocked and can trigger controller recovery.

### Goal Checker

```yaml
xy_goal_tolerance: 0.25
yaw_goal_tolerance: 0.45
stateful: True
```

The robot is considered at the goal when it is within 25 cm and about 0.45 rad of the target orientation. Stateful mode means the checker can retain goal-checking state across ticks instead of treating every tick as a fully independent test.

### GracefulController Behavior

Relevant parameters:

| Parameter | Value | Effect |
|---|---:|---|
| `min_lookahead` | 0.8 | Minimum lookahead distance |
| `max_lookahead` | 2.6 | Maximum lookahead distance |
| `initial_rotation` | true | Rotate toward path before translating |
| `initial_rotation_threshold` | 0.55 | Heading error threshold for initial rotation |
| `prefer_final_rotation` | false | Avoid aggressive final spins for exact yaw |
| `allow_backward` | true | Controller may request reverse motion |
| `v_linear_min` | 0.05 | Minimum commanded forward speed from controller |
| `v_linear_max` | 0.65 | Maximum commanded forward speed from controller |
| `v_angular_max` | 1.1 | Maximum angular speed from controller |
| `slowdown_radius` | 1.4 | Slowdown region near goal |

Although `allow_backward: true`, reverse motion is strongly limited later by the velocity smoother:

```yaml
min_velocity: [-0.06, 0.0, -1.0]
```

So reverse is available as an emergency / maneuvering behavior, but it is capped to a very low linear speed.

---

## 9. Velocity Smoother

`velocity_smoother` receives `cmd_vel_nav` and publishes `/cmd_vel`.

Parameters:

| Parameter | Value | Meaning |
|---|---:|---|
| `smoothing_frequency` | 25 Hz | Same nominal rate as controller |
| `feedback` | `OPEN_LOOP` | Smoothing does not close the loop on measured velocity |
| `max_velocity` | `[0.65, 0.0, 1.1]` | Max x, y, yaw velocity |
| `min_velocity` | `[-0.06, 0.0, -1.0]` | Reverse and negative yaw limits |
| `max_accel` | `[1.5, 0.0, 2.4]` | Acceleration limits |
| `max_decel` | `[-1.8, 0.0, -2.8]` | Deceleration limits |
| `velocity_timeout` | 1.0 s | Stop output if command stream times out |

This stage is the final motion safety/comfort gate before the robot base.

---

## 10. Local Costmap

The local costmap is a 20 m × 20 m rolling window in `odom`:

| Parameter | Value |
|---|---:|
| `update_frequency` | 20 Hz |
| `publish_frequency` | 6 Hz |
| `resolution` | 0.05 m |
| `width` / `height` | 20 m / 20 m |
| `robot_radius` | 0.52 m |
| `rolling_window` | true |

Plugins are applied in order:

```yaml
plugins: ["voxel_layer", "dynamic_obstacle_layer", "inflation_layer"]
```

### VoxelLayer

The local costmap uses `nav2_costmap_2d::VoxelLayer` on:

```text
/velodyne_points_filtered
```

Configuration:

- `data_type: PointCloud2`
- marking enabled;
- clearing enabled;
- obstacle range 10 m;
- raytrace range 10 m;
- max obstacle height 2 m;
- voxel Z resolution 0.05 m;
- 16 z voxels.

This layer marks ordinary occupied space from GLIM-filtered LiDAR points and raytraces clearing through free space.

### DynamicObstacleLayer

The custom `jo_sim::DynamicObstacleLayer` runs after the voxel layer and before inflation. This order is important:

1. VoxelLayer marks geometric obstacles.
2. DynamicObstacleLayer writes tracked obstacle boxes and velocity-risk regions.
3. InflationLayer inflates the combined result.

### InflationLayer

```yaml
inflation_radius: 1.2
cost_scaling_factor: 3.0
```

This inflates both static/voxel obstacles and lethal dynamic obstacle boxes.

---

## 11. Global Costmap

The global costmap is also rolling in `odom`, but larger:

| Parameter | Value |
|---|---:|
| `update_frequency` | 8 Hz |
| `publish_frequency` | 2 Hz |
| `resolution` | 0.05 m |
| `width` / `height` | 40 m / 40 m |
| `robot_radius` | 0.52 m |
| `rolling_window` | true |
| `track_unknown_space` | true |

Plugins are:

```yaml
plugins: ["obstacle_layer", "dynamic_obstacle_layer", "inflation_layer"]
```

The global costmap uses `ObstacleLayer` rather than `VoxelLayer`, but still consumes the same point cloud:

```text
/velodyne_points_filtered
```

The dynamic obstacle layer is configured similarly to the local one, with slightly higher minimum risk cost:

```yaml
dynamic_risk_min_cost: 140
```

This makes the global planner more willing to avoid the predicted front region of moving objects.

---

## 12. DynamicObstacleLayer

`jo_sim::DynamicObstacleLayer` is a custom Nav2 costmap layer implemented in:

```text
jo_navigation/src/dynamic_obstacle_layer.cpp
```

It is exported through:

```text
jo_navigation/costmap_plugins.xml
```

and loaded in `nav2_local.yaml` as:

```yaml
plugin: "jo_sim::DynamicObstacleLayer"
```

### Inputs

The layer always subscribes to:

```text
/onboard_detector/tracked_dynamic_obstacles
```

This topic is a `jo_msgs/msg/ObstacleArray`.

When enabled by config, it also subscribes to:

```yaml
use_glim_rejection_obstacles: True
glim_rejection_obstacles_topic: /glim_ros/rejection_obstacles
```

The GLIM rejection obstacle stream is used as a fallback / supplementary source for objects whose points were removed by GLIM dynamic rejection even if onboard tracking is temporarily missing.

### Freshness Gates

Two timeouts protect the layer from stale data:

| Source | Parameter | Current value |
|---|---|---:|
| onboard detector | `tracking_timeout` | 1.0 s |
| GLIM rejection obstacles | `glim_rejection_timeout` | 0.5 s |

If a source is older than its timeout, that source is ignored for the current costmap update.

### Duplicate Handling Between onboard_detector And GLIM

If onboard detector tracking is valid, the layer builds a set of onboard `track_id`s.

When GLIM rejection obstacles are processed, any GLIM obstacle whose `track_id` is already present in the onboard message is skipped. This gives priority to the tracked onboard obstacle because it has the best semantic/tracking state.

In short:

```text
same track_id in onboard + GLIM → use onboard only
track_id only in GLIM           → use GLIM fallback
```

### Cost Geometry

For every obstacle, the layer computes a padded physical rectangle:

```text
hx = size.x / 2 + obstacle_padding
hy = size.y / 2 + obstacle_padding
hz = size.z / 2 + obstacle_padding
```

With the current config:

```yaml
obstacle_padding: 0.15
```

The physical rectangle is axis-aligned in the costmap frame, not rotated with the obstacle velocity.

For onboard detector obstacles:

```text
physical rectangle → LETHAL_OBSTACLE
```

For GLIM rejection obstacles:

```text
physical rectangle → dynamic_risk_cost
```

This is intentionally less absolute: when onboard tracking is missing, GLIM can still warn Nav2, but it does not create the same hard lethal box unless the obstacle is also present in onboard tracking.

### Velocity Risk Region

If obstacle speed is above:

```yaml
risk_velocity_threshold: 0.2
```

the layer creates an egg/ellipse-like 2D risk region aligned with velocity.

The velocity direction is:

```text
heading = atan2(vy, vx)
```

Coordinates are transformed into the obstacle velocity frame:

```text
u = forward/backward axis along velocity
v = lateral axis
```

The semi-axes are:

```text
front semi-axis = hx + speed * velocity_inflation_k
rear semi-axis  = hx + speed * rear_inflation_k
lateral semi-axis = hy + speed * lateral_inflation_k
```

Current local/global config:

| Parameter | Value |
|---|---:|
| `velocity_inflation_k` | 4.0 |
| `rear_inflation_k` | 0.5 |
| `lateral_inflation_k` | 0.2 |

So the risk region is much longer in front of the moving obstacle, only slightly extended behind it, and only moderately widened laterally.

The region test is:

```text
(u / semi_u)^2 + (v / semi_v)^2 <= 1
```

where `semi_u` is front or rear depending on the sign of `u`.

This makes the shape an asymmetric velocity-aligned ellipse: long in front, shorter behind. In RViz it is visualized with a sphere marker scaled as an elongated flat ellipse.

### Risk Cost Falloff

Inside the velocity risk region, cost is highest near the obstacle and decays toward the ellipse boundary:

```text
falloff = 1 - normalized_distance
```

The back half is additionally reduced:

```text
directional_bias = 1.0   if u >= 0
directional_bias = 0.35  if u < 0
```

Cost is then clamped between:

```text
dynamic_risk_min_cost
dynamic_risk_cost
```

Current values:

| Costmap | `dynamic_risk_cost` | `dynamic_risk_min_cost` |
|---|---:|---:|
| local | 252 | 120 |
| global | 252 | 140 |

`252` is high but below Nav2's `LETHAL_OBSTACLE` value `254`.

### GLIM Velocity Scaling

For GLIM rejection obstacles:

```yaml
glim_rejection_velocity_scale: 0.75
```

This scales the velocity-based front/rear/lateral inflation. GLIM-only obstacles still create a caution region, but shorter than the region created from onboard tracking.

### Clearing Behavior

At each update, the layer remembers the previous update bounds. Before writing new costs, it clears the cells it wrote previously.

Important detail: it does not clear cells that are already `LETHAL_OBSTACLE`. Those are assumed to belong to the VoxelLayer / static geometric obstacle representation.

The layer is also:

```cpp
isClearable() → false
```

so standard costmap clear recoveries do not treat it as a normal clearable layer.

### Marker Output

The layer publishes:

```text
dynamic_obstacle_footprints
```

relative to the costmap node namespace. Typical resolved topics are:

```text
/local_costmap/dynamic_obstacle_footprints
/global_costmap/dynamic_obstacle_footprints
```

Markers:

- yellow cube: padded physical obstacle box;
- orange flat sphere: velocity risk ellipse.

---

## 13. Behavior Server And Recoveries

The behavior server exposes:

```yaml
behavior_plugins: ["spin", "backup", "drive_on_heading", "assisted_teleop", "wait"]
```

Configured plugins:

- `nav2_behaviors::Spin`
- `nav2_behaviors::BackUp`
- `nav2_behaviors::DriveOnHeading`
- `nav2_behaviors::Wait`
- `nav2_behaviors::AssistedTeleop`

Behavior server parameters:

| Parameter | Value |
|---|---:|
| `cycle_frequency` | 20 Hz |
| `simulate_ahead_time` | 3.0 s |
| `max_rotational_vel` | 0.8 |
| `min_rotational_vel` | 0.2 |
| `rotational_acc_lim` | 2.0 |

Costmap topics used by behavior server:

```text
local_costmap/costmap_raw
local_costmap/published_footprint
```

Costmap clearing is not listed as a behavior plugin because clearing is a Nav2 BT action, not one of the behavior server plugins. The provided behavior tree XML files call the clear services directly:

```text
local_costmap/clear_entirely_local_costmap
global_costmap/clear_entirely_global_costmap
```

---

## 14. Waypoint Follower

The waypoint follower runs at:

```yaml
loop_rate: 20
stop_on_failure: false
```

It uses:

```yaml
wait_at_waypoint:
  plugin: "nav2_waypoint_follower::WaitAtWaypoint"
  enabled: True
  waypoint_pause_duration: 200
```

So waypoint missions continue after individual waypoint failures and pause briefly at each waypoint.

---

## 15. Map Server And Map Saver

`nav2_local.yaml` contains parameter blocks for `map_server` and `map_saver`, but `navigation_local.launch.py` does not launch them as part of the lifecycle node list.

`map_server.yaml_filename` is empty:

```yaml
yaml_filename: ""
```

This confirms that the local navigation setup is not centered on a preloaded occupancy map.

---

## 16. Integration With onboard_detector And GLIM

This navigation config expects the perception side to provide two different outputs.

### Filtered LiDAR Cloud

Both costmaps subscribe to:

```text
/velodyne_points_filtered
```

This is the geometric point cloud used by:

- local `VoxelLayer`;
- global `ObstacleLayer`.

In the current system this topic is produced downstream of GLIM dynamic rejection. Dynamic object points may already have been removed from this cloud.

### Tracked Dynamic Obstacles

The custom dynamic layer subscribes to:

```text
/onboard_detector/tracked_dynamic_obstacles
```

This topic carries obstacle pose, size, velocity, acceleration, age, track id, and status.

`DynamicObstacleLayer` currently does not branch on obstacle status. It treats every obstacle present in this topic as an object to write into the costmap. Velocity controls whether the elongated risk region is added.

### GLIM Rejection Obstacles

The layer also listens to:

```text
/glim_ros/rejection_obstacles
```

This helps with the failure mode where GLIM has removed points because an object was inside a recent rejection volume, but onboard_detector temporarily does not publish the object as tracked/dynamic.

In that case:

- the ordinary point cloud may be missing the object;
- onboard detector may not publish a lethal tracked obstacle;
- GLIM rejection obstacles can still add a caution region into the costmap.

The implementation gives onboard tracking priority when both sources report the same `track_id`.

---

## 17. Tuning Reference

### Robot Gets Too Close To Moving Objects

Relevant parameters:

| Parameter | Increase to... |
|---|---|
| `velocity_inflation_k` | extend the front risk region |
| `lateral_inflation_k` | widen the side risk region |
| `dynamic_risk_min_cost` | make the ellipse boundary less attractive |
| `dynamic_risk_cost` | make the high-risk core more expensive |
| `inflation_radius` | increase general clearance around lethal cells |

### Robot Refuses To Pass Behind Moving Objects

Relevant parameters:

| Parameter | Decrease to... |
|---|---|
| `rear_inflation_k` | reduce the rear half of the velocity ellipse |
| `dynamic_risk_min_cost` | reduce residual cost at the ellipse boundary |
| `inflation_radius` | reduce global inflation around lethal obstacle boxes |

### GLIM-Only Obstacles Are Too Aggressive

Relevant parameters:

| Parameter | Effect |
|---|---|
| `glim_rejection_velocity_scale` | lower values shorten GLIM-only velocity ellipses |
| `glim_rejection_timeout` | lower values make GLIM-only obstacles expire sooner |
| `dynamic_risk_min_cost` | lowers the minimum risk cost for all dynamic ellipses |

### Dynamic Obstacles Disappear Too Quickly

Relevant parameters:

| Parameter | Effect |
|---|---|
| `tracking_timeout` | increases how long onboard detector obstacles remain valid |
| `glim_rejection_timeout` | increases how long GLIM rejection obstacles remain valid |

Remember that increasing these can also preserve stale obstacle costs longer.

### Controller Is Too Willing To Reverse

Relevant parameters:

| Parameter | Effect |
|---|---|
| `FollowPath.allow_backward` | disables backward commands if set false |
| `velocity_smoother.min_velocity[0]` | current value `-0.06` tightly limits reverse speed |
| `initial_rotation` | when true, favors rotate-then-forward behavior |

### Planner Enters Unknown Space Too Freely

Relevant parameters:

| Parameter | Effect |
|---|---|
| `GridBased.allow_unknown` | if false, planner avoids unknown cells |
| `global_costmap.track_unknown_space` | controls whether unknown space is represented |

---

## 18. Debugging Guide

### Check Whether Nav2 Is Using Sim Time

Do not trust only `nav2_local.yaml`. Check the launch command:

```text
ros2 launch jo_navigation navigation_local.launch.py use_sim_time:=true
```

Without that override, the launch default is `false`.

### Check Costmap Inputs

Use:

```text
ros2 topic hz /velodyne_points_filtered
ros2 topic hz /onboard_detector/tracked_dynamic_obstacles
ros2 topic hz /glim_ros/rejection_obstacles
```

If `/velodyne_points_filtered` is missing, ordinary obstacle marking will be empty.

If `/onboard_detector/tracked_dynamic_obstacles` is missing, dynamic lethal boxes from onboard tracking will be absent.

If `/glim_ros/rejection_obstacles` is missing, the GLIM fallback path is inactive even if enabled in config.

### Check Dynamic Layer Markers

Look for:

```text
/local_costmap/dynamic_obstacle_footprints
/global_costmap/dynamic_obstacle_footprints
```

Yellow boxes show padded physical obstacle regions. Orange ellipses show velocity risk regions.

### Check Whether Clear Recoveries Exist

The provided BT XML files call:

```text
local_costmap/clear_entirely_local_costmap
global_costmap/clear_entirely_global_costmap
```

If a different BT is loaded at runtime, inspect that XML or the action goal's `behavior_tree` field.

### Check Why A Moving Object Is Not Lethal

The physical rectangle is lethal only for onboard detector obstacles. GLIM-only rejection obstacles write high risk cost, not lethal cost.

Therefore check:

1. Is the object present in `/onboard_detector/tracked_dynamic_obstacles`?
2. Is the message fresh relative to `tracking_timeout`?
3. Is the object only present in `/glim_ros/rejection_obstacles`?
4. Is the `track_id` duplicated between onboard and GLIM, causing the GLIM copy to be skipped?

### Check Why The Risk Ellipse Is Missing

The ellipse is only added when:

```text
speed > risk_velocity_threshold
```

With the current config this means speed must exceed `0.2 m/s`. Slower obstacles still get the physical rectangle, but not the velocity-elongated risk field.

