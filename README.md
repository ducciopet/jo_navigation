# jo_navigation

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-34C759?style=flat-square&logo=ros)](https://docs.ros.org/)
[![Nav2](https://img.shields.io/badge/Nav2-Latest-4285F4?style=flat-square&logo=ros)](https://navigation.ros.org/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?style=flat-square&logo=ubuntu)](https://ubuntu.com/)

ROS 2 navigation and localization package for the Jo tracked robot platform. Provides localization via sensor fusion (EKF) and autonomous navigation using the Nav2 stack, with support for both local (odometry-based) and global (GPS-based) operation.


<div align="center">
  <img src="res/navigation.gif" alt="Navigation Stack" height="500"/>
</div>


## Package Contents

- **Launch files** — for localization and navigation in local and global modalities
- **EKF configurations** — tuned for odometry-only and GPS-fused localization
- **Nav2 configuration** — costmap, planner, and controller parameters for the tracked platform
- **RViz configs** — pre-configured layouts for monitoring the perception and navigation stack



## Localization

The localization stack is responsible for estimating the robot's pose and publishing the relevant TF transforms. Two modes are available depending on the available sensor inputs.

> **Note:** GLIM (the SLAM system used in Jo) does not publish on `/tf` by default, so the localization launch files are required to obtain a stable fixed frame in RViz and for Nav2 to function correctly.

### Local odometry

Runs a single EKF node that fuses wheel odometry and IMU data to estimate the `odom` → `base_link` transform. 

```bash
ros2 launch jo_navigation localization.launch.py
```

Once running, `odom` can be used as the fixed frame in RViz.

### Global localization (GPS-fused)

Runs two EKF instances in tandem:

1. **Local EKF** — estimates `odom` → `base_link` (identical to the local odometry launch)
2. **Global EKF** — fuses GPS data to estimate `map` → `odom`, providing a globally-referenced pose

```bash
ros2 launch jo_navigation localization_gps.launch.py 
```

## Navigation

The navigation stack is built on Nav2 and requires a localization source to be running beforehand. Two configurations mirror the two localization modalities.

### Local navigation

Uses the `odom` frame as the planning reference. Suitable for navigation in a known local area without a global map.

**Requires:** [Local odometry](#local-odometry) to be running.

```bash
ros2 launch jo_navigation navigation_local.launch.py rviz:=true
```

This launches:
- Nav2 stack (planner, controller, costmap nodes)
- RViz configured to display the perception stack, local costmap, and planned path

To command the robot, use the **2D Goal Pose** tool in RViz and click a target on the map. Set the fixed frame to `odom`.

### Global navigation

Uses the `map` frame published by the global EKF. Supports navigation over larger areas with GPS-anchored pose estimates.

**Requires:** [Global localization](#global-localization-gps-fused) to be running.

```bash
ros2 launch jo_navigation navigation_gps.launch.py rviz:=true
```

This is a modified version of the local navigation configuration adapted to consume the `map` frame and the dual-EKF odometry sources.


## Additional Resources

- [Nav2 Documentation](https://navigation.ros.org/)
- [robot_localization Documentation](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
- [ROS 2 TF2](https://docs.ros.org/en/jazzy/Concepts/Intermediate/About-Tf2.html)
- [jo_sim](https://github.com/mlisi1/jo_sim.git)
- [jo_zotac](https://github.com/mlisi1/jo-zotac.git)