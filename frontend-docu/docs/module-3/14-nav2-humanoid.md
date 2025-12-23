---
id: nav2-humanoid
title: Nav2 for Humanoids
sidebar_label: Chapter 14 Nav2 Humanoid
---

# Nav2 for Humanoids

## Overview

This chapter covers Nav2 path planning specifically tuned for bipedal humanoid balance and movement, including costmaps, behavior trees, and bipedal footstep planning. Humanoid navigation requires specialized approaches due to balance constraints and unique locomotion patterns.

## Key Topics

- Nav2 configuration for bipedal robots
- Humanoid-specific costmaps
- Behavior trees for navigation
- Footstep planning algorithms
- Balance-aware path planning

## Humanoid Navigation Architecture

The navigation system for bipedal robots requires special considerations for balance and stability:

- **Stability-aware path planning**: Paths must account for the robot's center of mass
- **Footstep planning**: Precise placement of feet for stable locomotion
- **Balance recovery**: Behaviors to maintain or regain balance during navigation

### Nav2 Configuration for Bipedal Movement

```yaml
# Nav2 configuration for humanoid robot navigation
bt_navigator:
  ros__parameters:
    # Behavior tree for humanoid navigation
    default_bt_xml_filename: "humanoid_navigator_bt.xml"
    enable_groot_monitoring: True
    use_sim_time: True

controller_server:
  ros__parameters:
    # Controller for humanoid movement
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.05
    min_y_velocity_threshold: 0.05
    min_theta_velocity_threshold: 0.1
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    controller_plugins: ["FollowPath"]

    # Humanoid-specific controller
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 25
      model_dt: 0.05
      batch_size: 1000
      vx_std: 0.3
      vy_std: 0.1
      wz_std: 0.3
      vx_max: 0.5
      vx_min: -0.2
      vy_max: 0.3
      wz_max: 0.5
      # Humanoid-specific constraints
      balance_constraint_weight: 10.0
      footstep_constraint_weight: 5.0

local_costmap:
  ros__parameters:
    update_frequency: 10.0
    publish_frequency: 5.0
    global_frame: odom
    robot_base_frame: base_link
    use_sim_time: True
    rolling_window: true
    width: 6
    height: 6
    resolution: 0.05
    plugins: [
      "obstacles_layer",
      "inflation_layer",
      "footstep_layer",
      "stability_layer"
    ]

    obstacles_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      enabled: True
      cost_scaling_factor: 3.0
      inflation_radius: 0.55

    footstep_layer:
      plugin: "custom::FootstepLayer"
      enabled: True
      # Humanoid-specific parameters
      foot_separation: 0.3
      step_height: 0.1
      max_step_length: 0.4

    stability_layer:
      plugin: "custom::StabilityLayer"
      enabled: True
      # Balance-related parameters
      com_tracking_weight: 10.0
      zmp_margin: 0.05

global_costmap:
  ros__parameters:
    global_frame: map
    robot_base_frame: base_link
    update_frequency: 1.0
    publish_frequency: 1.0
    resolution: 0.05
    use_sim_time: True
    plugins: [
      "static_layer",
      "obstacles_layer",
      "inflation_layer",
      "stability_layer"
    ]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
      map_subscribe_transient_local: True

    obstacles_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      enabled: True
      observation_sources: scan
      scan:
        topic: /laser_scan
        max_obstacle_height: 2.0
        clearing: True
        marking: True
        data_type: "LaserScan"

    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      enabled: True
      cost_scaling_factor: 3.0
      inflation_radius: 0.55

    stability_layer:
      plugin: "custom::StabilityLayer"
      enabled: True
      # Balance-related parameters for global planning
      com_tracking_weight: 5.0
      zmp_margin: 0.1
```

## Costmap Configuration for Humanoids

Humanoid robots require specialized costmap layers to account for balance and stability:

```yaml
# Additional humanoid-specific costmap configuration
local_costmap:
  plugins:
    - {name: obstacles_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    - {name: footstep_layer, type: "custom::FootstepLayer"}
    - {name: stability_layer, type: "custom::StabilityLayer"}

global_costmap:
  plugins:
    - {name: static_layer, type: "nav2_costmap_2d::StaticLayer"}
    - {name: obstacles_layer, type: "nav2_costmap_2d::ObstacleLayer"}
    - {name: inflation_layer, type: "nav2_costmap_2d::InflationLayer"}
    - {name: stability_layer, type: "custom::StabilityLayer"}
```

## Behavior Trees for Humanoid Navigation

Behavior trees in Nav2 can be customized for humanoid-specific navigation behaviors:

```xml
<!-- humanoid_navigator_bt.xml -->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">
    <PipelineSequence name="NavigateWithReplanning">
      <RateController hz="1.0">
        <RecoveryNode number_of_retries="6" name="ComputeAndTryController">
          <PipelineSequence name="ComputeAndExecuteController">
            <ControllerSelector input_path="goal_path" output_path="selected_path"/>
            <PathUnwrapper path="{selected_path}"/>
            <HumanoidController path="{selected_path}"/>
          </PipelineSequence>
          <RecoveryNode number_of_retries="2" name="HumanoidSpin">
            <HumanoidSpin spin_dist="1.57"/>
          </RecoveryNode>
        </RecoveryNode>
      </RateController>
      <ReactiveSequence name="CheckGoalReached">
        <GoalReached goal="current_goal" path="goal_path"/>
        <ComputePathToPose goal="{current_goal}" path="goal_path" planner_id="GridBased"/>
      </ReactiveSequence>
    </PipelineSequence>
  </BehaviorTree>
</root>
```

## Footstep Planning for Bipedal Navigation

Footstep planning is critical for humanoid navigation:

```python
# Python example for humanoid footstep planning
import numpy as np
from geometry_msgs.msg import Point

class HumanoidFootstepPlanner:
    def __init__(self):
        self.foot_separation = 0.3  # Distance between feet
        self.step_height = 0.1      # Height to lift foot during stepping
        self.max_step_length = 0.4  # Maximum forward step length

    def plan_footsteps(self, path, robot_pose):
        """
        Plan stable footsteps for humanoid navigation
        """
        footsteps = []

        # Calculate footsteps based on path and robot constraints
        current_pose = robot_pose
        for i in range(len(path) - 1):
            step = self.calculate_next_step(current_pose, path[i+1])
            if self.is_stable_footstep(step):
                footsteps.append(step)
                current_pose = step.end_pose

        return footsteps

    def calculate_next_step(self, current_pose, target_point):
        """
        Calculate the next footstep based on current pose and target
        """
        # Calculate step direction and distance
        dx = target_point.x - current_pose.position.x
        dy = target_point.y - current_pose.position.y
        distance = np.sqrt(dx**2 + dy**2)

        # Limit step size for stability
        if distance > self.max_step_length:
            dx = dx * self.max_step_length / distance
            dy = dy * self.max_step_length / distance

        # Calculate foot position based on current stance
        # (left or right foot based on step sequence)
        foot_position = Point()
        foot_position.x = current_pose.position.x + dx
        foot_position.y = current_pose.position.y + dy
        foot_position.z = current_pose.position.z

        return foot_position

    def is_stable_footstep(self, footstep):
        """
        Check if footstep maintains robot stability
        """
        # Check if footstep is within support polygon
        # and maintains center of mass within stable region
        return True  # Simplified for example
```