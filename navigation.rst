.. _plugins:

Navigation Plugins
##################

There are a number of plugin interfaces for users to create their own custom applications or algorithms with.
Namely, the costmap layer, planner, controller, behavior tree, and behavior plugins.
A list of all known plugins are listed here below for ROS 2 Navigation.
If you know of a plugin, or you have created a new plugin, please consider submitting a pull request with that information.

Costmap Layers
==============

+--------------------------------+----------------------------------+
|            Plugin Name         |       Description                |
+================================+==================================+
| `Voxel Layer`_                 | Maintains persistant             |
+--------------------------------+----------------------------------+
|                                | 3D voxel layer using depth and   |
|                                | laser sensor readings and        |
|                                | raycasting to clear free space   |
+--------------------------------+----------------------------------+
| `Range Layer`_                 | Uses a probabalistic model to    |
|                                | put data from sensors that       |
|                                | publish range msgs on the costmap|
+--------------------------------+----------------------------------+
| `Static Layer`_                | Gets static ``map`` and loads    |
|                                | occupancy information into       |
|                                | costmap                          |
+--------------------------------+----------------------------------+
| `Inflation Layer`_             | Inflates lethal obstacles in     |
|                                | costmap with exponential decay   |
+--------------------------------+----------------------------------+
|  `Obstacle Layer`_             | Maintains persistent 2D costmap  |
|                                | from 2D laser scans with         |
|                                | raycasting to clear free space   |
+--------------------------------+----------------------------------+
| `Spatio-Temporal Voxel Layer`_ | Maintains temporal 3D sparse     |
|                                | volumetric voxel grid with decay |
|                                | through sensor models            |
+--------------------------------+----------------------------------+
| `Non-Persistent Voxel Layer`_  | Maintains 3D occupancy grid      |
|                                | consisting only of the most      |
|                                | sets of measurements             |
+--------------------------------+----------------------------------+

.. _Voxel Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/voxel_layer.cpp
.. _Static Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/static_layer.cpp
.. _Range Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/range_sensor_layer.cpp
.. _Inflation Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/inflation_layer.cpp
.. _Obstacle Layer: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/obstacle_layer.cpp
.. _Spatio-Temporal Voxel Layer: https://github.com/SteveMacenski/spatio_temporal_voxel_layer/
.. _Non-Persistent Voxel Layer: https://github.com/SteveMacenski/nonpersistent_voxel_layer

Costmap Filters
===============

+--------------------+-----------------------------------+
|    Plugin Name     |       Description                 |
+====================+===================================+
| `Keepout Filter`_  | Maintains keep-out/safety zones   |
|                    | and preferred lanes for moving    |
+--------------------+-----------------------------------+
| `Speed Filter`_    | Limits maximum velocity of robot  |
|                    | in speed restriction areas        |
+--------------------+-----------------------------------+

.. _Keepout Filter: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/keepout_filter.cpp
.. _Speed Filter: https://github.com/ros-planning/navigation2/tree/main/nav2_costmap_2d/plugins/costmap_filters/speed_filter.cpp

Controllers
===========

+----------------------------+----------------------------------+-----------------------+
|      Plugin Name           |       Description                | Drivetrain support    |
+============================+==================================+=======================+
|  `DWB Controller`_         | A highly configurable  DWA       | Differential,         |
|                            | implementation with plugin       | Omnidirectional,      |
|                            | interfaces                       | Legged                |
+----------------------------+----------------------------------+-----------------------+
|  `TEB Controller`_         | A MPC-like controller suitable   | **Ackermann**, Legged,|
|                            | for ackermann, differential, and | Omnidirectional,      |
|                            | holonomic robots.                | Differential          |
+----------------------------+----------------------------------+-----------------------+
| `Regulated Pure Pursuit`_  | A service / industrial robot     | **Ackermann**, Legged,|
|                            | variation on the pure pursuit    | Differential          |
|                            | algorithm with adaptive features.|                       |
+----------------------------+----------------------------------+-----------------------+
| `Rotation Shim Controller`_| A "shim" controller to rotate    | Differential, Omni,   |
|                            | to path heading before passing   | model rotate in place |
|                            | to main controller for  tracking.|                       |
+----------------------------+----------------------------------+-----------------------+

.. _DWB Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_dwb_controller
.. _TEB Controller: https://github.com/rst-tu-dortmund/teb_local_planner
.. _Regulated Pure Pursuit: https://github.com/ros-planning/navigation2/tree/main/nav2_regulated_pure_pursuit_controller
.. _Rotation Shim Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_rotation_shim_controller

Planners
========

+---------------------------+------------------------------+---------------------+
| Plugin Name               |       Description            | Drivetrain support  |
+===========================+==============================+=====================+
|  `NavFn Planner`_         | A navigation function        | Differential,       |
|                           | using A* or Dijkstras        | Omnidirectional,    |
|                           | expansion, assumes 2D        | Legged              |
|                           | holonomic particle           |                     |
+---------------------------+------------------------------+---------------------+
| `SmacPlannerHybrid`_      | A SE2 Hybrid-A*              | **Ackermann**,      |
|  (formerly `SmacPlanner`) | implementation using either  | Differential,       |
|                           | Dubin or Reeds-shepp motion  | Omnidirectional,    |
|                           | models with smoother and     | Legged              |
|                           | multi-resolution query.      |                     |
|                           | Cars, car-like, and          |                     |
|                           | ackermann vehicles.          |                     |
|                           | Kinematically feasible.      |                     |
+---------------------------+------------------------------+---------------------+
|  `SmacPlanner2D`_         | A 2D A* implementation       | Differential,       |
|                           | Using either 4 or 8          | Omnidirectional,    |
|                           | connected neighborhoods      | Legged              |
|                           | with smoother and            |                     |
|                           | multi-resolution query       |                     |
+---------------------------+------------------------------+---------------------+
|  `SmacPlannerLattice`_    | An implementation of State   | Differential,       |
|                           | Lattice Planner using        | Omnidirectional,    |
|                           | pre-generated minimum control| Ackermann,          |
|                           | sets for kinematically       | Legged,             |
|                           | feasible planning with any   | Arbitrary / Custom  |
|                           | type of vehicle imaginable.  |                     |
|                           | Includes generator script for|                     |
|                           | Ackermann, diff, omni, and   |                     |
|                           | legged robots.               |                     |
+---------------------------+------------------------------+---------------------+
|`ThetaStarPlanner`_        | An implementaion of Theta*   | Differential,       |
|                           | using either 4 or 8          | Omnidirectional     |
|                           | connected neighborhoods,     |                     |
|                           | assumes the robot as a       |                     |
|                           | 2D holonomic particle        |                     |
+---------------------------+------------------------------+---------------------+

.. _NavFn Planner: https://github.com/ros-planning/navigation2/tree/main/nav2_navfn_planner
.. _SmacPlannerHybrid: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner
.. _SmacPlanner2D: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner
.. _ThetaStarPlanner: https://github.com/ros-planning/navigation2/tree/main/nav2_theta_star_planner
.. _SmacPlannerLattice: https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner


Smoothers
=========

+---------------------------+------------------------------+
| Plugin Name               |       Description            |
+===========================+==============================+
|  `Simple Smoother`_       | A simple path smoother for   |
|                           | infeasible (e.g. 2D)         |
|                           | planners                     |
+---------------------------+------------------------------+
|  `Constrained Smoother`_  | A path smoother using a      |
|                           | constraints problem solver   |
|                           | to optimize various criteria |
|                           | such as smoothness or        |
|                           | distance from obstacles,     |
|                           | maintaining minimum turning  |
|                           | radius                       |
+---------------------------+------------------------------+

.. _Simple Smoother: https://github.com/ros-planning/navigation2/tree/main/nav2_smoother
.. _Constrained Smoother: https://github.com/ros-planning/navigation2/tree/main/nav2_constrained_smoother

Behaviors
=========

+----------------------+----------------------------------+
|  Plugin Name         |       Description                |
+======================+==================================+
|  `Clear Costmap`_    | A service to clear the given     |
|                      | costmap in case of incorrect     |
|                      | perception or robot is stuck     |
+----------------------+----------------------------------+
|  `Spin`_             | Rotate behavior of configurable  |
|                      | angles to clear out free space   |
|                      | and nudge robot out of potential |
|                      | local failures                   |
+----------------------+----------------------------------+
|    `Back Up`_        | Back up behavior of configurable |
|                      | distance to back out of a        |
|                      | situation where the robot is     |
|                      | stuck                            |
+----------------------+----------------------------------+
|             `Wait`_  | Wait behavior with configurable  |
|                      | time to wait in case of time     |
|                      | based obstacle like human traffic|
|                      | or getting more sensor data      |
+----------------------+----------------------------------+
|  `Drive On Heading`_ | Drive on heading behavior with   |
|                      | configurable distance to drive   |
+----------------------+----------------------------------+
|  `Assisted Teleop`_  | AssistedTeleop behavior that     |
|                      | scales teleop commands to        |
|                      | prevent collisions.              |
+----------------------+----------------------------------+

.. _Back Up: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Spin: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Wait: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Drive On Heading: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins
.. _Clear Costmap: https://github.com/ros-planning/navigation2/blob/main/nav2_costmap_2d/src/clear_costmap_service.cpp
.. _Assisted Teleop: https://github.com/ros-planning/navigation2/tree/main/nav2_behaviors/plugins

Waypoint Task Executors
=======================

+---------------------------------+----------------------------------+
|        Plugin Name              |       Description                |
+=================================+==================================+
| `WaitAtWaypoint`_               | A plugin to execute a wait       |
|                                 | behavior  on                     |
|                                 | waypoint arrivals.               |
|                                 |                                  |
+---------------------------------+----------------------------------+
| `PhotoAtWaypoint`_              | A plugin to take and save photos |
|                                 | to specified directory on        |
|                                 | waypoint arrivals.               |
|                                 |                                  |
+---------------------------------+----------------------------------+
| `InputAtWaypoint`_              | A plugin to wait for user input  |
|                                 | before moving onto the next      |
|                                 | waypoint.                        |
+---------------------------------+----------------------------------+

.. _WaitAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/wait_at_waypoint.cpp
.. _PhotoAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/photo_at_waypoint.cpp
.. _InputAtWaypoint: https://github.com/ros-planning/navigation2/tree/main/nav2_waypoint_follower/plugins/input_at_waypoint.cpp

Goal Checkers
=============

+---------------------------------+----------------------------------+
|     Plugin Name                 |       Description                |
+=================================+==================================+
| `SimpleGoalChecker`_            | A plugin check whether robot     |
|                                 | is within translational distance |
|                                 | and rotational distance of goal. |
|                                 |                                  |
+---------------------------------+----------------------------------+
| `StoppedGoalChecker`_           | A plugin check whether robot     |
|                                 | is within translational distance |
|                                 | , rotational distance of goal,   |
|                                 | and velocity threshold.          |
+---------------------------------+----------------------------------+

.. _SimpleGoalChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_goal_checker.cpp
.. _StoppedGoalChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/stopped_goal_checker.cpp

Progress Checkers
=================

+---------------------------------+----------------------------------+
|         Plugin Name             |       Description                |
+=================================+==================================+
| `SimpleProgressChecker`_        | A plugin to check whether the    |
|                                 | robot was able to move a minimum |
|                                 | distance in a given time to      |
|                                 | make progress towards a goal     |
+---------------------------------+----------------------------------+

.. _SimpleProgressChecker: https://github.com/ros-planning/navigation2/blob/main/nav2_controller/plugins/simple_progress_checker.cpp


Behavior Tree Nodes
===================

+--------------------------------------------+------------------------------------------+
| Action Plugin Name                         |       Description                        |
+============================================+==========================================+
| `Back Up Action`_                          | Calls backup behavior action             |
+--------------------------------------------+------------------------------------------+
| `Drive On Heading Action`_                 | Calls drive on heading behavior action   |
+--------------------------------------------+------------------------------------------+
| `Assisted Teleop Action`_                  | Calls assisted teleop behavior action    |
+--------------------------------------------+------------------------------------------+
| `Clear Entire Costmap Service`_            | Calls clear entire costmap service       |
+--------------------------------------------+------------------------------------------+
| `Clear Costmap Except Region Service`_     | Calls clear costmap except region service|
+--------------------------------------------+------------------------------------------+
| `Clear Costmap Around Robot Service`_      | Calls clear costmap around robot service |
+--------------------------------------------+------------------------------------------+
| `Compute Path to Pose Action`_             | Calls Nav2 planner server                |
+--------------------------------------------+------------------------------------------+
| `Smooth Path Action`_                      | Calls Nav2 smoother server               |
+--------------------------------------------+------------------------------------------+
| `Follow Path Action`_                      | Calls Nav2 controller server             |
+--------------------------------------------+------------------------------------------+
| `Navigate to Pose Action`_                 | BT Node for other                        |
|                                            | BehaviorTree.CPP BTs to call             |
|                                            | Navigation2 as a subtree action          |
+--------------------------------------------+------------------------------------------+
| `Reinitalize Global Localization Service`_ | Reinitialize AMCL to a new pose          |
+--------------------------------------------+------------------------------------------+
| `Spin Action`_                             | Calls spin behavior action               |
+--------------------------------------------+------------------------------------------+
| `Wait Action`_                             | Calls wait behavior action               |
+--------------------------------------------+------------------------------------------+
| `Truncate Path`_                           | Modifies a path making it shorter        |
+--------------------------------------------+------------------------------------------+
| `Truncate Path Local`_                     | Extracts a path section around robot     |
+--------------------------------------------+------------------------------------------+
| `Planner Selector`_                        | Selects the global planner based on a    |
|                                            | topic input, otherwises uses a default   |
|                                            | planner id                               |
+--------------------------------------------+------------------------------------------+
| `Controller Selector`_                     | Selects the controller based on a        |
|                                            | topic input, otherwises uses a default   |
|                                            | controller id                            |
+--------------------------------------------+------------------------------------------+
| `Goal Checker Selector`_                   | Selects the goal checker based on a      |
|                                            | topic input, otherwises uses a default   |
|                                            | goal checker id                          |
+--------------------------------------------+------------------------------------------+
| `Navigate Through Poses`_                  | BT Node for other BehaviorTree.CPP BTs   |
|                                            | to call Nav2's NavThroughPoses action    |
|                                            |                                          |
+--------------------------------------------+------------------------------------------+
| `Remove Passed Goals`_                     | Removes goal poses passed or within a    |
|                                            | tolerance for culling old viapoints from |
|                                            | path re-planning                         |
+--------------------------------------------+------------------------------------------+
| `Compute Path Through Poses`_              | Computes a path through a set of poses   |
|                                            | rather than a single end goal pose       |
|                                            | using the planner plugin specified       |
+--------------------------------------------+------------------------------------------+
| `Cancel Control Action`_                   | Cancels Nav2 controller server           |
+--------------------------------------------+------------------------------------------+
| `Cancel BackUp Action`_                    | Cancels backup behavior action           |
+--------------------------------------------+------------------------------------------+
| `Cancel Spin Action`_                      | Cancels spin behavior action             |
+--------------------------------------------+------------------------------------------+
| `Cancel Wait Action`_                      | Cancels wait behavior action             |
+--------------------------------------------+------------------------------------------+
| `Cancel Drive on Heading Action`_          | Cancels drive on heading behavior action |
+--------------------------------------------+------------------------------------------+
| `Cancel Assisted Teleop Action`_           | Cancels assisted teleop behavior action  |
+--------------------------------------------+------------------------------------------+

.. _Back Up Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_action.cpp
.. _Drive On Heading Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_action.cpp
.. _Assisted Teleop Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_action.cpp
.. _Clear Entire Costmap Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Clear Costmap Except Region Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Clear Costmap Around Robot Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/clear_costmap_service.cpp
.. _Compute Path to Pose Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_to_pose_action.cpp
.. _Smooth Path Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/smooth_path_action.cpp
.. _Follow Path Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/follow_path_action.cpp
.. _Navigate to Pose Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_to_pose_action.cpp
.. _Reinitalize Global Localization Service: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/reinitialize_global_localization_service.cpp
.. _Spin Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_action.cpp
.. _Wait Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_action.cpp
.. _Truncate Path: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_action.cpp
.. _Truncate Path Local: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/truncate_path_local_action.cpp
.. _Planner Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/planner_selector_node.cpp
.. _Controller Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_selector_node.cpp
.. _Goal Checker Selector: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/goal_checker_selector_node.cpp
.. _Navigate Through Poses: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/navigate_through_poses_action.cpp
.. _Remove Passed Goals: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/remove_passed_goals_action.cpp
.. _Compute Path Through Poses: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/compute_path_through_poses_action.cpp
.. _Cancel Control Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/controller_cancel_node.cpp
.. _Cancel BackUp Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/back_up_cancel_node.cpp
.. _Cancel Spin Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/spin_cancel_node.cpp
.. _Cancel Wait Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/wait_cancel_node.cpp
.. _Cancel Drive on Heading Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/drive_on_heading_cancel_node.cpp
.. _Cancel Assisted Teleop Action: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/action/assisted_teleop_cancel_node.cpp


+------------------------------------+------------------------+
| Condition Plugin Name              |       Description      |
+====================================+========================+
| `Goal Reached Condition`_          | Checks if goal is      |
|                                    | reached within tol.    |
+------------------------------------+------------------------+
| `Goal Updated Condition`_          | Checks if goal is      |
|                                    | preempted.             |
+------------------------------------+------------------------+
| `Globally Updated Goal Condition`_ | Checks if goal is      |
|                                    | preempted in the global|
|                                    | BT context             |
+------------------------------------+------------------------+
| `Initial Pose received Condition`_ | Checks if initial pose |
|                                    | has been set           |
+------------------------------------+------------------------+
| `Is Stuck Condition`_              | Checks if robot is     |
|                                    | making progress or     |
|                                    | stuck                  |
+------------------------------------+------------------------+
| `Transform Available Condition`_   | Checks if a TF         |
|                                    | transformation is      |
|                                    | available. When        |
|                                    | succeeds returns       |
|                                    | success for subsequent |
|                                    | calls.                 |
+------------------------------------+------------------------+
| `Distance Traveled Condition`_     | Checks is robot has    |
|                                    | traveled a given       |
|                                    | distance.              |
+------------------------------------+------------------------+
| `Time Expired Condition`_          | Checks if a given      |
|                                    | time period has        |
|                                    | passed.                |
+------------------------------------+------------------------+
| `Is Battery Low Condition`_        | Checks if battery      |
|                                    | percentage is below    |
|                                    | a specified value.     |
+------------------------------------+------------------------+
| `Is Path Valid Condition`_         | Checks if a path is    |
|                                    | valid by making sure   |
|                                    | there are no LETHAL    |
|                                    | obstacles along the    |
|                                    | path.                  |
+------------------------------------+------------------------+
| `Path Expiring Timer`_             | Checks if the timer has|
|                                    | expired. The timer is  |
|                                    | reset if the path gets |
|                                    | updated.               |
+------------------------------------+------------------------+


.. _Goal Reached Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_reached_condition.cpp
.. _Goal Updated Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/goal_updated_condition.cpp
.. _Globally Updated Goal Condition: https://github.com/navigation2/blob/replanning/nav2_behavior_tree/plugins/condition/globally_updated_goal_condition.cpp
.. _Initial Pose received Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/initial_pose_received_condition.cpp
.. _Is Stuck Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_stuck_condition.cpp
.. _Transform Available Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/transform_available_condition.cpp
.. _Distance Traveled Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/distance_traveled_condition.cpp
.. _Time Expired Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/time_expired_condition.cpp
.. _Is Battery Low Condition: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/is_battery_low_condition.cpp
.. _Is Path Valid Condition: https://github.com/navigation2/blob/replanning/nav2_behavior_tree/plugins/condition/is_path_valid_condition.cpp
.. _Path Expiring Timer: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/condition/path_expiring_timer_condition.cpp

+--------------------------+----------------------------------+
| Decorator Plugin Name    |       Description                |
+==========================+==================================+
| `Rate Controller`_       | Throttles child node to a given  |
|                          | rate                             |
+--------------------------+----------------------------------+
| `Distance Controller`_   | Ticks child node based on the    |
|                          | distance traveled by the robot   |
+--------------------------+----------------------------------+
| `Speed Controller`_      | Throttles child node to a rate   |
|                          | based on current robot speed.    |
+--------------------------+----------------------------------+
| `Goal Updater`_          | Updates the goal received via    |
|                          | topic subscription.              |
+--------------------------+----------------------------------+
| `Single Trigger`_        | Triggers nodes/subtrees below    |
|                          | only a single time per BT run.   |
+--------------------------+----------------------------------+
| `PathLongerOnApproach`_  | Triggers child nodes if the new  |
|                          | global path is significantly     |
|                          | larger than the old global path  |
|                          | on approach to the goal          |
+--------------------------+----------------------------------+

.. _Rate Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/rate_controller.cpp
.. _Distance Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/distance_controller.cpp
.. _Speed Controller: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/speed_controller.cpp
.. _Goal Updater: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/goal_updater_node.cpp
.. _Single Trigger: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/single_trigger_node.cpp
.. _PathLongerOnApproach: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/decorator/path_longer_on_approach.cpp

+-----------------------+----------------------------------+
| Control Plugin Name   |       Description                |
+=======================+==================================+
| `Pipeline Sequence`_  | A variant of a sequence node that|
|                       | will re-tick previous children   |
|                       | even if another child is running |
+-----------------------+----------------------------------+
| `Recovery`_           | Node must contain 2 children     |
|                       | and returns success if first     |
|                       | succeeds. If first fails, the    |
|                       | second will be ticked. If        |
|                       | successful, it will retry the    |
|                       | first and then return its value  |
+-----------------------+----------------------------------+
| `Round Robin`_        | Will tick ``i`` th child until   |
|                       | a result and move on to ``i+1``  |
+-----------------------+----------------------------------+

.. _Pipeline Sequence: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/pipeline_sequence.cpp
.. _Recovery: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/recovery_node.cpp
.. _Round Robin: https://github.com/ros-planning/navigation2/tree/main/nav2_behavior_tree/plugins/control/round_robin_node.cpp
