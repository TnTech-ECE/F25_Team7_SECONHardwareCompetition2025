# Detailed Design

## Function of the Subsystem

The role of the Navigation system is to handle autonomous navigation of the arena while avoiding collision with the antennas, or sides of the board. The Navigation subsystem. For autonomous navigation, our team shall use Robot Operating System 2 Navigation Stack (ROS 2 Nav2). The role of the Autonomous Navigation Subsystem is to move from point A to point B without hitting obstacles. Working in-hand with the Local Controller Subsystem and the Object Detection Subsystem - the Navigation Subsystem will take in data from the Object Detection Subsystem in order to determine if the object in front of it is an antennae, Astroduck, crater, or arena edge and it will use this data to to determine if we move around an obstacle or pick it up. Once the course path is determined, the Navigation Substack will send this information to the Local Controller which will communicate the desired rotation position of the wheel motors and provide movement towards Point B.


## Specifications and Constraints

There are various constraints that are applicable to the Autonomous Navigation subsystem. These constraints include: Static vs. Dynamic Environments, Sensor Limitations, Odometry Drift, Robotic Hardware Kinematic Ability and Obstacle Avoidance.


Static vs. Dynamic Environments:
Some autonomous navigation systems are best suited for static and semi-static indoor environments. And others can traverse dynamic environments. Unstructured environments (i.e. grassy terrain), crowded spaces and highly dynamic 3D environments pose issues with the Navigation Stack’s ability perception accuracy and response delays. 


Sensor Limitations:
Autonomous navigation relies on sensor data like LiDAR for 2D mapping and obstacle detection. In complex environments, certain objects can be treated as intraversable. ROS 2 Nav2 also can also succumb to a low fault tolerance and get stuck in an inefficient path due to noisy data.


Odometry Shift: 
The odometry (position estimation) of autonomous navigation systems creates a physical limitation on the subsystem due to wheel slip, uneven surfaces and sensor noise. This creates a discrepancy in the positional coordinates of the robot.


Obstacle Avoidance: 
The last constraint of the autonomous subsystem set by IEEE on the robot is for the robot to be able to navigate autonomously without hitting obstacles such as the antennas.



## Overview of Proposed Solution

A proposed solution to work around Static vs. Dynamic Environments, Sensor Limitations and Odometry Shift is to create a static environment with low complexity and utilize ROS 2 Nav2’s eternal localization methods to correct Odometry shift. Though ROS 2 can’t entirely cure the physical limitation of Odometry shift, it can use external localization methods to correct and override it. 
ROS 2 NAV2 particularly is best suited for static and semi-static indoor environments and by creating a static environment on the board where there is minimal movement, a flat board and minimal dynamic obstacles - the team can ensure Navigation Stack’s ability to perceive its surroundings and send this data to the local controller.
ROS 2 Nav2 can avoid static and dynamic obstacles through continuous updates to its Costmaps. This is done via information received from the sensors. This data is then processed into obstacle layers in the costmap and ROS will publish this data so the local controller can move the wheels according to the obstacles in front of it. [1]



## Interface with Other Subsystems

ROS 2 Nav2 seeks to organize navigation through modular nodes connected to ROS 2 topics, services and actions. Core units include the behavior tree (BT), controller servers, and costmap servers which exchange paths, velocity commands and environmental data to avoid obstacles and move from Point A to Point B. [https://docs.nav2.org/] 
Through ROS 2’s “subscribe and publish” system - data is communicated and retrieved. Velocity commands are published to the robot’s local controller through ros2_control. The Local Controller converts these commands into motor signals. The linear and angular velocity derived from the Global Controller’s goals and the robot's kinematic constraints are sent at a high frequency to wheel drivers in the Local Controller. The wheels are then adjusted based on the published data. [https://docs.nav2.org/] 
The navigation subsystem works alongside the Global Controller which retrieves and processes sensor and camera data in order to make decisions. These cameras will be mounted on the robot and the UAV, the sensors will be mounted on the robot. The imagery and data retrieved from these cameras and sensors allow for the navigation substack to create an inflation layer around the obstacles in order to avoid accidental collision by moving too close to it.



## Buildable Schematic 

Pseudocode:

function navigate_to_goal(goal_pose):
   Step 1: Initialize system
   start lifecycle nodes (planner_server, controller_server, costmap_server, bt_navigator)
 	   load plugins (planners, controllers, costmap layers)

   Step 2: Behavior Tree orchestrates navigation
   BT = load_behavior_tree("navigate_to_pose.xml")

   Step 3: Global Planning
current_pose = get_robot_pose(tf, odom)
global_plan = planner_server.compute_path(current_pose, goal_pose,    global_costmap)

if global_plan is invalid:
           run_recovery_behavior()
           return FAILURE

  Step 4: Local Control Loop
while not goal_reached:
local_costmap = update_costmap(sensor_data)
        cmd_vel = controller_server.compute_velocity(global_plan, local_costmap, current_pose)

  if cmd_vel is invalid:
            run_recovery_behavior()
            continue

  publish(cmd_vel to mobile_base)

  if reached(goal_pose):
            return SUCCESS
  return FAILURE



# Libraries include:

nav2_core: Defines abstract interfaces for planners, controllers, and recovery behaviors. Plugins implement these interfaces (e.g., SmacPlanner, DWB Controller).
nav2_util: Provides utility functions like lifecycle management, geometry helpers, and robot utilities.
nav2_costmap_2d: Maintains 2D costmaps with obstacle, inflation, static, and voxel layers. These maps are used by planners and controllers.
nav2_behavior_tree: Implements behavior trees that orchestrate navigation tasks (navigate to pose, follow path, recover).
nav2_bt_navigator: Executes the behavior tree, coordinating planner and controller servers.
nav2_controller: Hosts local controllers (e.g., DWB, MPPI, RPP) that compute velocity commands.
nav2_planner: Hosts global planners (e.g., SmacPlanner, NavFn) that compute paths.
nav2_amcl: Provides localization using Adaptive Monte Carlo Localization (AMCL).
nav2_collision_monitor: Monitors for collisions and enforces safety behaviors.

[https://deepwiki.com/ros-navigation/navigation2/5.6-utility-libraries]



## Flowchart

For sections including a software component, produce a chart that demonstrates the decision-making process of the microcontroller. It should provide an overview of the device's function without exhaustive detail.


## Analysis

Deliver a full and relevant analysis of the design demonstrating that it should meet the constraints and accomplish the intended function. This analysis should be comprehensive and well articulated for persuasiveness.

## References

All sources that have contributed to the detailed design and are not considered common knowledge should be duly cited, incorporating multiple references.
