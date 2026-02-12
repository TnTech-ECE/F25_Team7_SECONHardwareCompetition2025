# Detailed Design


## Function of the Subsystem


The intended function of the Navigation system is to handle autonomous navigation of the board while avoiding collision with the antennas, or sides of the board. For autonomous navigation, our team shall use Robot Operating System 2 Navigation Stack 2 (ROS 2 Nav2). 
    
The role of the Autonomous Navigation Subsystem is to move from point A to point B without hitting obstacles. Working in-hand with the Local Controller Subsystem and the Object Detection Subsystem - the Navigation Subsystem will take in data from the Object Detection Subsystem in order to determine if the object in front of it is an antennae, Astroduck, crater, or arena edge and it will use this data to determine if the robot will move around an obstacle or pick it up. Once the course path is determined, the Navigation Substack will send this information to the Local Controller which will communicate the desired rotational position of the wheel motors and provide movement towards Point B.
    
An essential component of the Navigation subsystem is Autonomous Navigation and Internal Guidance.
    
* <ins>Autonomous Navigation</ins> provides directional input directly to an autonomous vehicle in order to execute the planned route without human intervention. 
* <ins>Internal Guidance</ins> is a means to maintain navigation accuracy through Inertial Measurement Units (IMUs) and software without human interference.




## Overview of Software and Build Procedure
   ROS 2 Nav2 operates on Linux software. Linux computers rented from Tennessee Tech Volpe Library IT have limitations and restrictions on what can be installed. For this reason a breakdown below is provided to show the software that will be used to run ROS 2 through Windows operating system


### Desired Solution:

* **Middleware**
    * <ins>ROS 2</ins>
* **Default Distro Type**
    * <ins>_Window Subsystem for Linux 2 (WSL2)_</ins> - Runs linux environment
        * **Operating system: Ubuntu 24** - Open source operating system
    * ROS 2 version:
        * **Kilted** - Latest version of ROS 2
* **SLAM (Simultaneous Localization and Mapping) Package**
    * <ins>_SLAM Toolbox_</ins>
        * Uses data from LiDAR sensors and robot/ camera drones
* **3D Simulated Software** (Optional)
    * <ins>_Gazebo_</ins> - Can run/ practice simulation of ROS 2


### Alternative Solutions:

* **Default Distro Type**
    * <ins>_Native Installation_</ins>
        * Install ROS 2 binaries **(<ins>ROS2 Humble</ins>)**
        * Install <ins>**Visual Studio**</ins>
        * Build navigation stack from the source
    * ROS 2 version:
        * <ins>**Jazzy**</ins> - Older version of ROS 2 previous to Kilted
* **SLAM (Simultaneous Localization and Mapping) Package**
    * <ins>_Cartographer_</ins>




## Specifications and Constraints
There are various constraints that are applicable to the Autonomous Navigation subsystem. These constraints include: Static vs. Dynamic Environments, Sensor Limitations, Odometry Drift, Robotic Hardware Kinematic Ability and Obstacle Avoidance.


### Constraints

#### 1. Static vs. Dynamic Environments:
* Some autonomous navigation systems are best suited for static and semi-static indoor environments. And others can traverse dynamic environments. 
* Unstructured environments (i.e. grassy terrain), crowded spaces and highly dynamic 3D environments pose issues with the navigation stack’s perception accuracy and response delays. 




#### 2. Sensor Limitations:
* Autonomous navigation relies on sensor data like LiDAR for 2D mapping and obstacle detection. In complex environments, certain objects can be treated as intraversable. 
* ROS 2 Nav2 can succumb to a low fault tolerance and get stuck in an inefficient path due to noisy data.




#### 3. Odometry Shift: 
* The odometry (position estimation) of autonomous navigation systems creates a physical limitation on the subsystem due to wheel slip, uneven surfaces and sensor noise. This creates a discrepancy in the positional coordinates of the robot.




#### 4. Obstacle Avoidance: 
* This constraint set by IEEE on the robot is intended for the robot to navigate the board autonomously without hitting any obstacles such as the antennas.




#### 5. Socio-Economic: 
* ROS 2 Nav2 was chosen for its free and open-source nature. This increased financial availability in the budget allows for widespread accessibility and adaptation of the software. It also funds to be allocated to other subsystems to create a robust profile of the autonomous robot.


#### 6. Ethical: 
* The importance of safety in an autonomous system is to avoid harm to humans. Any machine without a human operator is subject to error in navigation which can cause collision with pedestrians or animals and result in injury. The team desires to avoid aggressive path planning of the navigation system. To avoid this, an autonomous vehicle will need to see humans as a priority and yield to them. This can be done by using social navigation plug-ins in the open source code. Another alternative is to create an environment that prevents humans from wandering into the space of the robot. As per the IEEE requirement to build a board where the tasks will take place, our team shall create an area with borders that allows the robot to move without the interference of people.


#### 7. Standards: 
* <ins> IEC 62443 - Industrial Cybersecurity:</ins> Secures Industrial Automation and Control Systems and Operational Technology against cyber threats
    * **ROS 2 Security (SROS2):** Our team shall use ROS 2 Nav2 due to its use of DDS-Security standard (Data Distribution Service) which provides encryption, authentication and access control to prevent insertion of malicious code or data.

* <ins>ISO/IEC 25010 - Software Quality Model:</ins> Provides quality model for software products
    * **ISO/IEC 25010** is a standard that ensures:
        * <ins>Reliability</ins> - No crashes in runtime
            * Troubleshooting code for errors
        * <ins>Performance efficiency</ins> - Real-time obstacle avoidance
            * Handled by path planning, localization, inflation radius clearance and the Object Detection subsystem SLAM map
        * <ins>Security</ins> - Protected communication
            * Implemented and supported by SROS2
        * <ins>Maintainability</ins> - Modular plugin architecture
            * ROS 2 Nav2 open-source code allows for modularity



### Specifications

Specifications applicable to the Autonomous Navigation subsystem are Inflation Radius, Timing, Path Planning and Localization: 



#### 1. Inflation Radius
* During navigation the ground robot needs space to move between and around objects on the board. This space is called clearance and is handled by providing an inflation radius which acts as a safety buffer. Higher safety buffers around obstacles reduce the risk of collision but also reduce maneuverability




#### 2. Timing
* Timing affects the rate that velocity commands are calculated by the local trajectory planner. A higher frequency (update rate) will lead to more frequent velocity command calculations. A higher frequency/ update rate makes obstacle avoidance and responsiveness more efficient but increases CPU load




#### 3. Path Planning
* Smaller pixel values increase map detail and planning exactness but requires more memory and computation




#### 4. Localization and Error
* Defined as the robot’s estimated position in relation to its actual position on the board. Higher numbers of particles used by Adaptive Monte Carlo Localization (AMCL) for localization decrease localization error by increasing accuracy but reduce speed.




#### 5. Latency
* The concept of latency refers to the time delay between when data is produced and when it is processed. The sensors will produce data relating to the location of the objects on the board. The Low-Level Controller Subsystem will give data on the position of the robot on the board and the positive relative to objects detected on the board (via the Object Detection subsystem). All of this data produced will be processed by the ROS 2 so that the robot can successfully navigate.


## Overview of Proposed Solution
A solution to the constraints and specifications listed will be created by utilizing the strengths of ROS 2 Nav2 as well creating a structured environment that is tailored to suit the performance and best efficiency practices of ROS 2 Nav2.


### <ins>Constraints</ins>:
One proposition to work around Static vs. Dynamic Environments, Sensor Limitations and Odometry Shift is to create a static environment with low complexity and utilize ROS 2 Nav2’s internal localization methods to correct Odometry shift. 




#### 1. Odometry Shift: 
Localization error is handled via global localization algorithms which in our case is AMCL. AMCL is used to correct odometry shifts and realign the robots positioning with the real coordinates. Though ROS 2 can’t entirely cure the physical limitation of Odometry shift, it can use its internal localization methods to correct and override it.




#### 2. Static vs. Dynamic Environments and and Sensor Limitations:
* ROS 2 NAV2 particularly is best suited for static and semi-static indoor environments and by creating a static environment on the board where there is minimal movement, a flat board and minimal dynamic obstacles - the team can ensure Navigation Stack’s ability to perceive its surroundings and send this data to the local controller.
* This static environment with minimal obstacles places the odds in our favor for the navigation system. With a repetitive environment, ROS 2 can lose track of its positioning due to the lack of landmarks to help establish itself. Due to the placement of the antennas, crater, Earth, and start-zone the board will have enough landmarks to convey relative-surroundings and decrease localization error.




#### 3. Obstacle Avoidance: 
ROS 2 Nav2 can avoid static and dynamic obstacles through continuous updates to its Costmaps. This is done via information received from the sensors. This data is then processed into obstacle layers in the costmap and ROS will publish this data so the local controller can move the wheels according to the obstacles in front of it.




### <ins>Specifications and Measureability:</ins>

#### Measurability

* The measures that will be used to determine the success of the navigation subsystem are the ranges set for Latency, Inflation Radius, Timing, Path Planning, and Localization. Our team shall shoot for a success rate of 70% meaning; when conducting 10 trail runs for the robot, for 7 trials our team will strive to achieve the ranges defined below for the navigation system in order to ensure quickness and efficiency.


#### 6. Inflation Radius / Clearance
* The inflation radius of ROS 2 Nav2 will be set within a range of 0.076 m to 0.1524m which translates to a clearance of 3-6 inches in order to give robot a safe buffer and approach the antennas without reducing maneuverability




#### 7. Timing
* For timing, due to the low complexity of the environment, a frequency range between 20 Hz to 50 Hz will be used. This will update the motion command 20-50 times per sec in order to increase responsiveness without requesting more CPU power.




#### 8. Path Planning
* The map resolution for path planning will be set to 0.05 m per pixel instead of 0.01 m to allow for decreased computational demands while creating a standardized detail of the map.




#### 9. Localization
* To reduce localization error a particle number range of 750-1,000 will be used to ensure the certainty of the robot’s positioning on the board. The maximum range for particles is 2,000-5,000 which would lower the robot’s speed. Using 750-1,000 will allow for a balance between increased speed and accuracy.




#### 10. Latency
* It is within our Team’s best interest to seek a low latency time in order to complete our tasks within 3 minutes. Latency will be measured by using - ros2_tracing to achieve a minimum latency range of 1ms - 10ms.



## Interface with Other Subsystems

ROS 2 Nav2 seeks to organize navigation through modular nodes connected to ROS 2 topics, services and actions. Core units include the behavior tree (BT), controller servers, and costmap servers which exchange paths, velocity commands and environmental data to avoid obstacles and move from Point A to Point B.

### Interface with the Low-Level Controller System (LLC)

Through ROS 2’s “subscribe and publish” system - data is communicated and retrieved. Velocity commands are published to the robot’s local controller through ros2_control. The Local Controller converts these commands into motor signals. The linear and angular velocity derived from the Global Controller’s goals and the robot's kinematic constraints are sent at a high frequency to wheel drivers in the Local Controller. The wheels are then adjusted based on the published data. 

### Interface with the Global Controller System (GC)

The navigation subsystem works alongside the Global Controller which retrieves and processes sensor and camera data in order to make decisions. These cameras will be mounted on the robot and the UAV, the sensors will be mounted on the robot. The imagery and data retrieved from these cameras and sensors allow for the navigation substack to create a clearance called an inflation layer around the obstacles in order to avoid accidental collision by moving too close to it.

### Interface with the Object Detection System

The Navigation subsystem retrieves the detected obstacles from the Object Detection subsystem and the distance from the robot to the obstacle. The Navigation subsystem retrieves the distance from the robot to the obstacle, and an approximate positioning of objects on the board within the map. ROS 2 Nav2 will output updates to the transform frame and map.




## Software Schematic

### ROS 2 Nav2 Communication Inputs/Outputs

<img width="1707" height="880" alt="ROS 2 Nav2 Communication Inputs_Outputs" src="https://github.com/user-attachments/assets/6c699042-2140-4104-9a15-9a32cadb1da7" />


### Pseudocode for Navigation to Goal

    function navigate_to_goal(goal_pose):
    # Step 1: Initialize system
    start lifecycle nodes (planner_server, controller_server, costmap_server, bt_navigator)
    load plugins (planners, controllers, costmap layers)


    # Step 2: Behavior Tree orchestrates navigation
    BT = load_behavior_tree("navigate_to_pose.xml")


    # Step 3: Global Planning
    current_pose = get_robot_pose(tf, odom)
    global_plan = planner_server.compute_path(current_pose, goal_pose, global_costmap)


    if global_plan is invalid:
        run_recovery_behavior()
        return FAILURE


    # Step 4: Local Control Loop
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



## Relevant Libraries for ROS 2 Nav2

Relevant libraries for include:

* nav2_core: Defines abstract interfaces for planners, controllers, and recovery behaviors. Plugins implement these interfaces (e.g., SmacPlanner, DWB Controller).
  
* nav2_util: Provides utility functions like lifecycle management, geometry helpers, and robot utilities.

* nav2_costmap_2d: Maintains 2D costmaps with obstacle, inflation, static, and voxel layers. These maps are used by planners and controllers.

* nav2_behavior_tree: Implements behavior trees that orchestrate navigation tasks (navigate to pose, follow path, recover).

* nav2_bt_navigator: Executes the behavior tree, coordinating planner and controller servers.

* nav2_controller: Hosts local controllers (e.g., DWB, MPPI, RPP) that compute velocity commands.

* nav2_planner: Hosts global planners (e.g., SmacPlanner, NavFn) that compute paths.

* nav2_amcl: Provides localization using Adaptive Monte Carlo Localization (AMCL).

* nav2_collision_monitor: Monitors for collisions and enforces safety behaviors.


## Flowchart

<img width="2329" height="1777" alt="ROS 2 Nav2 (Nav) Flowchart" src="https://github.com/user-attachments/assets/7c592bbc-2eb6-487c-ac5d-5a5a8b2b95c5" />


## Analysis
ROS 2 Nav2 will be utilized in respect to its provision of behavior tree, planner server, controller Server, costmap servers, actions, topics, services, rates, sensor and environment data.


### How obstacle avoidance is achieved across subsystems

#### Planner server:
* Uses the global costmap to avoid known obstacles and produce a collision-free path with inflation buffers. ROS 2 will then reroute if there’s an obstruction detected from the behavior tree.

#### Controller server: 
* Uses the local costmap to process dynamic obstacles, local changes, and adjust the velocity (cmd_vel) to steer around objects.

#### Behavior Tree Coordination: 
* Triggers recovery behaviors to clear the costmap, spin the robot around, and retry planning if the server reports a blocked path or extreme error. This will re-set an operable path plan.

ROS 2 Nav2’s design allows for customization and configuration that can increase navigation efficiency and impact data and communication.

### Configuration Considerations that Impact Data and Communication

#### Controller and planner selection: 
* Different plugins (DWB, MPPI, RPP, Smac planners) change the nature of internal scoring, smoothing, and horizon. For instance, high-rate controllers paired with robust global planners provide increased avoidance and stability in mixed static–dynamic environments.

#### Costmap layer tuning: 
* Obstacle and voxel layer parameters (i.e. observation persistence, clearing) and inflation radius directly shape the costmap produced and alter the data that the planner and controller use as well as the clearance enforced in the path.

#### Pipeline customization: 
* If needed, ROS 2 Nav2’s modular pipeline allows for the integration of additional local planning ahead of the controller. This allows the local costmap to refine the global plan.


### Behavior tree and orchestration
#### Role: 
* Coordinates tasks (i.e. compute path, follow path, recover) by calling planner and controller actions, and controlling conditions (i.e. transform availability, costmap validity).

#### Inputs: 
* Navigation goal (pose), robot state (tf frames, odom), status from planner/controller actions, and costmap readiness.

#### Outputs: 
* Action goals to planner and controller, recovery actions and navigation status.

#### Communication: 
* Communications for behavior tree include actions for planning and control, topics for tf and odometry and services to manage lifecycle/ configuration. The behavior tree ensures that the planner server first runs to produce the global path, then the controller server uses the path for motion as costmaps update.


### Planner server (global planning)

#### Role: 
* Computes a global path from current pose to goal using the global costmap (i.e. A*, SmacPlanner).

#### Inputs: 
* Global costmap, robot pose (via tf/odom), goal pose, planning parameters.

#### Outputs: 
* nav_msgs/Path (global plan) published for downstream of the system, action result back to behavior tree.

#### Communication: 
* Communications for the planner server include the action to request planning, topics for path and costmap and frame transforms. The planner server runs at lower frequency than the controller server and feeds the global plan to the controller server.


### Controller server (local control)

#### Role: 
* Follows the given plan while avoiding close obstacles using the local costmap, generates velocity commands (cmd_vel) to the wheel controllers (LLC subsystem).

#### Inputs: 
* nav_msgs/Path (latest global or locally refined plan), local costmap, robot odometry, robot kinematic limits, controller-specific parameters (i.e. DWB (Dynamic Window Bounding), MPPI (Model Predictive Path Integral Controller), RPP (Regulated Pure Pursuit)).

#### Outputs: 
* geometry_msgs/Twist (cmd_vel) to the mobile base, status and feedback to the behavior tree.

#### Communication: 
* Communications include action for “follow path”, topics for cmd_vel, costmap, tf, odom. The controller server operates at a higher frequency than the planning server and enables responsive obstacle avoidance and plan tracking.


### Costmap servers (global and local)

#### Role: 
* Maintain 2D grids of environment costs (static map, obstacles, inflation buffers). The costmap server is continuously updated from sensors to showcase free or occupied space.

#### Inputs: 
* sensor_msgs/LaserScan, sensor_msgs/PointCloud2, static map (nav_msgs/OccupancyGrid), and plugin layers (obstacle, voxel, inflation).

#### Outputs: 
* costmap2d grids exposing obstacle and inflated zones to planner and controller.

#### Communication: 
* Communications consist of topics for sensor streams and the map. It also includes services and parameters for plugin configuration. The global costmap feeds planning, the local costmap feeds control for real-time obstacle avoidance.


### Sensor and environment data

#### Laser/Depth streams: 
* LaserScan or PointCloud2 deliver ranges and points used by obstacle or voxel layers to mark and clear cells. The inflation layer expands obstacle costs to enforce clearance. These are used by both global and local costmaps with local costmap prioritized for real-time avoidance near the robot.

#### Map and transforms: 
* The map server provides an OccupancyGrid used by the global costmap. Transforms (tf) and odometry supply the robot pose in the global frame for planning and in the base frame for control, ensuring consistent coordinates across nodes.
    * In the event of a transform frame issue ROS 2 will run a planner recovery and clear the costmap to reduce localization error.


### Actions, Topics, Services, and Rates

#### Actions: 
* ComputePathToPose (planner) and FollowPath (controller) encapsulate goal requests, feedback, and results, enabling the behavior tree to manage execution and recovery behaviors. This establishes the global control (decision-making) and local control (motion execution) boundaries.

#### Topics: 
* cmd_vel (Twist) to the base, path publication for visualization and monitoring, sensor topics for costmaps, tf and odom for pose. Topic names and QoS (Quality of Service) are configured to balance reliability and latency.

#### Typical update rates: 
* Costmap and Planner Frequency Update Rate:
    * Global planning runs at a lower update frequency (which our team shall set to ~1 Hz or on-demand).
        * Global planner frequency input: planner_frequency
  
* The local costmap runs at higher update frequency (which our team shall set between 5-10+ Hz).
    * Local and Global Costmap (environmental awareness) update frequency input: update_frequency
        * These ranges support immediate obstacle avoidance and smooth motion tracking without wasting energy through computing.
     
* Controller Server Frequency Update Rate:
    * Timing frequency (update rate) is the frequency in which the velocity commands from the local planner are calculated. As outlined above our team shall strive for a frequency range between 20 Hz to 50 Hz.
        * Controller frequency input: controller_frequency

* Odometry Rate:
    * Odometry rate shall be set at a minimum value of 20Hz+. This high frequency allows our robot to get a smooth and accurate approximate position.
        * Odometer Parameters: frequency, rate, publish_rate



## References


[1] H. Fujita, Y. Guo, A. Porter, and X. Wu, “Autonomous Navigation - an Overview | 
Sciencedirect topics,” Science Direct, https://www.sciencedirect.com/topics/computer-science/autonomous-navigation (accessed Dec. 8, 2025). 

[2] “Nav2 — Nav2 1.0.0 documentation,” docs.nav2.org. https://docs.nav2.org/

[3] “Utility Libraries | ros-navigation/navigation2 | DeepWiki,” DeepWiki, 2025. 
https://deepwiki.com/ros-navigation/navigation2/5.6-utility-libraries (accessed Dec. 08, 2025).

[4] “Mapping and Localization — Nav2 1.0.0 documentation,” Nav2.org, 2023. 
https://docs.nav2.org/setup_guides/sensors/mapping_localization.html

[5] “Setting Up Transformations — Nav2 1.0.0 documentation,” Nav2.org, 2023. 
https://docs.nav2.org/setup_guides/transformation/setup_transforms.html (accessed Apr. 18, 2025).

[6] “Dynamic Object Following — Nav2 1.0.0 documentation,” Nav2.org, 2025. 
https://docs.nav2.org/tutorials/docs/navigation2_dynamic_point_following.html (accessed Dec. 08, 2025).

[7] “🥅 Goal Navigation - PAL OS 25.01 documentation,” Pal-robotics.com, 2024. 
https://docs.pal-robotics.com/25.01/navigation/goal-navigation.html (accessed Dec. 08, 2025).

[8] “nav2_behavior_tree - ROS Package Overview,” Ros.org, 2025. 
https://index.ros.org/p/nav2_behavior_tree/ (accessed Dec. 08, 2025).

[9] Kevin Wood | Robotics & AI, “Learn ROS 2: Beginner to Advanced Course (Concepts 
and Code),” YouTube, Jan. 24, 2025. https://www.youtube.com/watch?v=HJAE5Pk8Nyw (accessed Dec. 08, 2025).

[10] S. Macenski, A. Soragna, M. Carroll, and Z. Ge, “Impact of ROS 2 Node 
Composition in Robotic Systems,” arXiv.org, May 16, 2023. https://arxiv.org/abs/2305.09933 (accessed Nov. 26, 2023).

[11] “Configuration Guide — Nav2 1.0.0 documentation,” Nav2.org, 2025. 
https://docs.nav2.org/configuration/index.html#core-servers (accessed Dec. 08, 2025).

[12] “ros-navigation/navigation2,” GitHub, Jun. 01, 2024. 
https://github.com/ros-navigation/navigation2

[13] “ROS 2 Tutorials,” YouTube. 
http://www.youtube.com/playlist?list=PLgG0XDQqJckkSJDPhXsFU_RIqEh08nG0V (accessed Dec. 08, 2025).

[14] “Navigation ROS 2,” YouTube.    
http://www.youtube.com/playlist?list=PLgG0XDQqJcknP8fhdAXxYv6AYnfqjP6WU (accessed Dec. 08, 2025).

[15]“Understanding IEC 62443,” www.iec.ch, Feb. 26, 2021. 
https://www.iec.ch/blog/understanding-iec-62443

‌[16]“ISO/IEC 25010 - Systems and Software Quality,” arc42 Quality Model, Aug. 31, 
2025. https://quality.arc42.org/standards/iso-25010

[17]“ROS 2 Security — ROS 2 Documentation: Iron documentation,” Ros.org, 2025. 
https://docs.ros.org/en/iron/Concepts/Intermediate/About-Security.html (accessed Feb. 12, 2026).

[18]“ROS 2 DDS-Security integration,” design.ros2.org. 
https://design.ros2.org/articles/ros2_dds_security.html

‌[19]T. Schworer, J. E. Schmidt, and D. Chrysostomou, “Nav2CAN: Achieving Context  
Aware Navigation in ROS2 Using Nav2 and RGB-D sensing,” 2021, doi: 
https://doi.org/10.1109.

[20]“Running Your First ROS Node on Ubuntu, macOS, or Windows,” Foxglove.dev, 
2021. https://foxglove.dev/blog/running-your-first-ros-node (accessed Feb. 12, 2026).

[21]TUM-AVS, “GitHub - TUM-AVS/ros2_latency_analysis,” GitHub, Mar. 03, 2023. 
https://github.com/TUM-AVS/ros2_latency_analysis (accessed Feb. 12, 2026).

[22]“ROS 2 Composition: Intra-Process Comms, Zero-Copy And Latency,” Patsnp 
Eureka, 2017. https://eureka.patsnap.com/report-ros-2-composition-intra-process-communications-zero-copy-and-latency (accessed Feb. 12, 2026).
