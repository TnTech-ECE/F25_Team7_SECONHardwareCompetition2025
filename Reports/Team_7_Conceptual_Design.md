# Conceptual Design


## Introduction

The IEEE (Institute of Electrical and Electronics Engineers) hosts an annual conference for the southeastern region known as SoutheastCon (SECON). This event brings together members of IEEE—including electrical engineers, computer engineers, and computer scientists—to share and discuss research across various technical fields. As part of the conference, multiple competitions are held, ranging from circuit design to website development, including a highly regarded hardware competition [1]. Tennessee Tech University has consistently participated in these competitions, showcasing the skills and innovation of its students and future engineers. For this year’s challenge, participants are tasked with developing an autonomous ground robot and an unmanned aerial vehicle (UAV) capable of communicating and collaborating to complete a series of objectives. Team performance is evaluated based on the number of “Astro-Ducks” rescued, antennas restarted, and successful satellite launches completed within the given time limit. This competition presents a complex and engaging set of engineering challenges, such as autonomous control, wireless communication, and task-specific robotic design [2]. The team plans to collaborate with students from the Mechanical Engineering Department and several faculty members. Their combined expertise will assist in constructing the practice arena, designing the final robot, and ensuring that the project is executed to the highest standard.


## Restating the Fully Formulated Problem


## Comparative Analysis of Potential Solutions


## High-Level Solution


### Hardware Block Diagram:



### Operational Flow Chart:

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Operational%20Flow%20Chart.png)



## Atomic Subsystem Specifications


## Global Control Subsystem

Customer: Dr. Johnson

Designer: Jane Vassar

### *Specifications:*
1. Both the robot and UAV shall be completely autonomous [Vehicle Spec 1]
2. The UAV shall not weigh more than 0.55 pounds or 250 grams [Vehicle spec 5]
3. The robot and UAV shall have a clearly labeled start switch. [Vehicle Spec 12]
4. Teams shall have a maximum of 3 minutes to earn points. [Objective Spec 3]
     - The robot shall have the ability to quickly process sensor data.
     - The robot shall have the ability to quickly process AI workloads.
     - The robot shall have the ability to quickly make decisions and communicate them.
5. Teams shall rescue a total of 6 Astro-Ducks and shall return them to the “Lunar Landing Area.” [Objective Specs 4/5]
     - The robot shall be able to identify, locate, and navigate to the Astro-Ducks.
     - The robot shall be able to identify and navigate to the Lunar Landing Area.
6. Teams shall establish power to the 4 antennas throughout the course. Power shall be restored differently for each antenna. Once power is restored to the antenna, a randomly colored LED shall light up (red, blue, green, and purple). [Objective Spec 6]
     - The robot shall be able to identify, locate, and navigate to the antenna towers.
     - The robot must identify and complete the task located on each tower.
     - The UAV shall identify the color of the LED on each antenna tower.
     - The UAV shall know which area each antenna tower and their LEDs are located in.
7. Two starting white LED bars shall be placed on top of the arena wall, one on each side of the 12” x 12” starting area. [Board Spec 5]
     - The robot shall be able to sense the starting white LEDs in order to begin operation.
8. Points shall be lost every time the robot or UAV has an unintentional collision [Objective Constrain 2]
     - The robot and UAV shall be able to detect and avoid obstacles.

### *Comparative Analysis of Potential Solutions:*
#### Global Contoller
In order to meet specifications 1, 5, and 6; the robot shall have a computer that is capable of receiving incoming data from its sensors, processing that data, and making decisions with that data. The robot shall be able to identify objects like the Astro Ducks and antenna towers and then plan an efficient route to navigate to those objects. Finally the computer shall send the needed instructions to the low level controller in order to execute those decisions. 

 
The robot’s ability to detect objects and handle navigation tasks shall depend on its ability to effectively utilize artificial intelligence algorithms. AI algorithms are computationally heavy tasks, meaning that a simple microcontroller shall not be enough to control the robot [1]. A single board computer would be more suited for managing the robot’s high level control. A Raspberry Pi 5 shall be potential option for this task. The Raspberry Pi 5 doesn’t have specialty hardware for AI processing, but it is powerful enough by itself to run light AI load [2]. It is also possible to purchase an expansion board for the Raspberry Pi 5 called the AI hat+ [3]. This module has built in accelerator cores that allow a Raspberry Pi 5 the ability to handle bigger AI loads. However, the extra component introduces an additional point of failure into the system that could prove difficult to troubleshoot. Instead picking a single board computer that is designed for handling AI loads would be a better option. The Jetson line of single board computers made by NVIDIA were designed with AI processing in mind. Potential candidates include the Nano, TX2, and Orin. The Jetson Nano is more powerful and suited to running AI loads compared to the Raspberry Pi 5 (with out considering the AI hat+) [4] [5], however, the TX2 is approximately 2.5 times more powerful than the Nano [6]. The Jetson Orin is listed as being more powerful than both of the previous boards [7]. All three boards have access to the same set of NVIDIA development tools, and have similar development board configurations.  


The robot shall also need a software architecture that will allow the robot to handle all its AI loads and sensor integration. The team has the option to continue developing the architecture designed by the previous team. There is a foundation that the team can use to build upon and add on to, including computer vision algorithms [8] [9]. The team can also build a new software architecture using the Robot Operating System (ROS). This architecture is open source and has a set of prebuilt tools for common robot applications. NVIDIA has also built a ROS distribution called Issac which is designed for using AI tools such as object detection and navigation algorithms [10]. This ROS library also makes using a Jetson board more appealing. 


The team shall use a NVIDIA Jestion single board computer to handle the sensor intake, AI processing, and decision making for the robot. The team shall utilize ROS as a tool to help implement AI work loads and to control the robot. As the team approaches the detailed design phase, they shall decide which Jeston single board computer will be used. This decision will depend on the computers ability to process the robots AI loads fast enough to meet specification 3. ROS has the capability to run both C++ and Python programs, so the team will consider salvaging programs from the previous team’s robot. 


#### Cameras
In order to meet specifications 1, 5, 6, and 8 the robot shall need to be able to visually perceive its environment so it can identify objects and plan paths to those objects. The team shall utilize two cameras; one that will be mounted on the robot’s frame, and one that shall be mounted on the UAV. Both cameras shall need to be able to detect color, as the previously listed specifications shall require the robot and UAV to identify areas like the lunar landing zone and the antenna LEDs based on their colors. Thus both cameras shall, at minimum, be RGB cameras [11]. 


The robot mounted camera shall be needed to detect more than just color. The robot’s ability to see and identify objects only meets part of specifications 1, 5, 6, and 8. The robot shall need to have the ability to know where those objects are in physical space. This can be done using a software process called 3D reconstruction, where three dimension data can be created from a two dimensional image [12], however, there are cameras that can provide this information. These are called RGBD cameras. RGBD cameras are color detecting cameras that have the means of detecting depth. Stereo cameras (cameras with more than one vision sensor) and cameras with time of flight style sensors are the most common [13]. This information shall be necessary for determining how the robot will navigate to objects like the Astro-Ducks. 


The robot mounted camera shall be a RGBD camera that can be used to detect objects and provide information about their locations. The UAV camera shall at minimum be a RGB camera that shall be used to identify LED and area colors. The team shall pick a UAV mounted camera that will help the UAV comply with specification 2. As the team approaches the detailed design phase, the team shall further consider the role of the UAV mounted camera. Some cameras have built in tools for processing image data for tasks like object detection. The team shall consider whether or not the drone mounted camera will take a direct role in processing the image data that it will collect or if sending the unprocessed data to the Jeston computer will be more efficient. 

#### General Sensors

In order to complete specifications 1, 3, 4, 5, 6, and 7 the robot shall need an array of different sensors that will allow the robot to perceive its environment. The robot shall know its position, its orientation, and have an effective method of tracking its location on the game board. The robot shall also have a means of starting autonomously as defined in specification 1, but also have an auxiliary start switch in case the robot fails to start autonomously.  


##### Starting the Robot
The robot shall have two switches that will be accessible from the robot's chassis; the power switch and the start switch. The power switch shall provide power to the robot’s components. The start switch shall send a signal to the robot’s computer to begin operating. Specification 7 requires that the robot begin operation without human intervention, however, the inclusion of a manual start switch shall allow the robot to begin operation in the event it fails to start autonomously. At the beginning of the competition, the robot and drone shall start automatically when the arena’s white “Start LED” bar illuminates as defined in specification 7. This can be done by using an photoresistor to detect when the LED turns on. This is explained in more detail in a later section. 

##### Navigation Sensors
The robot shall use multiple sensors in order to determine its exact location on the game board. These sensors shall also be used to pinpoint the robot on a virtual SLAM map and be used to improve that map. The concept of a SLAM map shall be discussed in a later section. A common technique that the team shall utilize will be sensor fusion. This technique shall allow the robot to combine the data from different types of sensors in order to create a more accurate representation of the robot's location. A common sensor fusion combination includes the use of LiDAR and IMU sensors which the team plans to utilize. 

LiDAR, or Light Detection and Ranging, sensors use laser pulses to measure distance. They can be used for detecting obstacles, and be used to create virtual maps of the robots surroundings. Their ability to gather high volumes of accurate data in a short time frame makes these sensors a good option. They are also well known for their ability to create three dimensional maps, which shall be integral for our navigational strategy as defined in its own subsystem [14] [15]. 

IMUs, or Inertial Measurement Units, are devices that contain an array of sensors like accelerometers, gyroscopes, and potentially magnetometers [16]. These devices are able to track changes in velocity and rotational orientation. These measurements shall allow the robot to know its exact orientation at a given time. The velocity data can also be integrated over short periods in order to find the robots position data [17]. 

Using these two sensor types together shall allow the robot to know its exact position and orientation on the game board. The robot shall use one IMU for position and orientation. As the team continues into the detailed design phase, they shall consider how to implement the LiDAR sensor(s). LiDAR sensors come in various configurations and the team shall consider which configuration is the most cost effective while being able to provide all of the needed information for the robot’s successful operation. 


## Object Detection Subsystem
The drone shall act as an overhead observer while the robot acts as the main ground unit. The drone’s job is to locate and keep track of the robot while navigating it throughout the course.  

By doing this the drone and robot shall do the following: 
1. The drone shall create a SLAM map of the competition within 20 seconds  
2. The drone shall locate and navigate the robot 
3. The drone and robot shall both locate the Astro-Ducks and antennas 
4. The drone and robot shall both identify the specific task of the antennas 
5. The drone shall determine the color of the antennas’ LEDs 
6. The drone and robot shall automatically starting using the LED bar on the competition board 

#### *Creating a SLAM Map:*
To achieve reliable localization and mapping within the strict three-minute match limit, the drone shall utilize a geometry-based SLAM initialization approach using the specification given by the competition ruleset. According to the official ruleset, the field consist of a rectangular 4’ by 8’ plywood base surrounded by 1” by 8” by 8’ border walls. This playing surface also includes a 2’ diameter crater with an 8” flat area near the center of the arena which provides a distinct landmark for visual recognition. These fixed dimensions enable the drone to initialize SLAM instantaneously using a pre-defined arena model. Thus, eliminating the need for extended exploration that would take up more time than necessary.  Below are the steps for which this quick and efficient SLAM map shall be created within the first 20 seconds of the run. 

1. Pre-Run Setup
    -  A prior map of the arena shall be generated from the published arena specifications. This map shall encodes the outer 4’ by 8’ boundary, crater footprint and any other static features that have a fixed location. Since the rules allow full autonomy and unrestricted use of sensors and software, this pre-built model is fully allowed.

2. Initialization and Instant Localization (0-3 seconds)
    - The drone shall instantly take off from the robot and shall position itself at the bottom left corner of the arena where the staring area is. Using the drone’s camera, the system shall detect the wall and floor boundaries then compute a camera-to-floor homography. This geometric fitting aligns the drone’s camera frame with the pre-defined	global coordinate frame of the map, providing instantaneous metric scale and orientation without any markers.

3. Arena Mapping (3-12 seconds)
    - After alignment, the drone shall make a single high-speed pass along the perimeter of the competition arena followed by a figure-eight pass near the center. Visual-inertial odometry (VIO) shall combine the camera and IMU to track the motion between the frames captured by the camera. At the same time, the onboard SLAM system shall triangulate limited 3D points and updates a occupancy grid with around 5 cm of resolution. Due to the pre-defined arena geometry, this step focuses on the refining accuracy and identifying small environmental variations.

4. Drift Correction and Map Closure (12-18 seconds)
    - Once returning to the starting corner, the drone shall re-observe the same arena walls and perform a layout-based loop closure. This is to align the live map back to the known rectangular competition field. This correction shall ensure that positional drift accumulated during the short flight is reduced within centimeters of the real scale.

5. Map Output (18-20 seconds)
    - Finally within 20 seconds, the drone shall produce a rough but globally accurate map. The mapping shall continue in the background as the run goes along, increasing map density while maintaining localization for coordination with the robot.

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/SLAM_Map_FLowchart.png)

#### *Locating and Navigating the Robot:*
After the initial SLAM map is generated by the drone, the robot shall perform autonomous localization and navigation within that map while identifying objects inside the competition arena. Objects such as the Astro-Ducks and antennas shall be identified using vision-based object detection algorithms. Below are the steps using algorithms for the robot to locate itself and start making its way through the course.

1. Initialization and Localization
    - At the start of the run, the robot shall be deployed inside the 12” by 12” by 12” staring cube. The robot shall use the SLAM map generated by the drone to establish its initial position. This shall be done by using the onboard LiDAR sensors and vision camera to compare the nearby walls with those of the known 4’ by 8’ board dimensions. By using a Pose Graph Optimization algorithm, the robot’s local coordinate frame shall be aligned with the SLAM map. This localization state is maintained through sensor fusion in an Extended Kalman Filter (EKF) that shall combine the data from the wheel encoders, IMU, and LiDAR sensors. This maintains a true position of the robot even if the data is imperfect.

2. Landmark-Detection Algorithm
    - A CNN-based object detection model shall be implemented using similar architecture to the YOLOv5-nano. The robot’s camera shall capture frames which are resized and normalized. The YOLOv5-nano shall perform bounding-box predictions with labels for the Astro-Ducks, antennas, and the crater edge. Suppression shall help eliminate duplicate detections which centroid coordinates shall then be projected into the world space. The detected objects shall then be fed into the localization module as landmarks.
  
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Robot_Localization_Navigation_Flowachart.png)

#### *Locating Astro-Ducks and Antennas:*
To efficiently locate the Astro-Ducks and antennas, a dual-layer detection algorithm shall be used that shall combine the drone’s aerial scanning with the robot’s ground level conformation. Together the drone and robot shall maintain the shared SLAM map that is continuously evolving.

1. Drone-Based Aerial Scanning
    - During the beginning SLAM mapping in the first 20 seconds of the run, the drone shall run a lightweight version of the YOLOv5-Nano model tuned for overhead imagery. The drone shall be able to observe the entire field from above and recognize objects based off their characteristics. The Astro-Ducks shall appear as small, bright circular blobs with soft edges while the antennas shall appear as taller structures with clear circular dishes. Each time the drone detects an object, it shall estimate its approximate center pixel and convert them into real-world coordinates relative to the arena. 

2. Robot-Based Close-Range Confirmation
    - As the robot drives towards the coordinates received from the drone, it shall activate the front facing camera and short-range sensors to begin detailed inspection. The robot shall run the same algorithm as the drone but with a model trained for ground-level perspectives. From this distance, the robot shall clearly identify the shapes and textures of the object. The robot shall detect either short shapes with high color saturation or taller objects with dishes at the top. By combining the shape and colors with the object’s height, the robot shall eliminate the chances of a misclassification.
  
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Locating_AstroDucks_and_Antennas.png)

#### *Identifying the Antenna's Task:*
Each of the four antennas shall have unique task to power the LED associated with them. Antenna #1 shall be located in Area #2 and shall have a button task. Antenna #2 shall be located in Area #3 and shall have a crank task. Antenna #3 shall be located in Area #4 and shall have a pressure plate task. Antenna #4 shall be located in Area #1 and shall have a keypad task. Due to the fixed locations of the antennas, the drone and robot can use the SLAM map to automatically know which antenna is where in the arena. Thus, the robot shall know which task needs to be completed without having to visually identify the task itself.

1. Realignment
    - Each antenna’s facing direction is stored in the robot’s map data. By using this, the robot shall be able to position itself in the correct compass direction for the task. The robot’s camera shall be used to ensure that the robot’s position is aligned with that of the actuator being used for the task at hand. The robot’s distant sensors (LiDAR or ultrasonic) shall provide feedback to close in until the measured range matches the activation distance. The robot shall then hold its position for half a second to verify its stability.

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Antenna_Task_Identification.png)

##### *Determining Color of Antenna LEDs:*
Once the robot shall complete a task at the antenna and restore power, an LED in the antennas dish illuminates with one of the four colors: red, blue, green, purple. The drone shall identify the color of the LED and transmit that information back to the base station. This process shall take place while the robot and drone are in the same area, ensuring that the robot can associate with the correct LED with the correct antenna.

1. LED Detection Algorithm
    - The drone’s camera shall be positioned over the antenna’s LED. From there, the drone shall capture high-resolution RGB frames of the antenna dish area. By using the know antenna dimensions, the images shall be cropped around the dish’s location. A circular mask shall then isolate the LED region to remove the surrounding gray and black areas of the antenna. The RGB frames shall be converted to the HSV (High-Saturation-Value) color space, which shall allow color separation under different lighting conditions. The average hue value inside the circular region shall determine the LED color.

2. Validation Filters
    - To prevent false readings from reflections or mixed lighting, the algorithm shall do three things. First, the algorithm shall check the saturation and brightness within the region of the LED. Second, the algorithm shall take the average hue across five frames. Third, if the hue is inconsistent throughout the five frames, the drone shall reposition and try again.
  
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/LED_Detection.png)

#### *Detecting the Starting LED:*
At the beginning of the competition, the robot and drone shall start automatically when the arena’s white “Start LED” bar illuminates. To accomplish this, the robot shall be equipped with a photoresistor (LDR) mounted in a small black tube aimed at the “Start LED” region. The black tube shall act as a light baffle, blocking any unnecessary light from overhead fixtures. When the “Start LED” turns on, the brightness in the sensor’s field of view shall sharply increase causes the voltage of the system to change. Once the voltage shall exceed a set threshold, the software shall confirm that the “Start LED” has been detected. To prevent false triggers, the robot shall use a short debounce delay, requiring the signal to stay above the set threshold for a couple hundred milliseconds before starting.

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Detecting_Starting_LED.png)



## Communications Subsystem
This subsystem is responsible for transferring information between the UAV, the ground robot, and the Earth. Communication between the UAV and the robot shall primarily be established through Wi-Fi. The UAV shall serve as the access point, while the robot shall connect using an adapter linked to the Jetson Nano. This setup allows the robot to send navigational commands to the UAV and enables the UAV to transmit a continuous stream of sensor and camera data back to the robot. For communication with Earth, the UAV shall transmit data regarding the colors of specific satellites using infrared (IR) LEDs. To achieve this, the UAV shall incorporate a small microcontroller equipped with low-power Bluetooth capability to control the IR LEDs. Consequently, the Jetson Nano shall also include Bluetooth functionality to interface with this subsystem. In summary, the robot and UAV shall communicate through Wi-Fi for control and data exchange, while Bluetooth is utilized for sending IR commands. The UAV then uses IR LEDs to transmit the necessary information to Earth.


## Autonomous Navigation Subsystem
According to IEEE SECON guidelines, our team shall create a fully autonomous ground robot that can accurately navigate the board and complete given tasks. This shall be done without any collisions with obstacles on the board. Our team shall use Robot Operating System 2 Navigation Stack (ROS 2 Nav2) in order for our ground robot to autonomously navigate. ROS 2 is a system that allows the robot to move from point A to point B in a space. The usage of Localization and Mapping helps the robot understand its position while creating a real-time map of the environment within the space and moving without hitting obstacles. 

1. Transform Frames and Spatial Coordination:
    - ROS 2 uses Transform Frames (called TF2 - the transform library) in order to develop an understanding of the robot's constituents and track coordinate frames over time. These frames shall be broken down into the map, odom, base_link and LiDAR sensor. A Transformation Matrix shall then be used to convert the position of the Map reference frame to the Odometer reference frame, base_link and then the LiDAR reference frame. During this process, timestamping shall be used to ensure synchronization of the data and tell the navigation the time that the input is taken. 

2. Localization and Mapping:
    - ROS 2 uses AMCL (Adaptive Monte Carlo Localization) and/or GPS to correct wheel slipping and adjust the odometer frame through Global Localization. This shall keep an updated transform of the Map to the Odom. Local methods (Odometry or IMU) shall be used to get a transformation from Odometry to base_link through wheel encoders or IMU. This shall provide an accurate evaluation of motion. The team shall be using a UAV to map the board in order to avoid simultaneous localization and mapping (a feature where the map is created as the robot moves along the board).

3. Perception, Costmaps, Path Planning and Motion Control:
    - Nav2 shall use LiDAR sensors to create a neutral 2D map of the space with black representing obstacles, white representing free space and grey representing unknown objects. This shall be done through Global Costmaps (developing a path through the space over time) and Local Costmaps (short-term path planning around obstacles and motion adjustment). This shall be sent via linear and angular velocities of the robot actuators (convert energy into motion).


## Power Subsystem

#### *Purpose and Functions:*
The Power Subsystem shall store, switch, condition, distribute, and monitor electrical energy for the robot. It: 
  - Shall receive energy from the main battery pack.
  - Shall implements a hard Emergency Stop (E-Stop) and a soft Start/Stop control path.
  - Shall distributes raw VBAT to high-power loads (H-bridges / drivetrain) via a protected Power Bus.
  - Shall generate regulated rails for compute and auxiliaries (Jetson/global controller, sensors, servos, actuators).
  - Shall monitor voltage, current, temperature, and faults; report power-health to the Global Controller; enforce safe shutdown.

#### *Operating Modes:*
1. Off – No rails energized; only charger/measurement (if fitted) alive.
2. E-Stop – Mechanical/electrical isolation of all outbound power; stored energy safely discharged to below safe levels.
3. Pre-Charge/Arming – Controller rail up; buses checked (OV/UV/OC/short) before enabling high-power bus.
4. Run – All required rails enabled, with continuous communications and protections.
5. Fault – Any latched fault disables affected rail(s); posts fault code to controller; requires operator reset. 

#### *Power Architecture:*
  - Source: Battery pack (TBD exact capacity)
  - High-Power Bus (VBUS_HI): VBAT through main fuse, E-Stop, and Start/Stop switch to H-bridges.
  - Regulated Rails:
    - +5 V Compute (5V_SYS): for Jetson Nano/Global Controller (10–25 W; Jetson peak ~5 V @ 4–5 A.
    - +5 V Aux (5V_AUX): for sensors and low-power peripherals (budget 1–2 A).
    - +6–7.4 V Servo Rail (SERVO_V): or +5–6 V depending on servo spec.
    - +12 V Aux. if needed for fans/relays (budget 1–2 A).
    - Step-Down Stages: buck converters from VBAT to rails above.
    - Return: Single-point star ground near battery negative; high-current returns separated from signal grounds.

#### *Interfaces:*
##### Power Interfaces
  - BAT_IN – Power; input; VBAT. Connector: SB50/XT60 (TBD).
  - VBUS_HI_OUT – Power; output to H-bridges. Max continuous current TBD (size for drivetrain peak + 30% headroom).
  - 5V_SYS_OUT – Power; output to Jetson & Global Controller; 5 V regulated, ripple <50 mVp-p @ 1 A; transient load step 2 A/µs compliant.
  - 5V_AUX_OUT / 12V_AUX_OUT / SERVO_V_OUT – Power; outputs; currents per budget.

##### Control
  - EN_HI_BUS – Logic; input from Global Controller; enables high-power contactor/FET (active-high, 3.3/5 V tolerant).
  - PG_5V_SYS / PG_SERVO / PG_HI_BUS – Logic; outputs; power-good for each rail (open-drain, active-low).
  - ESTOP_CHAIN – Safety loop; series circuit through mushroom E-Stop; opens to force hard disconnect.
  - I²C_PWR  – Digital comms to Global Controller for communications and configuration (address/protocol map below).
    - Communication topics: VBAT, rail voltages, rail currents, temperatures, fault codes, energy used (coulomb count).
    - Commands: enable/disable rails, clear faults, set soft-start/limits.

#### *Power Budget:*
  - Drivetrain (via H-bridges): TBD per motor.
  - Jetson Nano: 5–20 W typical; short peaks to ~25 W depending on peripherals and mode.
  - Sensors: 1–5 W.
  - Servos/actuators: highly variable, TBD.


#### *Protections & Safety:*
  - Input fuse sized to protect wiring from fault.
  - Reverse-polarity protection at BAT_IN(Diode).
  - E-Stop performs isolation of VBUS_HI and disables all regulators (except optional always-on monitor) within ≤50 ms.
  - Soft-start / inrush limiting for each rail; pre-charge for bulk caps on VBUS_HI.
  - OV/UV/OC/OTP on every regulator; short-circuit proof on aux rails.
  - Brown-out handling: on VBAT < threshold, assert PG low, signal controller, and perform staged shutdown (compute → servos → drivetrain).
  - EMC: input LC filter, layout segregation, TVS on external connectors.

#### *Environmental & Mechanical:*
  - Operating temp: TBD (target -10 °C to +50 °C); reduce the power currents at elevated temps.
  - Connectors; rails labeled; color-coded wiring; serviceable fuses.

#### *Verification & Test:*
  - Bench bring-up with programmable load: validate regulation, ripple, efficiency across load/temperature.
  - Fault injection (shorts/overloads) to confirm OC trip and recovery.
  - E-Stop timing test (scope PG lines vs. supply collapse).
  - EMI check (conducted ripple on VBAT; sensor bus integrity during motor switching).

#### *Shall Statements:*
1. The subsystem shall accept a battery input in the range TBD (e.g., 10–25 V) and provide isolation of high-power outputs when E-Stop is activated.
2. The subsystem shall provide a 5 V_SYS rail capable of ≥5 A continuous.
3. The subsystem shall provide a SERVO_V rail (voltage TBD per servo spec) rated for ≥10 A peak with independent over-current limiting and brown-out isolation from 5 V_SYS.
4. The subsystem shall distribute VBUS_HI to the drivetrain H-bridges with a current capacity ≥ (sum of motor peak currents × 1.3) and include pre-charge/inrush control.
5. The subsystem shall implement OV, UV, OC, and OTP protections on all regulated rails and shall latch-off or fold-back according to the documented table of limits.
6. The subsystem shall expose Power-Good (PG) signals for VBUS_HI, 5 V_SYS, and SERVO_V, and these PG signals shall stop within ≤50 ms of any fault or E-Stop.
7. The subsystem shall publish VBAT voltage, battery current, rail currents/voltages, temperatures, and fault codes at ≥10 Hz over I²C (400 kHz) to the Global Controller.
8. The subsystem shall support a software-controlled enable for each rail and a hardware Start/Stop input; hardware controls shall override software.
9. The subsystem shall maintain compute power long enough for an orderly shutdown: ≥2 s hold-up on 5 V_SYS at 2 A after brown-out detection.
10. The subsystem shall meet wire gauge and connector ratings for the specified continuous and peak currents with ≥20% thermal margin.
11. The subsystem shall be serviceable: fuses accessible, connectors  and all rails clearly labeled.

#### *Open Items / To-Be-Determined:*
  - Final battery, voltage, and capacity.
  - Exact Jetson Nano SKU and max load (sets 5 V_SYS rating).
  - Servo and actuator quantity and type, TBD mechanical application.
  - Connector part numbers; fuse ratings; wire gauges; thermal solution.

Block Diagram for Power Subsystem: 


![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Power_and_Drivetrain__Hardware_Block_Diagram.png)


## Low-Level Controller (LLC) Subsystem
The Low-Level Controller (LLC) subsystem shall execute motion control instructions from the Global Controller and directly manage the electrical components that correspond with the ground robot’s movement. By managing the local controls, the LLC shall reduce the computational load on the global subsystem and enable the robot’s movement to have a more involved feedback system. The LLC shall receive signals from the power subsystem to operate all circuitry and connections utilized in the system. The power subsystem shall also house the emergency stop (E-stop) feature of the ground robot, as required by competition safety regulations.  

Using the information received from the Global Controller and Power subsystems, the LLC shall process different inputs, translate instructions, and calculate appropriate movement commands via an Arduino. Then the microcontroller shall direct the motion of the actuators. A sensor shall continuously track real-time performance and return feedback to the controller. The feedback shall help mitigate any deviations to ensure efficient and smooth movement.  

The LLC interfaces very closely with the mechanical engineering team. Much of the design’s specifications shall depend on the drivetrain, torque, and mechanical layout. The continued coordination between the electrical and mechanical engineering teams shall determine distribution, placements, connector types, ultimate electrical components, and safety protocol necessary for a successful UAV-robot design. 

### LLC Connection Block Diagrams: 
Though each subsystem is a complex and vital part of the UAV-robot integrated system, Diagram 1 only shows each subsystem’s impact on the LLC directly. Diagram 2 focuses on the internal relationships within the LLC subsystem itself. 

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/LLC_Block_Diagram_1.png)

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/LLC_Block_Diagram_2.png)

### Main Sections of the Specific LLC Subsystem:
This subsection shall delve into the different components present with the LLC subsystem, as depicted in Diagram 2.  

#### (1) Microcontroller: Arduino
The microcontroller shall act as a designated motion controller, or “second brain,” in the robot, directly controlling the motors and all related equipment of the ground robot. An Arduino shall be used to receive and interpret commands from the Global Controller, apply any local algorithms, and generate corresponding pulse-width modulation signals. The implemented code shall consider several variables such as resistance, inductance, inertia, friction, voltage (from the power subsystem), and torque to properly account for the speed, direction, and required power output needed to complete the different tasks on varying terrain. For example, navigating flat surfaces of the arena shall require different parameters than interacting with the crater. As more collaboration and coordination is achieved with the mechanical engineers, a final decision as to the specific Arduino model shall be made. 

#### (2) Actuators: Motors
The actuators shall be motors that convert electrical energy into the mechanical rotation that produces the physical mobility of the robot. In line with the ethical and environmental responsibility of professional engineers, Tennessee Tech’s SECON team shall reuse the two Pololu Metal Gearmotors [18] used in the previous year’s design. Since this year’s robot shall have four wheels, two more identical Pololu DC motors will be acquired. Each reused motor shall be tested to verify it is still within manufacturer specifications. 

#### (3) Sensors: Encoders
The sensors shall be encoders mounted to each motor. The encoders shall collect the real-time output and performance of the actuators so that adjustments can be made. The data collected shall allow for the correction of discrepancies, such as unequal torque distribution causing drift or veering. In this way, the robot shall act as a semi-self-correcting system that can successfully adapt to perform tasks within the three-minute time limit.  

#### Verification and Adaptability
The LLC shall be designed with an emphasis on modularity so that each component can be tested and independently changed if deemed necessary due to design efficiency or product failure. Early testing shall include verification of proper communication between the Global and Low-Level controllers, as well as the connections within the LLC itself. As collaborative efforts with the mechanical team progress, additional testing shall ensure proper integration of the LLC’s electrical systems to its mechanical counterparts, such as the wheels and drivetrain components. 


## Ethical, Professional, and Standards Considerations



## Resources
The team shall leverage a comprehensive set of resources to ensure a successful design, build, and competition run.

#### Human Resources:

The project is supported by Dr. Canfield (faculty advisor), who provides technical oversight and design feedback, and Dr. Johnson (customer/sponsor), who ensures alignment with the competition’s objectives. Each team member contributes specialized skills in mechanical design, electronics, printed circuit board development, and software engineering. The team also benefits from the experience of Dakota, a previous year’s contestant, who provides valuable insight into competition strategy, as well as Dr. Tristan Hill, who offers expertise in the Robot Operating System (ROS) for robot programming.

#### Subsystem Responsibilities and Estimated Time for Completion: 

| **Subsystem**             | **Lead(s)**                                | **Est. Time** |
|----------------------------|--------------------------------------------|----------------|
| **Global Controller**      | Jane                                       | 4–9 weeks |
| **Low-Level Controller**   | Angela & Torsten                           | 4–9 weeks |
| **Communication**          | Aiden                                      | 4–9 weeks |
| **Navigation**             | Atra-Niese                                 | 4–9 weeks |
| **Power**                  | Torsten                                    | 4–9 weeks |
| **Object Detection**       | Trevor                                     | 4–9 weeks |
| **Mechanical**             | Angela & Jane (in coordination with ME team) | 4–9 weeks |

#### Facilities and Support
The team shall utilize the Capstone Lab for prototyping, fabrication, and testing, which includes access to 3D printing, soldering, and electronic diagnostic equipment. Additional support will be provided through official practice boards, hardware rooms, and staging areas supplied by the IEEE SoutheastCon competition for final validation and calibration.

#### Material & Hardware Resources 
The project shall use materials outlined in the official Bill of Materials (BOM), including arena construction supplies, mechanical components, and 3D-printed parts such as antennas, craters, and the Earth module. The electronics foundation shall include Arduino boards, LEDs, sensors, batteries, and wiring kits. The team has elected to purchase an open-source drone to serve as the UAV platform, allowing for modular integration with our communication and navigation subsystems. The team also has access to previous years’ BOMs and IEEE competition resources, providing reference data for cost estimates, component sourcing, and system optimization. 

#### Documentation & Knowledge Resources 
The team shall utilize the official competition ruleset, CAD models, wiring diagrams, and assembly guides as primary design references. Access to KiCAD shall support schematic and PCB design, while NEC and IEEE standards shall ensure compliance with safety, electrical, and professional engineering practices. 

### Budget and Early Prototyping:
The team shall develop a budget by estimating costs for each subsystem. Early prototyping shall focus on the Global Controller, Communication, and Object Detection subsystems, as these represent the most complex integration points. Critical unknowns include sensor calibration accuracy, UAV-to-robot data transfer reliability, and autonomous navigation precision. These shall be addressed through iterative prototyping and testing in the Capstone Lab. 

| **Subsystem**        | **Est. Cost** |
|----------------------|---------------|
| Global Controller    | $250–$500     |
| Low-Level Controller | $220–$440     |
| Communication        | $100–$200     |
| Navigation           | $120–$240     |
| Object Detection     | $80–$200      |
| Power                | $250–$500     |
| **Total**            | **$1020–$2080** |


### Division of Labor:


### Timeline:

| **Task**               | **Project Lead**     | **Start Date** | **End Date** | **Days** |
|------------------------|----------------------|----------------|---------------|-----------|
| Project Proposal       | Team                 | 9/13/2025      | 10/6/2025     | 23 |
| Conceptual Design      | Team                 | 10/5/2025      | 10/27/2025    | 22 |
| Detailed Design        | Team                 | 10/20/2025     | 12/3/2025     | 44 |
| Design Presentation    | Team                 | 12/1/2025      | 12/8/2025     | 7 |
| Global Controller      | Jane                 | 10/18/2025     | 12/6/2025     | 49 |
| Low-Level Controller   | Angela & Torsten     | 11/1/2025      | 12/13/2025    | 42 |
| Communication          | Aiden                | 10/18/2025     | 12/6/2025     | 49 |
| Navigation             | Atra-Niese           | 11/1/2025      | 12/13/2025    | 42 |
| Power                  | Torsten              | 11/1/2025      | 12/13/2025    | 42 |
| Object Detection       | Trevor               | 10/18/2025     | 12/6/2025     | 49 |
| Mechanical             | Angela & Jane        | 11/1/2025      | 12/13/2025    | 42 |



## References
[1] mjs513, “Another T3.5 Rover with a OpenMV Camera (Machine Vision),” Teensy Forum, Aug. 11, 2017. https://forum.pjrc.com/index.php?threads/another-t3-5-rover-with-a-openmv-camera-machine-vision.45741/ (accessed Oct. 29, 2025).

[2] R. Mitchell, “Best SBCs for AI Projects in 2024: Comprehensive Guide,” Electromaker.io, Mar. 27, 2024. https://www.electromaker.io/blog/article/the-ultimate-guide-to-single-board-computers-for-ai-applications?srsltid=AfmBOooR42sNiD1Ac8Vz5HhEcTHXqnfuTKQgxJdXuL6CxgFoQY8jOzXm (accessed Oct. 29, 2025).

[3] Raspberry Pi Ltd, “Buy a Raspberry Pi AI HAT+ – Raspberry Pi,” Raspberry Pi, 2024. https://www.raspberrypi.com/products/ai-hat/

[4] G. Velrajan, “Nvidia Jetson Nano vs Raspberry Pi - Which one is better for your project?,” www.socketxp.com, Jan. 30, 2025. https://www.socketxp.com/iot/nvidia-jetson-nano-vs-raspberry-pi-which-one-is-better-for-your-project/ 

[5] NVIDIA, “NVIDIA jetson nano,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/ 

[6] NVIDIA, “NVIDIA Jetson TX2: High Performance AI at the Edge,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-tx2/

[7] NVIDIA, “NVIDIA Jetson AGX Orin,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/ 

[8] “F24_Team1_SECON/Reports/DetailedDesignNavigationAndMasterControl.md at main · TnTech-ECE/F24_Team1_SECON,” GitHub, 2025. https://github.com/TnTech-ECE/F24_Team1_SECON/blob/main/Reports/DetailedDesignNavigationAndMasterControl.md (accessed Oct. 29, 2025).

[9] TnTech-ECE, “F24_Team1_SECON/Reports/Detailed-Design-Camera.md at main · TnTech-ECE/F24_Team1_SECON,” GitHub, 2025. https://github.com/TnTech-ECE/F24_Team1_SECON/blob/main/Reports/Detailed-Design-Camera.md 

[10] NVIDIA, “Isaac ROS,” NVIDIA Developer. https://developer.nvidia.com/isaac/ros 

[11] L. Fang, “RGB cameras: Definition, components, and integration,” TechNexion, Oct. 11, 2024. https://www.technexion.com/resources/rgb-cameras/ 

[12] A. Rehman, “Beyond the Surface: Advanced 3D Mesh Generation from 2D Images in Python,” Medium, Feb. 16, 2024. https://medium.com/red-buffer/beyond-the-surface-advanced-3d-mesh-generation-from-2d-images-in-python-0de6dd3944ac (accessed Oct. 29, 2025). 

[13] P. Kumar, “What are RGBD cameras? Why RGBD cameras are preferred in some embedded vision applications? – e-con Systems,” e-con Systems, May 19, 2022. https://www.e-consystems.com/blog/camera/technology/what-are-rgbd-cameras-why-rgbd-cameras-are-preferred-in-some-embedded-vision-applications/ 

[14] A. Szczepaniak, “Leo Rover Blog - How is LiDAR used in Robotic Navigation? Pros and Cons,” www.leorover.tech, Jan. 16, 2023. https://www.leorover.tech/post/how-is-lidar-used-in-robotic-navigation-pros-and-cons 

[15] “How Do Robots Find Their Way? A Deep Dive into Navigation Sensors,” TechNexion, Aug. 08, 2025. https://www.technexion.com/resources/how-do-robots-find-their-way-a-deep-dive-into-navigation-sensors/ 

[16] A. Ayodele, “Types of Sensors in Robotics,” www.wevolver.com, Jan. 16, 2023. https://www.wevolver.com/article/sensors-in-robotics 

[17] A. Szczepaniak, “Leo Rover Blog - Top common ways to localize a mobile robot,” Leorover.tech, 2023. https://www.leorover.tech/post/top-common-ways-to-localize-a-mobile-robot 

[18] “Pololu - 6V High-Power (HP) 25D mm Gearmotors,” Pololu.com, 2025. https://www.pololu.com/category/183/6v-high-power-hp-25d-mm-gearmotors (accessed Oct. 30, 2025). 

[19] Texas Instruments. (2023). Understanding Boost Power Stages in Switch Mode Power Supplies (SLVA372B). 

[20] Texas Instruments. (2022). Handling Regenerative Energy in Motor Drive Systems. 

[21] Texas Instruments. (2023). Power Design Seminar SEM2300 Notes. 

[22] Analog Devices. (2020). Buck, Boost, and Buck-Boost Converter Fundamentals. 

[23] Microchip Technology. (2011). AN1149 – Battery Boost Converter Design Guide. 

[24] Infineon Technologies. (2020). AN 2020-06: Energy Recovery in Motor Control Systems. 

[25] International Organization for Standardization (ISO). (2015). ISO 13849-1: Safety of Machinery – Safety-related Parts of Control Systems. 

[26] International Electrotechnical Commission (IEC). (2005). IEC 60950-1: Information Technology Equipment – Safety – Part 1: General Requirements. 



## Statement of Contributions

John Land – Resources section, Power Atomic Subsystem Specifications 

Jane Vasar – Global Controller High Level Solution and Subsystem. 

Trevor Snyder - Object Detection Subsystem


