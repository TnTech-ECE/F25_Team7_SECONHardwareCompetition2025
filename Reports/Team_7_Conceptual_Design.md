# Conceptual Design

This document outlines the objectives of a conceptual design. After reading your conceptual design, the reader should understand:

- The fully formulated problem.
- The fully decomposed conceptual solution.
- Specifications for each of the atomic pieces of the solution.
- Any additional constraints and their origins.
- How the team will accomplish their goals given the available resources.

With these guidelines, each team is expected to create a suitable document to achieve the intended objectives and effectively inform their stakeholders.


## General Requirements for the Document
- Submissions must be composed in Markdown format. Submitting PDFs or Word documents is not permitted.
- All information that is not considered common knowledge among the audience must be properly cited.
- The document should be written in the third person.
- An introduction section should be included.
- The latest fully formulated problem must be clearly articulated using explicit "shall" statements.
- A comparative analysis of potential solutions must be performed
- The document must present a comprehensive, well-specified high-level solution.
- The solution must contain a hardware block diagram.
- The solution must contain an operational flowchart.
- For every atomic subsystem, a detailed functional description, inputs, outputs, and specifications must be provided.
- The document should include an acknowledgment of ethical, professional, and standards considerations, explaining the specific constraints imposed.
- The solution must include a refined estimate of the resources needed, including: costs, allocation of responsibilities for each subsystem, and a Gantt chart.


## Introduction

The introduction is intended to reintroduce the fully formulated problem. 


## Restating the Fully Formulated Problem

The fully formulated problem is the overall objective and scope complete with the set of shall statements. This was part of the project proposal. However, it may be that the scope has changed. So, state the fully formulated problem in the introduction of the conceptual design and planning document. For each of the constraints, explain the origin of the constraint (customer specification, standards, ethical concern, broader implication concern, etc).


## Comparative Analysis of Potential Solutions

In this section, various potential solutions are hypothesized, design considerations are discussed, and factors influencing the selection of a solution are outlined. The chosen solution is then identified with justifications for its selection.


## High-Level Solution

This section presents a comprehensive, high-level solution aimed at efficiently fulfilling all specified requirements and constraints. The solution is designed to maximize stakeholder goal attainment, adhere to established constraints, minimize risks, and optimize resource utilization. Please elaborate on how your design accomplishes these objectives.


### Hardware Block Diagram

Block diagrams are an excellent way to provide an overarching understanding of a system and the relationships among its individual components. Generally, block diagrams draw from visual modeling languages like the Universal Modeling Language (UML). Each block represents a subsystem, and each connection indicates a relationship between the connected blocks. Typically, the relationship in a system diagram denotes an input-output interaction.

In the block diagram, each subsystem should be depicted by a single block. For each block, there should be a brief explanation of its functional expectations and associated constraints. Similarly, each connection should have a concise description of the relationship it represents, including the nature of the connection (such as power, analog signal, serial communication, or wireless communication) and any relevant constraints.

The end result should present a comprehensive view of a well-defined system, delegating all atomic responsibilities necessary to accomplish the project scope to their respective subsystems.


### Operational Flow Chart

Similar to a block diagram, the flow chart aims to specify the system, but from the user's point of view rather than illustrating the arrangement of each subsystem. It outlines the steps a user needs to perform to use the device and the screens/interfaces they will encounter. A diagram should be drawn to represent this process. Each step should be represented in the diagram to visually depict the sequence of actions and corresponding screens/interfaces the user will encounter while using the device.


## Atomic Subsystem Specifications

Based on the high-level design, provide a comprehensive description of the functions each subsection will perform.

Inclued a description of the interfaces between this subsystem and other subsystems:
- Give the type of signal (e.g. power, analog signal, serial communication, wireless communication, etc).
- Clearly define the direction of the signal (input or output).
- Document the communication protocols used.
- Specifying what data will be sent and what will be received.

Detail the operation of the subsystem:
- Illustrate the expected user interface, if applicable.
- Include functional flowcharts that capture the major sequential steps needed to achieve the desired functionalities.

For all subsystems, formulate detailed "shall" statements. Ensure these statements are comprehensive enough so that an engineer who is unfamiliar with your project can design the subsystem based on your specifications. Assume the role of the customer in this context to provide clear and precise requirements.

### Object Detection Subsystem
The drone shall act as an overhead observer while the robot acts as the main ground unit. The drone’s job is to locate and keep track of the robot while navigating it throughout the course.  

By doing this the drone and robot shall do the following: 
1. The drone shall create a SLAM map of the competition within 20 seconds  
2. The drone shall locate and navigate the robot 
3. The drone and robot shall both locate the Astro-Ducks and antennas 
4. The drone and robot shall both identify the specific task of the antennas 
5. The drone shall determine the color of the antennas’ LEDs 
6. The drone and robot shall automatically starting using the LED bar on the competition board 

##### *Creating a SLAM Map:*
To achieve reliable localization and mapping within the strict three-minute match limit, the drone shall utilize a geometry-based SLAM initialization approach using the specification given by the competition ruleset. According to the official ruleset, the field consist of a rectangular 4’ by 8’ plywood base surrounded by 1” by 8” by 8’ border walls. This playing surface also includes a 2’ diameter crater with an 8” flat area near the center of the arena which provides a distinct landmark for visual recognition. These fixed dimensions enable the drone to initialize SLAM instantaneously using a pre-defined arena model. Thus, eliminating the need for extended exploration that would take up more time than necessary.  Below are the steps for which this quick and efficient SLAM map is created within the first 20 seconds of the run. 

1. Pre-Run Setup
    -  A prior map of the arena is generated from the published arena specifications. This map encodes the outer 4’ by 8’ boundary, crater footprint and any other static features that have a fixed location. Since the rules allow full autonomy and unrestricted use of sensors and software, this pre-built model is fully allowed.

2. Initialization and Instant Localization (0-3 seconds)
    - The drone will instantly take off from the robot and will position itself at the bottom left corner of the arena where the staring area is. Using the drone’s camera, the system shall detect the wall and floor boundaries then compute a camera-to-floor homography. This geometric fitting aligns the drone’s camera frame with the pre-defined	 global coordinate frame of the map, providing instantaneous metric scale and orientation without any markers.

3. Arena Mapping (3-12 seconds)
    - After alignment, the drone will make a single high-speed pass along the perimeter of the competition arena followed by a figure-eight pass near the center. Visual-inertial odometry (VIO) shall combine the camera and IMU to track the motion between the frames captured by the camera. At the same time, the onboard SLAM system shall triangulate limited 3D points and updates a occupancy grid with around 5 cm of resolution. Due to the pre-defined arena geometry, this step focuses on the refining accuracy and identifying small environmental variations.

4. Drift Correction and Map Closure (12-18 seconds)
    - Once returning to the starting corner, the drone re-observes that same arena walls and performs layout-based loop closure. This is to align the live map back to the known rectangular competition field. This correction ensures that positional drift accumulated during the short flight is reduced within cm of the real scale.

5. Map Output (18-20 seconds)
    - Finally within 20 seconds, the drone produced a rough but globally accurate map. The mapping will continue in the background as the run goes along, increasing map density while maintaining localization for coordination with the robot.

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/SLAM_Map_FLowchart.png)

##### *Locating and Navigating the Robot:*
After the initial SLAM map is generated by the drone, the robot shall perform autonomous localization and navigation within that map while identifying objects inside the competition arena. Objects such as the Astro-Ducks and antennas shall be identified using vision-based object detection algorithms. Below are the steps using algorithms for the robot to locate itself and start making its way through the course.

1. Initialization and Localization
    - At the start of the run, the robot is deployed inside the 12” by 12” by 12” staring cube. The robot shall use the SLAM map generated by the drone to establish its initial position. This shall be done by using the onboard LiDAR sensors and vision camera to compare the nearby walls with those of the known 4’ by 8’ board dimensions. By using a Pose Graph Optimization algorithm, the robot’s local coordinate frame shall be aligned with the SLAM map. This localization state is maintained through sensor fusion in an Extended Kalman Filter (EKF) that shall combine the data from the wheel encoders, IMU, and LiDAR sensors. This maintains a true position of the robot even if the data is imperfect.

2. Landmark-Detection Algorithm
    - A CNN-based object detection model shall be implemented using similar architecture to the YOLOv5-nano. The robot’s camera shall capture frames which are resized and normalized. The YOLOv5-nano shall perform bounding-box predictions with labels for the Astro-Ducks, antennas, and the crater edge. Suppression shall help eliminate duplicate detections which centroid coordinates shall then be projected into the world space. The detected objects shall then be fed into the localization module as landmarks.
  
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Robot_Localization_Navigation_Flowachart.png)

##### *Locating Astro-Ducks and Antennas:*
To efficiently locate the Astro-Ducks and antennas, a dual-layer detection algorithm shall be used that shall combine the drone’s aerial scanning with the robot’s ground level conformation. Together the drone and robot shall maintain the shared SLAM map that is continuously evolving.

1. Drone-Based Aerial Scanning
    - During the beginning SLAM mapping in the first 20 seconds of the run, the drone shall run a lightweight version of the YOLOv5-Nano model tuned for overhead imagery. The drone shall be able to observe the entire field from above and recognize objects based off their characteristics.  The Astro-Ducks shall appear as small, bright circular blobs with soft edges while the antennas shall appear as taller structures with clear circular dishes. Each time the drone detects an object, it shall estimate its approximate center pixel and convert them into real-world coordinates relative to the arena. 

2. Robot-Based Close-Range Confirmation
    - As the robot drives towards the coordinates received from the drone, it shall activate the front facing camera and short-range sensors to begin detailed inspection. The robot shall run the same algorithm as the drone but with a model trained for ground-level perspectives. From this distance, the robot shall clearly identify the shapes and textures of the object. The robot shall detect either short shapes with high color saturation or taller objects with dishes at the top. By combining the shape and colors with the object’s height, the robot shall eliminate the chances of a misclassification.
  
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Locating_AstroDucks_and_Antennas.png)

##### *Identifying the Antenna's Task:*
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

##### *Detecting the Starting LED:*
At the beginning of the competition, the robot and drone shall start automatically when the arena’s white “Start LED” bar illuminates. To accomplish this, the robot shall be equipped with a photoresistor (LDR) mounted in a small black tube aimed at the “Start LED” region. The black tube shall act as a light baffle, blocking any unnecessary light from overhead fixtures. When the “Start LED” turns on, the brightness in the sensor’s field of view shall sharply increase causes the voltage of the system to change. Once the voltage shall exceed a set threshold, the software shall confirm that the “Start LED” has been detected. To prevent false triggers, the robot shall use a short debounce delay, requiring the signal to stay above the set threshold for a couple hundred milliseconds before starting.

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Poster%20Template/Images/Detecting_Starting_LED.png)




## Ethical, Professional, and Standards Considerations

In the project proposal, each team must evaluate the broader impacts of the project on culture, society, the environment, public health, public safety, and the economy. Additionally, teams must consider relevant standards organizations that will inform the design process. A comprehensive discussion should be included on how these considerations have influenced the design. This includes detailing constraints, specifications, and practices implemented as a result, and how these address the identified considerations.


## Resources

You have already estimated the resources needed to complete the solution. Now, let's refine those estimates.

### Budget

Develop a budget proposal with justifications for expenses associated with each subsystem. Note that the total of this budget proposal can also serve as a specification for each subsystem. After creating the budgets for individual subsystems, merge them to create a comprehensive budget for the entire solution.

### Division of Labor

First, conduct a thorough analysis of the skills currently available within the team, and then compare these skills to the specific requirements of each subsystem. Based on this analysis, appoint a team member to take the specifications for each subsystem and generate a corresponding solution (i.e. detailed design). If there are more team members than subsystems, consider further subdividing the solutions into smaller tasks or components, thereby allowing each team member the opportunity to design a subsystem.

### Timeline

Revise the detailed timeline (Gantt chart) you created in the project proposal. Ensure that the timeline is optimized for detailed design. Address critical unknowns early and determine if a prototype needs to be constructed before the final build to validate a subsystem. Additionally, if subsystem $A$ imposes constraints on subsystem $B$, generally, subsystem $A$ should be designed first.


## References

All sources utilized in the conceptual design that are not considered common knowledge must be properly cited. Multiple references should be included.


## Statement of Contributions

Each team member is required to make a meaningful contribution to the project proposal. In this section, each team member is required to document their individual contributions to the report. One team member may not record another member's contributions on their behalf. By submitting, the team certifies that each member's statement of contributions is accurate.
