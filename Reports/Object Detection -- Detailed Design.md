# Detailed Design

This document delineates the objectives of a comprehensive system design. Upon reviewing this design, the reader should have a clear understanding of:

- How the specific subsystem integrates within the broader solution
- The constraints and specifications relevant to the subsystem
- The rationale behind each crucial design decision
- The procedure for constructing the solution


## General Requirements for the Document

The document should include:

- Explanation of the subsystem’s integration within the overall solution
- Detailed specifications and constraints specific to the subsystem
- Synopsis of the suggested solution
- Interfaces to other subsystems
- 3D models of customized mechanical elements*
- A buildable diagram*
- A Printed Circuit Board (PCB) design layout*
- An operational flowchart*
- A comprehensive Bill of Materials (BOM)
- Analysis of crucial design decisions

*Note: These technical documentation elements are mandatory only when relevant to the particular subsystem.


## Function of the Subsystem

The function of this subsystem is for objects and objectives to be detected throughout the course to efficiently communicate with the robot to navigate with ease. By using different object detection algorithms the drone and robot shall both be able to detect objects such as the antennas, tasks of antennas, antenna LED colors, Astro-Ducks, starting LED, and the layout of the course. These algorithms shall share different types of data that shall be wirelessly communicated with the robot and the Earth base using both WiFi and Bluetooth. Each of these algortithms shall be discussed in much greater detail later in the document.


## Specifications and Constraints

This section should provide a list of constraints applicable to the subsystem, along with the rationale behind these limitations. For instance, constraints can stem from physics-based limitations or requirements, subsystem prerequisites, standards, ethical considerations, or socio-economic factors.

The team should set specifications for each subsystem. These specifications may require modifications, which must be authorized by the team. It could be necessary to impose additional constraints as further information becomes available.

Every subsystem must incorporate at least one constraint stemming from standards, ethics, or socio-economic factors.


### Specifcations:

1. The drone and robot shall create a SLAM map of the competition within 20 seconds
2. The drone shall locate and navigate the robot
3. The drone and robot shall both locate the Astro-Ducks and antennas
4. The drone and robot shall both identify the specific task of the antennas
5. The drone shall determine the color of the antennas’ LEDs
6. The drone and robot shall automatically starting using the LED bar on the competition board

### Constraints: 
1. The drone shall not weigh more than 250 grams or 0.55 pounds
2. The drone shall not move outisde of the netted playing field
3. The robot and drone shall have a maximum of 3 minutes to complete objectives and score points
4. The drone shall properly identify the antennas' LEDs and tranmit the correct data to Earth
5. The robot shall automatically starts using the white starting LED


## Overview of Proposed Solution

In order to meet the specifications and constraints, object detection is broken up into two main groups. The first being the drone's object detection system with the other being the robot's object detection.


##### Drone Onject Detection:

To meet the listed specifcation and constrains above, the drone shall use the ESP32-S3 Sense. This camera is capable of detecting the crater edges, antenna-tower location, duck-like shapes, colored objects, and AprilTags. The ESP32-S3 allows the drone and robot to create a rough SLAM map of the competition field within the first 20 seconds (Specification #1), locate the robot using AprilTag detection (Specification #2), locate the Astro-Ducks and antennas (Specification #3), and the color of the antennas' LED (Specification #5). The camera is very lightweight with a weight of 6.6 grams to help stay within 250 grams limit of the drone (Constraint #1).

The image below is of the ESP32-S3 Sense:

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/ESP32_S3.png)


##### Robot Object Detection: 

To meet the listed specifications and constraints above, the robot shall use the Intel RealSense D435 RGBD camera and the Garmin LIDAR-Lite v4 LED sensors. The Intel RealSense camera shall be used to further confirm the robots location in the map and confirm the identifty of the Astr-Ducks and antennas (Specifcation #3). The robot shall already know where the antennas are located on the field from the pre-created map of the competition arena. By already knowing which antenna is where on the competition arean, the specific task of each antenna is already known. The robot shall navigate to drone where it uses it onboard Intel RealSense camera and Garmin LiDAR sensors to correctly position and algin itself with whichever antenna task is at hand (Specification #4).The robot shall aslo be equipped with a 5mm GL5528 Photoresistor to detect the starting LED for autonomous starting (Specification #6).

The image below is of the Intel RealSense:

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/Intel%20RealSense%20D435.jpg)


The image below is of the Garmin LiDAR:

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/Garmin%20LiDAR%20Lite%20v4.webp)



## Interface with Other Subsystems

The Object Detection subsystem communicates extensively with nearly every major subsystem within the robot–drone architecture. Its primary role is to convert raw camera data into meaningful, actionable information such as object identity, object location, antenna LED color, and task classification. The following subsections provide detailed descriptions of all inputs, outputs, data formats, communication methods, and the exact purpose of each data transfer.

### *Interface with Drone Vision:*

##### Inputs to Object Detection
  - Low-resolution grayscale frames (160x120) for SLAM initalization
  - High-resolution RGB frames (320x240 or 640x480) for object detection

##### Communication Method
  - Wi-Fi UDP or TCP stream using the drone's ESP32-S3
  - Images are packeted and fowarded to robot's communication subsytem

##### Purpose
  - Provide real-time ariel imagery of the areana
  - Detect the Astro-Ducks, antennas, crater, and basic arena geometry
  - Initialize SLAM map for global controller subsystem

### *Interface with Communication Subsystem:*

##### Inputs to Object Detection
  - Image packets from the Crazyflie 2.1+ drone
  - Robot Intel RealSense camera frames
  - Status messages from global controller

##### Outputs from Object Detection
  - Compressed detection results
      - Object class (duck, antenna, dish, ...)
      - Bounding box coordinates (x_min, y_min, width, height)
      - Confidence score
      - Object height classification
      - LED color classification (red, blue, green, white)
  - Feature points for SLAM map
      - ORB/FAST keypoints
      - Known landmark coordinates (crater rim, walls, antennas)

##### Communication Method
  - ROS2 topics or TCP messaging to the communicatins stack
  - JSON or Protobuf for lightweight transmission

##### Purpose
  - Allows global controller to merger detected objects with navigation and SLAM data
  - Maintain consistent communucation between drone, robot sensors, and processing nodes.

### *Interface with Global Controller:*

##### Outputs to Global Controller
  - Object Identification Data
      - Object type: duck or antenna
      - Task type inference: button, pressure plate, crank, keypad
  - Object Position Data
      - Pixel-space coordinates
      - Depth estimation
  - Antenna LED Color
      - Dominant HSV region classification output
      - Timestamping for synchronization
  - SLAM Initialization Features
      - Crater center detection
      - Boundary detections
      - Fiducial points

##### Communication Method
  - ROS2 topic: /object_detection/results
  - Global Controller publishes callbacks
      - "Naviagate to this coordinate"
      - "This object belongs to task #3"
      - "Update global map"

##### Purpose
  - Alows mission-level decision making
  - Support path planning and object prioritization
  - Enables SLAM updates based on visual beaconing

### *Interface with Navigation Subsystem:*

##### Outputs from Object Detection
  - Approximate (x,y) coordinates of detected objects
      - Converted from image-space to world-space using camera calibration
  - Obstacle detections
      - Flags geometric shapes or unexpected objects in the competition arena
  - Distance-to-object estimates

##### Communication Method
  - ROS2 TF messages
  - Pose updates to "/nav/object_positions"

##### Purpose
  - Allows Navigation to generate a path directly to the antenna of duck
  - Real-time obstacle avoidance
  - Link visual detections with wheel odometry and  IMU data


### *Interface with Local Controller:*

##### Outputs
  - Final target pixel alignment values
      - Used for fine-positioning near an object
      - Example: "Object is 24 pixels left of center. Rotate left 3 deg"
  - Target distance
      - Enables precise approach maneuvers when stopping in front of an antenna

##### Communication Method
  - ROS2 topic or serial pass-through direct message

##### Purpose
  - Allow the robot to precisely align itself with the antenna for task completion
  - Ensure smooth docking and payload-level precision

### *Interface with Safety and Power:*

##### Outputs
 - Sends glag if camrea fails or image quality drops

##### Purpose
 - Fail-safe operation
 - Allows the robot to fall back to default waypoint navigation if camera becomes unavailable
    


## 3D Model of Custom Mechanical Components

Should there be mechanical elements, display diverse views of the necessary 3D models within the document. Ensure the image's readability and appropriate scaling. Offer explanations as required.


## Buildable Schematic 

Integrate a buildable electrical schematic directly into the document. If the diagram is unreadable or improperly scaled, the supervisor will deny approval. Divide the diagram into sections if the text and components seem too small.

The schematic should be relevant to the design and provide ample details necessary for constructing the model. It must be comprehensive so that someone, with no prior knowledge of the design, can easily understand it. Each related component's value and measurement should be clearly mentioned.


## Printed Circuit Board Layout

Include a manufacturable printed circuit board layout.


## Flowchart

For sections including a software component, produce a chart that demonstrates the decision-making process of the microcontroller. It should provide an overview of the device's function without exhaustive detail.


## BOM

Provide a comprehensive list of all necessary components along with their prices and the total cost of the subsystem. This information should be presented in a tabular format, complete with the manufacturer, part number, distributor, distributor part number, quantity, price, and purchasing website URL. If the component is included in your schematic diagram, ensure inclusion of the component name on the BOM (i.e R1, C45, U4).

## Analysis

Deliver a full and relevant analysis of the design demonstrating that it should meet the constraints and accomplish the intended function. This analysis should be comprehensive and well articulated for persuasiveness.

## References

All sources that have contributed to the detailed design and are not considered common knowledge should be duly cited, incorporating multiple references.
