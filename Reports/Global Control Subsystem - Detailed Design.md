# Detailed Design

## Function of the Subsystem

The global control system is designed to function as the robot's data collection and control center. This subsystem consist of a single board computer that will be used to compute and process the robot's high level control processes, such as its navigation, object detection, and decision making behaviors. The single board computer will use these processes to send instructions to the low level controller that is responsible for controlling the robot's motors. It will also send commands to the robot's companion UAV. The Global control system also includes the senors that will be used to help collect position data that will be feed to the navigation subsystem.    

## Specifications and Constraints

Listed below are the specifications and constraints for this subsystem as derived from the SECON 2026 rules manual [1].
 
1.  Both the robot and UAV shall be completely autonomous.
2.  The robot and UAV shall have a clearly labeled start switch. 
3.  Teams shall have a maximum of 3 minutes to earn points. 
  a.  The robot shall have the ability to quickly process sensor data.
  b.  The robot shall have the ability to quickly process AI workloads.
  c.  The robot shall have the ability to quickly make decisions and communicate them.
4.  Teams shall rescue a total of 6 Astro-Ducks and shall return them to the “Lunar Landing Area.” 
  a.  The robot shall be able to identify, locate, and navigate to the Astro-Ducks.
  b.  The robot shall be able to identify and navigate to the Lunar Landing Area.
5. Teams shall establish power to the 4 antennas throughout the course. Power shall be restored differently for each antenna. Once power is restored to the antenna, a randomly colored LED shall light up (red, blue, green, and purple).
  a.  The robot shall be able to identify, locate, and navigate to the antenna towers.
  b.  The robot must identify and complete the task located on each tower.
6.  Two starting white LED bars shall be placed on top of the arena wall, one on each side of the 12” x 12” starting area. 
  a.  The robot shall be able to sense the starting white LEDs in order to begin operation.
7.  Points shall be lost every time the robot or UAV has an unintentional collision 
  a.  The robot and UAV shall be able to detect and avoid obstacles.

## Overview of Proposed Solution

The global control system is divided into multiple major components: The Global controller itself and the sensors that it relies on in order to collect the data the robot needs to make decisions. 

The global controller will act as the central computer that will be used for collecting data, processing data, and making decisions based on that data. This computer will be the device that makes all of the control decisions for the robot. It will also host the processes for object detection and navigation. Based on the data that is received and computed, it will make decisions and send the necessary commands to the low-level controller that will coordinate the robot’s motion and actuators. The robot shall use an NVIDIA Jetson Orin for this task. This single board computer was designed to efficiently run AI loads which will be essential for the robot’s object detection and navigation processes.

The robot shall also use a combination of sensors that will allow the robot to perceive its location on the gameboard, and be able to identify the objects on it. LiDAR sensors and an IMU will be used to pinpoint the robot’s location on the game board. The robot shall also utilize photoresistors which will be used to start the robot autonomously. The robot will also have a manual start button. The robot and the UAV will have cameras that will send image data to the global computer over USB and WiFi respectively. More detail about the cameras can be found in the object detection detailed design document [object detection doc].

## Interface with Other Subsystems

The Jetson Orin has access to video encodes, video decoders, CSI camera support, PCIe, USB, 1 gigabit ethernet, display ports, UART, SPI, I2S, I2C, GPIO, PWM, DMIC, and DSPK protocols. The development board comes with a 40-pin header for IO, 2 CSI camera connectors, 4 USB type A connectors, a display port, an ethernet port, and a barrel jack for power [2]. The robot mounted camera will connect to the global computer using a USB connection. The UAV comes with a USB adaptor that will allow the global control computer to communicate with the UAV flight controller. Details for the USB adaptor can be found in the communication detailed design document[comms detailed]. The UAV mounted camera will connect to the global computer over a WiFi connection. The Jetson Orin does not come with built in WiFi, so an adaptor will need to be purchased. The Low Level Controller will connect to the global computer over USB. Both the IMU and the LiDAR sensors will connect to the global computer using I2C. Both the autonomous and manual start signals will be GPIO signals.

Inputs:
* Autonomous Start Signals
  * GPIO Analog Signal
* Manual Start Signal
  * GPIO Digital Signal
* LiDAR Sensors
  * Distance Data over I2C
* IMU
  * Acceleration and Orientation Data over I2C
* Robot Mounted Camera
  * Image and Distance Data over USB
* Drone Mounted Camera
  * Image Data over WiFi
Outputs:
* Command Signals to Low Level Controller
  * Motion and Actuation commands over USB
* Navigation Commands (Coordinates of the designated path to follow)
  * These shall be transmitted over USB
* UAV Command and Data Signals
  * Launch and Land Commands over a radio signal via USB adaptor
  * LED Information over Wifi.



#### Figure 1 - Jeston Orin Top View
<img width="1058" height="466" alt="son" src="https://github.com/user-attachments/assets/aae0ed3f-2c23-46a3-91c5-4c845903eb94" />


#### Figure 2 - Jetson Orin Bottom View
<img width="1084" height="559" alt="jet" src="https://github.com/user-attachments/assets/519fdd8c-3f3a-483f-9307-66d0a9f6ad4d" />


#### Figure 3 - Jetson Orin Block Diagram
<img width="1053" height="833" alt="diagram" src="https://github.com/user-attachments/assets/ed7d0934-27cd-4a91-8047-55d0931bcecf" />

#### Figure 4 - Jetson Orin Header Pinout Diagram
<img width="709" height="1161" alt="pinhead" src="https://github.com/user-attachments/assets/52d15e0d-a1ab-4c2d-a1e2-e43f5689db09" />

## Buildable Schematic

#### Figure 5 - Connections to Jetson IO
<img width="2255" height="1597" alt="scheme" src="https://github.com/user-attachments/assets/5f8a4f5e-36fc-445c-891e-f3fbe23f0a7e" />

## Flowchart

#### Figure 6 - System Block Diagram
<img width="2064" height="1377" alt="Full Block Diagram" src="https://github.com/user-attachments/assets/4c4f986b-03d7-48fe-b823-53fa32cf9fdf" />

#### Figure 7- Software Flowchart 1

<img width="665" height="722" alt="soft 4" src="https://github.com/user-attachments/assets/07d74887-c5e6-4381-9261-d96a6c396d7c" />

#### Figure 8 - Software Flowchart 2

<img width="608" height="1118" alt="soft 3" src="https://github.com/user-attachments/assets/445299ef-aabc-4988-a97b-d758c968ee33" />

#### Figure 9 - Software Flowchart 3

<img width="417" height="1043" alt="soft 2" src="https://github.com/user-attachments/assets/ff4a76a4-f36e-4c28-b0d3-370891e4bf1a" />

#### Figure 10 - Sofrware Flowchart 4

<img width="677" height="917" alt="soft 1" src="https://github.com/user-attachments/assets/310cf1e4-0754-43c7-b672-4b84cfc72ddd" />

## BOM
| Part        | Manufacturer| Part Number | Distributor | Distributo Part Number | Quantity | Price | Total Price | Website URL |
| ----------- | ----------- | ----------- | ----------- | ---------------------- | -------- | ----- | ----------- | ----------- |
| Jetson Orin | NVIDIA | N/A | Amazon | 945-137766-0000-000 | 1 | $249.00 | $249.00 | [Store Link](https://www.amazon.com/dp/B0BZJTQ5YP?utm_source=nvidia&th=1) | 
| Garmin LIDAR-Lite V4 | Garmin | N/A | SparkFun | SEN-18009 | 3 | $119.95 | $359.85 | [Store Link](https://www.sparkfun.com/garmin-lidar-lite-v4-led-distance-measurement-sensor-qwiic.html) |
| 9Dof IMU Breakout | SparkFun | SEN-15335 | SparkFun | SEN-15335 | 1 | $21.95 | $21.95 | [Store Link](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html) |
| GL12528 Photoresistor (10 pack) | Juried Engineering | N/A | Amazon | B08F3WPNPF | 1 | $22.98 | $22.98 | [Store Link](https://www.amazon.com/Juried-Engineering-Photoresistor-GL12528-Sensitive/dp/B08F3WPNPF) |
| Auxiliary Start Switch | Judco Manufacturing Inc. | 40-4325-00 | Digikey | 512PB-ND | 1 | $2.55 | $2.55 | [Store Link](https://www.digikey.com/en/products/detail/judco-manufacturing-inc/40-4325-00/254287) |
| Flexible Qwiic Cable - 200mm | Spark | PRT-17258 | SparkFun | PRT-17258 | 4 | $1.95 | $7.80 | [Store Link](https://www.sparkfun.com/flexible-qwiic-cable-200mm.html) |
| Flexible Qwiic Cable - Female Jumper | SparkFun | CAB-17261 | SparkFun | CAB-17261 | 2 | $1.95 | $3.90 | [Store Link](https://www.sparkfun.com/flexible-qwiic-cable-female-jumper-4-pin.html) |
| QWiic MultiPort | SparkFun | BOB-18012 | SparkFun | BOB-18012 | 1 | $2.50 | $2.50 | [Store Link](https://www.sparkfun.com/sparkfun-qwiic-multiport.html) |
| Jumper Wire Kit | SparkFun | PRT-00124 | SparkFun | PRT-00124 | 1 | $8.95 | $8.95 | [Store Link](https://www.sparkfun.com/jumper-wire-kit-140pcs.html) |
| Right Angle Header - Male | SparkFun | PRT-30218 | SparkFun | PRT-30218 | 4 | $0.75 | $3.00 | [Store Link](https://www.sparkfun.com/right-angle-header-male-pth-0-1in-6-pin.html) |
| USB WiFi Adapter | BrosTrend | N/A | Amazon | B0F6N1H84N | 1 | $23.99 | $23.99 | [Store Link](https://www.amazon.com/BrosTrend-AX900-Linux-WiFi-Adapter/dp/B0F6N1H84N/ref=sr_1_14?dib=eyJ2IjoiMSJ9.78-l3ki-Cdgk66rHmmKHe5qLTTbS9eqaGMqy-N1vxuT9ts6YpoDAxhiYgS5h9aVn9VTi9ZNJwANd-MHeNQAX7OxGamrVIyeR2zN0ZvEaOG2p-lf7OJzYmn8SxOmwP3koahc9B50WKRfAQRArUsQ7JJ-P3BLRCqjkE8z_S6lf0qjZg05xnagBSqaYgHQ1IOcmqJOAubb61hBmwY1hpaqUG-YNwDHtmeGyrqFnx_kKe4M.lKWOEbYNy6bvYiR4wKm4y_ric09EUjYlcziARQIoZ6I&dib_tag=se&keywords=linux+compatible+wifi+adapter&qid=1771211949&sr=8-14) |

## Analysis
### Global Controller
In order to meet specifications 1, 5, and 6; the robot shall have a computer that is capable of receiving incoming data from its sensors, processing that data, and making decisions with that data. The robot shall be able to identify objects like the Astro Ducks and antenna towers and then plan an efficient route to navigate to those objects. Finally the computer shall send the needed instructions to the low level controller in order to execute those decisions. The robot’s ability to detect objects and handle navigation tasks shall depend on its ability to effectively utilize artificial intelligence algorithms. A single board computer would be more suited for managing the robot’s high level control. The robot shall use a Jetson Orin, more specifically a Jetson Orin Nano Super Developer Kit, for this function as it was designed with AI processing in mind. Using a Jeston single board computer over a similar option like a Raspberry Pi was discussed in a previous document [3]. The robot shall use a Jeston Orin instead of the Nano or the TX2 because of its processing power. 

The Jetson Orin is listed as having an AI performance of 67 tera-operation per second (TOPS) [2]. The Nano and the TX2 have an AI performance of 0.472 and 1.33 tera floating point operation per second (TFLOPS) respectively [4] [5]. TOPS and TFLOPS are both metrics that are used to describe AI performance [6] [7], though they are different, with TOPS referring to 8 bit integer operations while TFLOPS refers to floating point operation [8]. However, a general case conversion can be made to compare the two [9]. An exact conversion is hardware architecture dependent, however, using the general case the Orin is shown to be the most powerful of the three listed computers. The previous competition’s robot was able to utilize the Jetson Nano, however, the 2026 competition’s robot will need to handle more AI loads than the previous competition’s robot. Thus the more powerful Jetson Orin shall allow the robot the ability to process all of its AI loads quickly, allowing the robot to complete all of its required task within the 3 minute time limit. Table 1.A shows a list of the AI process that the global computer shall be able to handle. 

#### Table 1
| Single Board Computer | AI Processing Power (TOPS) | AI Processing Power (TFLOPS)|
|-|-|-|
| Jetson Orin | 67 | apox 16.75 |
| Jetson TX2 | apox 5.32 | 1.33 |
| Jetson Nano | apox 1.88 | 0.472 |


The robot shall also need a software architecture that will allow the robot to handle all its AI loads and sensor integration. The team has the option to continue developing the architecture designed by the previous team. There is a foundation that the team can use to build upon and add on to, including computer vision algorithms [8] [9]. The team can also build a new software architecture using the Robot Operating System (ROS). This architecture is open source and has a set of prebuilt tools for common robot applications. NVIDIA has also built a ROS distribution called Issac which is designed for using AI tools such as object detection and navigation algorithms [10]. This ROS library also makes using a Jetson board more appealing.

The team shall use a NVIDIA Jestion single board computer to handle the sensor intake, AI processing, and decision making for the robot. The team shall utilize ROS as a tool to help implement AI work loads and to control the robot. As the team approaches the detailed design phase, they shall decide which Jeston single board computer will be used. This decision will depend on the computers ability to process the robots AI loads fast enough to meet specification 3. ROS has the capability to run both C++ and Python programs, so the team will consider salvaging programs from the previous team’s robot.






### General Sensors
In order to complete specifications 1, 2, 4, 5, 6,and 7 the robot shall need an array of different sensors that will allow the robot to perceive its environment. The robot shall know its position, its orientation, and have an effective method of tracking its location on the game board. The robot shall also have a means of starting autonomously as defined in specification 6, but also have an auxiliary start switch in case the robot fails to start autonomously.

#### Starting the Robot
In order for the robot to start autonomously as defined in specification 6, the robot shall use a set of photoresistors. Two photoresistors shall be placed toward the front of the robot and will be used to measure the ambient light of the environment. An additional photoresistor will be placed near the rear of the robot. The start LEDs will be placed along the walls of the starting area, thus the rear photoresistor will be facing one of the two start LEDs. These photoresistors will be connected from 5 V power to analog GPIO pins on the Arduino Mega. A program will then compare the ambient light of the environment to the light detected by the rear photoresistor. If the rear photoresistor detects more light than the two ambient light photoresistors, that means that the start LEDs have been turned on and the robot may begin operation [10]. The GL12528 photoresistors can handle up to 250 V and 200 mW of power which is sufficient for this implementation [11].

#### Figure 11 - Gameboard Layout
<img width="1563" height="831" alt="gameboard" src="https://github.com/user-attachments/assets/41f29d79-6af9-4110-9df1-bf07185edc72" />

#### Figure 12 - Screenshot of Automatic Start Code
<img width="1556" height="893" alt="code" src="https://github.com/user-attachments/assets/11362feb-4e52-4cfd-8547-82c2fc4be446" />

Specification 6 requires that the robot begin operation without human intervention, however, the inclusion of a manual start switch shall allow the robot to begin operation in the event it fails to start autonomously. Having a manual start switch also satisfies specification 2. The manual start switch will be a push button located near the top of the robot’s chassis. It will be connected from 5 V power to a digital GPIO pin on the Arduino Mega. This signal shall be used as an interrupt that will start the robot’s operation regardless of whether the robot detects the start LEDs. The push button can handle up to 14 VDC and 10 A which is sufficient for this implementation [12]. 

#### Navigation Sensors
The robot shall use multiple sensors in order to determine its exact location on the gameboard. These sensors shall also be used to pinpoint the robot on a virtual SLAM map and be used to improve that map. The robot's SLAM map and how it is used is discussed in both the object detection and navigation design documents [object detection] [navigation]. To summarize, the robot's SLAM map will allow it to navigate its envirment effiecntly and without collisions. The navigation subsystem will be passed the data that is collected form the navigation sensors so that the NAV2 stack can process it [navigation]. Three LiDAR and one IMU sensor will be used for this task. The robot and UAV mounted cameras will help with navigation as described in the object detection document [object detection].


The LiDAR sensors shall be able to measure the robot's distance from the gameboard walls or obsticals during the robot's operation, aiding the navigation system with obstical avoidence and position mapping. Garmin LIDAR-Lite V4 sensors will be used. These sensors have an effective range of 5 cm (1.97 in) to 10 m (32.8 ft) with a 1 cm resolution. They also have an error of +-1 cm at 2 m [lidar data sheet]. 

LiDAR, or Light Detection and Ranging, sensors use laser pulses to measure distance. They can be used for detecting obstacles, and be used to create virtual maps of the robots surroundings. Their ability to gather high volumes of accurate data in a short time frame makes these sensors a good option. They are also well known for their ability to create three dimensional maps, which shall be integral for our navigational strategy as defined in its own subsystem [14] [15].

IMUs, or Inertial Measurement Units, are devices that contain an array of sensors like accelerometers, gyroscopes, and potentially magnetometers [16]. These devices are able to track changes in velocity and rotational orientation. These measurements shall allow the robot to know its exact orientation at a given time. The velocity data can also be integrated over short periods in order to find the robots position data [17].

Using these two sensor types together shall allow the robot to know its exact position and orientation on the game board. The robot shall use one IMU for position and orientation. As the team continues into the detailed design phase, they shall consider how to implement the LiDAR sensor(s). LiDAR sensors come in various configurations and the team shall consider which configuration is the most cost effective while being able to provide all of the needed information for the robot’s successful operation.


## References
All sources that have contributed to the detailed design and are not considered common knowledge should be duly cited, incorporating multiple references.

[1] “2026 IEEE+SoutheastCon+Hardware+Competition+Ruleset_8_15_2025.” IEEE, Aug. 15, 2025 

[2] NVIDIA, “NVIDIA Jetson AGX Orin,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

[3] J. Vassar et al., “F25_Team7_SECONHardwareCompetition2025/reports/team 7 conceptual Design.md at main · tntech-ECE/F25_TEAM7_SECONHARDWARECOMPETITION2025,” GitHub, https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/main/Reports/Team%207%20Conceptual%20Design.md (accessed Nov. 10, 2025). 

[4] NVIDIA, “NVIDIA Jetson TX2: High Performance AI at the Edge,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-tx2/

[5] NVIDIA, “NVIDIA jetson nano,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/

[6] “What is Ai Tops? how it differs from teraflops.,” C&T Solution Inc. | 智愛科技股份有限公司, https://www.candtsolution.com/news_events-detail/tops-and-teraflops-in-AI/ (accessed Nov. 12, 2025).

[7] GeeksforGeeks, “Floating-point operations per second (flops),” GeeksforGeeks, https://www.geeksforgeeks.org/computer-organization-architecture/what-is-floating-point-operations-per-second-flops/ (accessed Nov. 12, 2025).

[8] P. Burns, “A guide to ai tops and NPU Performance Metrics,” Wireless Technology & Innovation, https://www.qualcomm.com/news/onq/2024/04/a-guide-to-ai-tops-and-npu-performance-metrics (accessed Nov. 12, 2025).

[9] “What is the relationship between the units of Tops and flops? - genspark,” What Is The Relationship Between The Units Of Tops And Flops?, https://www.genspark.ai/spark/what-is-the-relationship-between-the-units-of-tops-and-flops/466c318f-635c-30a6-90f2-594f3fe1b1d1 (accessed Nov. 12, 2025).

[10] D. Moye, A. Cruz, S. Hunter, and S. Borchers, “F24_Team1_SECON/reports/experimental analysis.md at Main · tntech-ECE/F24_TEAM1_SECON,” GitHub, https://github.com/TnTech-ECE/F24_Team1_SECON/blob/main/Reports/Experimental%20Analysis.md (accessed Nov. 5, 2025).

[11]“Juried Engineering Photoresistor GL12528 12528 Photo Light Sensitive Resistor Light Dependent Resistor 12 mm GM12528 (Pack of 10): Amazon.com: Industrial & Scientific,” Amazon.com, 2025. https://www.amazon.com/Juried-Engineering-Photoresistor-GL12528-Sensitive/dp/B08F3WPNPF (accessed Nov. 20, 2025).

[12] 40-4325-00 JUDCO Manufacturing Inc. | switches | DigiKey, https://www.digikey.com/en/products/detail/judco-manufacturing-inc/40-4325-00/254287 (accessed Nov. 20, 2025). 
