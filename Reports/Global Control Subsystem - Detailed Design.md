# Detailed Design

## Function of the Subsystem

The global control system is designed to function as the robot's data collection and control center. This subsystem consists of a single board computer that will be used to compute and process the robot's high level control processes, such as its navigation, object detection, and decision making behaviors. The single board computer will use these processes to send instructions to the low level controller that is responsible for controlling the robot's motors. It will also send commands to the robot's companion UAV. The Global control system also includes the sensors that will be used to help collect position data that will be fed to the navigation subsystem.

## Specifications and Constraints

The IEEE SECON rules handbook contains the specifications and contraints for the robot's operation and function. These specifications and constraints are listed bellow [1].
 
1.  Both the robot and UAV shall be completely autonomous.
2.  The robot and UAV shall have a clearly labeled start switch. 
3.  Teams shall have a maximum of 3 minutes to earn points.
  * The robot shall have the ability to quickly process sensor data.
  * The robot shall have the ability to quickly process AI workloads.
  * The robot shall have the ability to quickly make decisions and communicate them.
4.  Teams shall rescue a total of 6 Astro-Ducks and shall return them to the “Lunar Landing Area.”
  * The robot shall be able to identify, locate, and navigate to the Astro-Ducks.
  * The robot shall be able to identify and navigate to the Lunar Landing Area.
5. Teams shall establish power to the 4 antennas throughout the course. Power shall be restored differently for each antenna. Once power is restored to the antenna, a randomly colored LED shall light up (red, blue, green, and purple).
  * The robot shall be able to identify, locate, and navigate to the antenna towers.
  * The robot must identify and complete the task located on each tower.
6.  Two starting white LED bars shall be placed on top of the arena wall, one on each side of the 12” x 12” starting area. 
  * The robot shall be able to sense the starting white LEDs in order to begin operation.
7.  Points shall be lost every time the robot or UAV has an unintentional collision 
  * The robot and UAV shall be able to detect and avoid obstacles.

In addition to the listed specifications and constrains, socio-economic and ethecal contraints will be implemented.

#### Socio-Economic

In order to ensure that the project remains on budget and does not produce unnecessary waste, priority will be placed on using parts that can be salvaged from previous SECON robots. Reusing previous parts will allow the team to cut down on parts cost.

#### Ethical

As well as using salvaged parts, the team shall utilize salvaged programs and utilizes predeveloped tools to aid in the programing of robot and UAV. It is the team's priority to ensure proper licensing of used software and to disclose when such resources are used.


## Overview of Proposed Solution

The global control system is divided into multiple major components: The Global controller itself and the sensors that it relies on in order to collect the data the robot needs to make decisions.

The global controller will act as the central computer that will be used for collecting data, processing data, and making decisions based on that data. This computer will be the device that makes all of the control decisions for the robot. It will also host the processes for object detection and navigation. Based on the data that is received and computed, it will make decisions and send the necessary commands to the low-level controller that will coordinate the robot’s motion and actuators. The robot shall use an NVIDIA Jetson Orin for this task. This single board computer was designed to efficiently run AI loads which will be essential for the robot’s object detection and navigation processes.

The robot shall also use a combination of sensors that will allow the robot to perceive its location on the gameboard, and be able to identify the objects on it. LiDAR sensors and an IMU will be used to pinpoint the robot’s location on the game board. The robot shall also utilize photoresistors which will be used to start the robot autonomously. The robot will also have a manual start button. The robot and the UAV will have cameras that will send image data to the global computer over USB and WiFi respectively. More detail about the cameras can be found in the object detection detailed design document [2].

## Interface with Other Subsystems

The Jetson Orin has access to video encodes, video decoders, CSI camera support, PCIe, USB, 1 gigabit ethernet, display ports, UART, SPI, I2S, I2C, GPIO, PWM, DMIC, and DSPK protocols. The development board comes with a 40-pin header for IO, 2 CSI camera connectors, 4 USB type A connectors, a display port, an ethernet port, and a barrel jack for power [3]. The robot mounted camera will connect to the global computer using a USB connection. The UAV comes with a USB adaptor that will allow the global control computer to communicate with the UAV flight controller. Details for the USB adaptor can be found in the communication detailed design document[4]. The UAV mounted camera will connect to the global computer over a WiFi connection. The Jetson Orin does not come with built in WiFi, so an adaptor will need to be purchased. The Low Level Controller will connect to the global computer over USB. Both the IMU and the LiDAR sensors will connect to the global computer using I2C. Both the autonomous and manual start signals will be GPIO signals.

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

<img width="677" height="917" alt="soft 1" src="https://github.com/user-attachments/assets/310cf1e4-0754-43c7-b672-4b84cfc72ddd" />

#### Figure 8 - Software Flowchart 2

<img width="417" height="1043" alt="soft 2" src="https://github.com/user-attachments/assets/ff4a76a4-f36e-4c28-b0d3-370891e4bf1a" />


#### Figure 9 - Software Flowchart 3

<img width="608" height="1118" alt="soft 3" src="https://github.com/user-attachments/assets/445299ef-aabc-4988-a97b-d758c968ee33" />

#### Figure 10 - Sofrware Flowchart 4

<img width="665" height="722" alt="soft 4" src="https://github.com/user-attachments/assets/07d74887-c5e6-4381-9261-d96a6c396d7c" />

## BOM
| Part        | Manufacturer| Part Number | Distributor | Distributo Part Number | Quantity | Price | Total Price | Website URL |
| ----------- | ----------- | ----------- | ----------- | ---------------------- | -------- | ----- | ----------- | ----------- |
| Jetson Orin | NVIDIA | N/A | Amazon | 945-137766-0000-000 | 1 | $249.00 | $249.00 | [Store Link](https://www.amazon.com/dp/B0BZJTQ5YP?utm_source=nvidia&th=1) | 
| Garmin LIDAR-Lite V4 | Garmin | N/A | SparkFun | SEN-18009 | 3 | $119.95 | $359.85 | [Store Link](https://www.sparkfun.com/garmin-lidar-lite-v4-led-distance-measurement-sensor-qwiic.html) |
| 9Dof IMU Breakout | SparkFun | SEN-15335 | SparkFun | SEN-15335 | 1 | $21.95 | $21.95 | [Store Link](https://www.sparkfun.com/sparkfun-9dof-imu-breakout-icm-20948-qwiic.html) |
| GL12528 Photoresistor (10 pack) | Juried Engineering | N/A | Amazon | B08F3WPNPF | 1 | $22.98 | $22.98 | [Store Link](https://www.amazon.com/Juried-Engineering-Photoresistor-GL12528-Sensitive/dp/B08F3WPNPF) |
| Auxiliary Start Switch | Judco Manufacturing Inc. | 40-4325-00 | Digikey | 512PB-ND | 1 | $2.55 | $2.55 | [Store Link](https://www.digikey.com/en/products/detail/judco-manufacturing-inc/40-4325-00/254287) |
| Flexible Qwiic Cable - 200mm | SparkFun | PRT-17258 | SparkFun | PRT-17258 | 4 | $1.95 | $7.80 | [Store Link](https://www.sparkfun.com/flexible-qwiic-cable-200mm.html) |
| Flexible Qwiic Cable - Female Jumper | SparkFun | CAB-17261 | SparkFun | CAB-17261 | 2 | $1.95 | $3.90 | [Store Link](https://www.sparkfun.com/flexible-qwiic-cable-female-jumper-4-pin.html) |
| QWiic MultiPort | SparkFun | BOB-18012 | SparkFun | BOB-18012 | 1 | $2.50 | $2.50 | [Store Link](https://www.sparkfun.com/sparkfun-qwiic-multiport.html) |
| Jumper Wire Kit | SparkFun | PRT-00124 | SparkFun | PRT-00124 | 1 | $8.95 | $8.95 | [Store Link](https://www.sparkfun.com/jumper-wire-kit-140pcs.html) |
| Right Angle Header - Male | SparkFun | PRT-30218 | SparkFun | PRT-30218 | 4 | $0.75 | $3.00 | [Store Link](https://www.sparkfun.com/right-angle-header-male-pth-0-1in-6-pin.html) |
| USB WiFi Adapter | BrosTrend | N/A | Amazon | B0F6N1H84N | 1 | $23.99 | $23.99 | [Store Link](https://www.amazon.com/BrosTrend-AX900-Linux-WiFi-Adapter/dp/B0F6N1H84N/ref=sr_1_14?dib=eyJ2IjoiMSJ9.78-l3ki-Cdgk66rHmmKHe5qLTTbS9eqaGMqy-N1vxuT9ts6YpoDAxhiYgS5h9aVn9VTi9ZNJwANd-MHeNQAX7OxGamrVIyeR2zN0ZvEaOG2p-lf7OJzYmn8SxOmwP3koahc9B50WKRfAQRArUsQ7JJ-P3BLRCqjkE8z_S6lf0qjZg05xnagBSqaYgHQ1IOcmqJOAubb61hBmwY1hpaqUG-YNwDHtmeGyrqFnx_kKe4M.lKWOEbYNy6bvYiR4wKm4y_ric09EUjYlcziARQIoZ6I&dib_tag=se&keywords=linux+compatible+wifi+adapter&qid=1771211949&sr=8-14) |

## Analysis
### Global Controller
The global controller is the single board computer that will be responsible for the high level control of the robot. It will host the processes for navigation, object detection, and the robot's behavior scheme. The Global controller shall:

* Connect to the UAV using the CrazyFly PA USB adaptor to administor commands
* Connect to the UAV cammera using WiFi to recieve image data of the gameboard
* Connect to and send motor control commands to the Low Level Controller over USB
* Process the object detection algorithm accossiated with the UAV camera
* Process the object detection algorithm accossiated with the robot mounted camera
* Process the object detcion post-processing and object estimation
* Process the ROS2 NAV2 stack that controls the robots pathfinding and navigation
* Intake and feed the LiDAR data into the NAV2 stack
* Intake and feed the IMU data into the NAV2 stack

These functions are explained in greater detail in their respective design documents [2] [4] [5]. The global controller shall also run the program that will dictate the Robot's behavior during its operation. The robot's operation is illustrated in the figures 7-10. Once the start signal is given, whether that be the autonomous or manual start signal, a three minute timer will begin. This timer represents the three minutes the robot has to complete its operation. Once the timer begins, the UAV will launch and begin its operation as described in the object detection design document [2]. Using the data collected by the UAV and robot mounted camera, objects on the gameboard will be classified. Once either an astro duck or antenna tower is identified, a task is created and entered into an event queue to allow the robot to complete each task one at a time. Once the internal timer has thirty seconds remaining, the robot will complete its active task before returning to its starting zone. The UAV will then transmit the LED color data to Earth before landing.

In order to meet specifications 1, 5, and 6; a small single board computer is necessary for this implementation, because of the limited size constraints of the robot. The robot shall use a Jetson Orin, more specifically a Jetson Orin Nano Super Developer Kit. The robot shall use a Jeston Orin instead of the Jetson Nano or the Jetson TX2 because of its processing power. The Jetson Orin contains a 6-core ARM Cortex 64-bit CPU that operates at 1.7 GHz, a 1024-core NVIDIA Ampere architecture GPU that operates at 1020MHz, and 8GB 128-bit LPDDR5 RAM [3]. This makes the Jetson Orin more powerful than the Nano. The Jetson TX2's CPU and GPU both operate at a higher frequency than the Orin [6], however, the Orin's CPU and GPU contain significantly more cores than the TX2. The Jetson Orin's component processing power will allow it to execute its program function quickly and efficiently. The Jetson Orin shall also use Ubuntu as its operating system. This will allow for the ROS2 NAV2 stack to be implemented efficiently. The WiFi adapter that will be used to connect the UAV mounted camera to the global controller is Ubuntu compatible [6].

The Jetson series of single board computers are built with processing AI loads in mind. The robot shall be using two object detection algorithms alongside an object detection post processing algorithm, so having a computer that is designed for effectively handling that type of work load is desirable. The Jetson Orin is listed as having an AI performance of 67 tera-operation per second (TOPS) [2]. The Nano and the TX2 have an AI performance of 0.472 and 1.33 tera floating point operation per second (TFLOPS) respectively [7] [8]. TOPS and TFLOPS are both metrics that are used to describe AI performance [9] [10], though they are different, with TOPS referring to 8 bit integer operations while TFLOPS refers to floating point operation [9]. However, a general case conversion can be made to compare the two [11]. An exact conversion is hardware architecture dependent, however, using the general case the Orin is shown to be the most powerful of the three listed computers. The previous competition’s robot was able to utilize the Jetson Nano, however, the 2026 competition’s robot will need to handle more AI loads than the previous competition’s robot. Thus the more powerful Jetson Orin shall allow the robot the ability to process all of its AI loads quickly, allowing the robot to complete all of its required tasks within the 3 minute time limit. The Jetson Nano will serve as a potential replacement global controller in case it is needed.

#### Table 1
| Single Board Computer | AI Processing Power (TOPS) | AI Processing Power (TFLOPS)|
|-|-|-|
| Jetson Orin | 67 | apox 16.75 |
| Jetson TX2 | apox 5.32 | 1.33 |
| Jetson Nano | apox 1.88 | 0.472 |

### General Sensors

In order to complete specifications 1, 2, 4, 5, 6, and 7 the robot shall need an array of different sensors that will allow the robot to perceive its environment. The robot shall know its position, its orientation, and have an effective method of tracking its location on the game board. The robot shall also have a means of starting autonomously as defined in specification 6, but also have an auxiliary start switch in case the robot fails to start autonomously.

#### Starting the Robot

In order for the robot to start autonomously as defined in specification 6, the robot shall use a set of photoresistors. Two photoresistors shall be placed toward the front of the robot and will be used to measure the ambient light of the environment. An additional photoresistor will be placed near the rear of the robot. The start LEDs will be placed along the walls of the starting area, thus the rear photoresistor will be facing one of the two start LEDs. These photoresistors will be connected from 3.3 V power to analog GPIO pins on the Jetson. A program will then compare the ambient light of the environment to the light detected by the rear photoresistor. If the rear photoresistor detects more light than the two ambient light photoresistors, that means that the start LEDs have been turned on and the robot may begin operation [10]. The GL12528 photoresistors can handle up to 250 V and 200 mW of power which is sufficient for this implementation [14].

Specification 6 requires that the robot begin operation without human intervention, however, the inclusion of a manual start switch shall allow the robot to begin operation in the event it fails to start autonomously. Having a manual start switch also satisfies specification 2. The manual start switch will be a push button located near the top of the robot’s chassis. It will be connected from 3.3 V power to a digital GPIO pin on the Jetson. This signal shall be used as an interrupt that will start the robot’s operation regardless of whether the robot detects the start LEDs. The push button can handle up to 14 VDC and 10 A which is sufficient for this implementation [15].

#### Figure 11 - Screenshot of Automatic Start Code
<img width="1556" height="893" alt="code" src="https://github.com/user-attachments/assets/11362feb-4e52-4cfd-8547-82c2fc4be446" />

#### Navigation Sensors

The robot shall use multiple sensors in order to determine its exact location on the gameboard. These sensors shall also be used to pinpoint the robot on a virtual SLAM map and be used to improve that map. The robot's SLAM map and how it is used is discussed in both the object detection and navigation design documents [2] [5]. To summarize, the robot's SLAM map will allow it to navigate its environment efficiently and without collisions. The navigation subsystem will pass the data that is collected form the navigation sensors so that the NAV2 stack can process it [5]. Three LiDAR and one IMU sensor will be used for this task. The robot and UAV mounted cameras will help with navigation as described in the object detection document [2]. These sensors shall be connected to a single I2C bus using SparkFun QWIIC cables and an intermediate bus circuit board.

The LiDAR sensors shall be able to measure the robot's distance from the gameboard walls or obstacles during the robot's operation, aiding the navigation system with obstacle avoidance and position mapping. Garmin LIDAR-Lite V4 sensors will be used. These sensors have an effective range of 5 cm (1.97 in) to 10 m (32.8 ft) with a 1 cm resolution. They also have an error of +-1 cm at 2 m [16]. The gameboard is 4 ft by 8 ft (2.4 m), and with the robot being approximately 1 ft, these sensors will be operating in their optimal range during the robot's operation [1]. The LiDAR sensor shall transmit their data over an I2C bus. The I2C addresses of these sensors can be set through software, ensuring address conflicts do not occur [16] [17]. They also have an update rate from 200 Hz to 400 kHz (2.5 microseconds) over I2C allowing them to transmit data in a sufficiently quick manner. Two sensors will be placed on the side of the robot with one being placed on the rear. A pre-excisting software library exists for these sensors and shall be used/modified for the robot.

The IMU sensor shall be used to measure the angular velocity and acceleration of the robot in order to help track its orientation and its position. The SparkFun 9Dof breakout IMU will be used for this application. It contains a 3-axis gyroscope, accelometer, and Magnetometer. The Magnetometer shall not be used, as it is not needed for this application. The 3-Axis gyroscope has a programmable range (in degrees per second) ranging from ±250 dps, ±500 dps, ±1000 dps, and ±2000 dps. The 3-Axis accelerometer has a programmable range of ±2g, ±4g, ±8g, and ±16g. The chip these sensors are located in can transmit data over I2C at 400 kHz, allowing the data to be transmitted in a sufficiently quick manner [18]. This sensor can have two unique I2C addresses. Since the LiDAR sensors can be assigned an I2C address programmatically, and only one IMU is used, no additional steps are needed to prevent a duplicate address issue [17] [18]. The IMU will be located at the center of the robot's chassis.

#### Figure 12 - Gameboard Layout
<img width="1563" height="831" alt="gameboard" src="https://github.com/user-attachments/assets/41f29d79-6af9-4110-9df1-bf07185edc72" />

## References
All sources that have contributed to the detailed design and are not considered common knowledge should be duly cited, incorporating multiple references.

[1] “2026 IEEE+SoutheastCon+Hardware+Competition+Ruleset_8_15_2025.” IEEE, Aug. 15, 2025 

[2] TnTech-ECE, “F25_Team7_SECONHardwareCompetition2025/Reports/Object Detection -- Detailed Design.md at Object-Detection----Detailed-Design · TnTech-ECE/F25_Team7_SECONHardwareCompetition2025,” GitHub, 2025. https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Object%20Detection%20--%20Detailed%20Design.md
‌
[3] NVIDIA, “NVIDIA Jetson AGX Orin,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/

[4] A. Mullins, “Detailed_Design_Communication,” GitHub, Dec. 03, 2025. https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Detailed_Design_Communications/Reports/Detailed_Design_Communication.md

[5] TnTech-ECE, “F25_Team7_SECONHardwareCompetition2025/Reports/Navigation Detailed Design.md at Navigation----Detailed-Design · TnTech-ECE/F25_Team7_SECONHardwareCompetition2025,” GitHub, 2025. https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Navigation----Detailed-Design/Reports/Navigation%20Detailed%20Design.md

[6]“BrosTrend AX900 Mini USB WiFi 6 Linux Compatible WiFi Adapter,” Amazon. https://www.amazon.com/BrosTrend-AX900-Linux-WiFi-Adapter/dp/B0F6N1H84N/ref=sr_1_14?dib=eyJ2IjoiMSJ9.78-l3ki-Cdgk66rHmmKHe5qLTTbS9eqaGMqy-N1vxuT9ts6YpoDAxhiYgS5h9aVn9VTi9ZNJwANd-MHeNQAX7OxGamrVIyeR2zN0ZvEaOG2p-lf7OJzYmn8SxOmwP3koahc9B50WKRfAQRArUsQ7JJ-P3BLRCqjkE8z_S6lf0qjZg05xnagBSqaYgHQ1IOcmqJOAubb61hBmwY1hpaqUG-YNwDHtmeGyrqFnx_kKe4M.lKWOEbYNy6bvYiR4wKm4y_ric09EUjYlcziARQIoZ6I&dib_tag=se&keywords=linux+compatible+wifi+adapter&qid=1771211949&sr=8-14
‌‌
[7] NVIDIA, “NVIDIA Jetson TX2: High Performance AI at the Edge,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-tx2/

[8] NVIDIA, “NVIDIA jetson nano,” NVIDIA. https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-nano/product-development/

[9] “What is Ai Tops? how it differs from teraflops.,” C&T Solution Inc. | 智愛科技股份有限公司, https://www.candtsolution.com/news_events-detail/tops-and-teraflops-in-AI/ (accessed Nov. 12, 2025).

[10] GeeksforGeeks, “Floating-point operations per second (flops),” GeeksforGeeks, https://www.geeksforgeeks.org/computer-organization-architecture/what-is-floating-point-operations-per-second-flops/ (accessed Nov. 12, 2025).

[11] P. Burns, “A guide to ai tops and NPU Performance Metrics,” Wireless Technology & Innovation, https://www.qualcomm.com/news/onq/2024/04/a-guide-to-ai-tops-and-npu-performance-metrics (accessed Nov. 12, 2025).

[12] “What is the relationship between the units of Tops and flops? - genspark,” What Is The Relationship Between The Units Of Tops And Flops?, https://www.genspark.ai/spark/what-is-the-relationship-between-the-units-of-tops-and-flops/466c318f-635c-30a6-90f2-594f3fe1b1d1 (accessed Nov. 12, 2025).

[13] D. Moye, A. Cruz, S. Hunter, and S. Borchers, “F24_Team1_SECON/reports/experimental analysis.md at Main · tntech-ECE/F24_TEAM1_SECON,” GitHub, https://github.com/TnTech-ECE/F24_Team1_SECON/blob/main/Reports/Experimental%20Analysis.md (accessed Nov. 5, 2025).

[14]“Juried Engineering Photoresistor GL12528 12528 Photo Light Sensitive Resistor Light Dependent Resistor 12 mm GM12528 (Pack of 10): Amazon.com: Industrial & Scientific,” Amazon.com, 2025. https://www.amazon.com/Juried-Engineering-Photoresistor-GL12528-Sensitive/dp/B08F3WPNPF (accessed Nov. 20, 2025).

[15] 40-4325-00 JUDCO Manufacturing Inc. | switches | DigiKey, https://www.digikey.com/en/products/detail/judco-manufacturing-inc/40-4325-00/254287 (accessed Nov. 20, 2025). 

[16] “LIDAR-LITE V4 LED OPERATION MANUAL AND TECHNICAL SPECIFICATIONS Specifications Specification Measurement Unit dimensions (L × W × H) Labeling Requirements Connections LIDAR-Lite v4 LED Connection Diagram,” 2019. Available: https://static.garmin.com/pumac/LIDAR-Lite%20LED%20v4%20Instructions_EN-US.pdf

[17] C. Nelson, “Working with I2C Devices,” Adafruit Learning System, Mar. 09, 2022. https://learn.adafruit.com/working-with-i2c-devices/address-conflicts

[18] “ICM-20948.” Available: https://cdn.sparkfun.com/assets/7/f/e/c/d/DS-000189-ICM-20948-v1.3.pdf
‌
‌
‌
