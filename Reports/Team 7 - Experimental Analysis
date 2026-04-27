# Introduction 

What is this doc for?  

Talk about messures of Sucess  

Talk about the point system. 

# Global Controller 

## Relevant Success Criteria 

1. Both the robot and UAV shall be completely autonomous [Vehicle Spec 1] 
2. Teams shall have a maximum of 3 minutes to earn points. [Objective Spec 3] 
     a. The robot shall have the ability to quickly make decisions and communicate them. 
3. Teams shall rescue a total of 6 Astro-Ducks and shall return them to the “Lunar Landing Area.” [Objective Specs 4/5] 
     a. The robot shall be able to identify, locate, and navigate to the Astro-Ducks. 
     b. The robot shall be able to identify and navigate to the Lunar Landing Area. 
4. Teams shall establish power to the 4 antennas throughout the course. Power shall be restored differently for each antenna. Once power is restored to the antenna, a randomly colored LED shall light up (red, blue, green, and purple). [Objective Spec 6] 
     a. The robot shall be able to identify, locate, and navigate to the antenna towers. 
     b. The robot must identify and complete the task located on each tower. 
     c. The UAV shall identify the color of the LED on each antenna tower. 
     d. The UAV shall know which area each antenna tower and their LEDs are located in. 
5. Two starting white LED bars shall be placed on top of the arena wall, one on each side of the 12” x 12” starting area. [Board Spec 5] 
     a. The robot shall be able to sense the starting white LEDs in order to begin operation. 
6. Points shall be lost every time the robot or UAV has an unintentional collision [Objective Constrain 2] 
     a. The robot and UAV shall be able to detect and avoid obstacles 

Above are the listed specifications and constraints that the global control subsystem encompasses. Specifications 1 and 5 are achieved by the use of a photoresistor that will detect the light of the starting LED. This will be tested by using the starting LED on the practice board to determine if the photoresistor will allow the robot to start autonomously. Specifications 2, 4, and 6 are accomplished by the robot's master program that is hosted on the Arduino Mega. These specifications will be tested by allowing the robot to conduct a competition run and monitoring the tasks that the robot is able to complete and how long it takes for the robot to complete operation.  

Specification 3 cannot be properly tested, as the astro duck collecting functions of the robot were not completely developed. The systems needed in order to collect the astro ducks were developed but not fully integrated by the competition deadline. The UAV that was ordered for the project did not arrive before the competition deadline, thus any success critria involving it could not be tested. 

## Photoresistor – Autonomous Start Device 

The photoresistor is used in order to start the robot's autonomous operation. It will detect the light emitted from the starting LED and use that signal to begin the robot's operation. This will meet specifications 1 and 5.  

The robot was originally intended to use three photoresistors, two mounted on the front part of the chassis, and one mounted on the rear of the chassis. The two front-mounted photoresistors would be used to detect the ambient light of the environment around the robot while the rear-mounted photoresistor would be used to detect the starting LED. However, during the assembly and testing of the robot, the two front-mounted photoresistors were not functional and removed.  

### Purpose and Justification: 

The photoresistor is used to autonomously start the robot, which is a critical function in order for the robot to operate successfully. The robot's ability to start autonomously will secure the team a total of 15 points during the competition and allow the robot to complete other tasks to score more points. This test is to ensure that the photoresistor will be able to reliably start the robot, using the starting LED.  

### Detailed Procedure: 

The robot uses a single photoresistor that is located at its rear. It is connected to both 3.3 V power and the robot's Arduino Mega. As the photoresistor is exposed to more light, the analog value that is recorded by the Arduino decreases as the photoresistor's resistance increases. The Arduino will detect the starting LED if the analog value of the photoresistor is under a predefined threshold, at which the robot will being to move forward.  

For each test, the Starting LED will be activated and the robot's response will be recorded. A successful test is defined by the robot moving forward after being exposed to the starting LED. A failed test is defined by the robot not moving forward after being exposed to the starting LED.  

Figure 1 - The Robot’s Rear Mounted Photoresistor  

(Insert image here)  

Expected Results: 

The photoresistor will be able to detect the starting LED repeatedly and reliably.  

Actual Results: 

Below is a table containing the testing results. These tests were conducted at the IEEE SECON competition during the qualifying and semi-final rounds. 

Table 1 – Photoresistor Testing Results

| Competition Run | Did The Robot Start Autonomously? |
|-----------------|-----------------------------------|
| 1               | Yes                               |
| 2               | Yes                               |   
| 3               | Yes                               |
| 4               | Yes                               |
| 5               | Yes                               |

### Interpretation and Conclusions: 

 

The photoresistor was able to successfully detect the starting LED during all of the team's competition runs. While the team was testing the photoresistor's functionality, the robot was found to start prematurely if the robot was not handled carefully. It is theorized that the threshold for detecting the starting LED was likely too low, resulting in the photoresistor being too sensitive to the point where the robot could be triggered by a team member walking by the robot. The high sensitivity was required in order for the photoresistor to detect the starting LED. Using a single photoresistor worked during the competition, however, using the ambient light solution that was previously discussed would have likely produced more consistent results with less premature starts.  

Arduino Master Program 

The robots master program determines what actions the robot makes during the course of its operation. This program is hosted on the robots Arduino Mega and controls how the robot’s motors move in order to complete specific tasks. This will meet specifications 2, 4, and 6.  

The master program was originally intended to be hosted on the Jetson Orin. The Jetson Orin would act as the central computer that would make the decisions during the robot’s operation. This version of the master program would track different events that the robot encountered during its operation and based on the type of event encountered, entered the robot into the appropriate state required to resolve it. These states would be predefined but would allow the robot to move as needed in order to resolve events as they occurred. Both the Jeston Orin and its master program were not completed in time for the competition deadline. Instead, the master program that the robot uses was simplified so it could be programed onto the Arduino. This version of the program would execute predefined paths and actions once the robot received the autonomous start signal. This change forced the robot to become less flexible in its operation but allowed it to complete simple yet high-point earning tasks reliably.  

Purpose and Justification: 

The robot’s master program controls how the robot operates during a competition run, thus ensuring that the program functions as intended is critically important to the robot’s success. The tasks that the robot will need to complete during a competition run are preprogramed, and if these tasks are not programed properly, it could result in unintended collisions and damage to both the robot and the gameboard.  

Detailed Procedure: 

The robot’s master program can be broken down into the individual task that the robot needed to complete during a competition run. These parts of the program were tested individually to ensure that each part of the program operated as expected. Using the "DoActionRange()" function, the values for speed and position were set for each of the robot's motors. Below is a list of programed tasks and how each task was to be executed. Figure 2 shows a screenshot of the program required to complete task 1. The programs for the remaining tasks follow a similar structure of writing speed and position values to the motors, letting the motors run for a given amount of time, and taking a half second pause before writing a new set of speed and position values. 

1. Task 1 - Activate The Antenna with The Pushbutton Task
     - The Robot will move forward and hit the Antenna pushbutton
     - It will then move back and forward three times to press the button a total of four times
2. Task 2 - Allign Robot with The Crater
     - The robot will move backwards from the antenna tower
     - It will then rotate 90 degrees in the direction of the crater
     - The robot will move backwards, using the gameboard wall to straighten itself
3. Task 3 - Enter/Exit Crater
     - The robot will move forward into the crater to enter it
     - The robot will then move backward to exit the crater
4. Task 4 - Return to The Starting Area
     - The robot will rotate 90 degrees, orienting its rear in the direction of the starting zone
     - The robot will move backward into the starting zone
5. Task 5 - Deposit The Beacon
     - The robot will extend its robotic arm
     - The claw of the roboitc arm will open, reliesing the beacon
6. Task 6 - Loop Around The Crater.
     - The robot will move toward the crater after it aligns with the crater
     - The robot will turn slightly to the right before entering the crater
     - The robot will move forward and left simultaneously to circle the crater

All of the six task will be tested in a similar way. Testing will involve observing the robot during its operation and recording if it completes each of the listed task exactly as they are described. A Successful test is defined as the robot completing a given task. A failed test is defined as the robot not completing a given task. 

Figure 2 – Screenshot Showing The Program for Task 1 (Insert Image here) 

Firgue 3 - Screenshot of The Gameboard for Reference (Insert Image here) 

Expected Results: 

Assuming that the robot does not encounter hardware issues during testing, the robot will be able to complete the above defined task without unintentional collisions. The robot's path is hard coded based on the layout of the gameboard and was designed with its obstacles in mind.  

Actual Results: 

Below are the tables containing the testing result. These tests were conducted at the IEEE SECON competition during the qualifying and semi-final rounds.  

Table 2 – Task 1 Testing Results 
| Competition Run | Did The Robot Reach The Antenna? | Number Antenna Button Presses |
|-----------------|----------------------------------|-------------------------------|
| 1               | Yes                              | 4                             |
| 2               | Yes                              | 4                             |  
| 3               | Yes                              | 4                             | 
| 4               | Yes                              | 4                             |
| 5               | Yes                              | 4                             |

Table 3 – Task 2 Testing Results 
| Competition Run | Did The Robot Successfully Rotate? | Did The Robot Align with The Crater?  |
|-----------------|------------------------------------|---------------------------------------|
| 1               | Yes                                | Yes                                   |
| 2               | Yes                                | Yes                                   |  
| 3               | Yes                                | Yes                                   |
| 4               | Yes                                | Yes                                   |
| 5               | Yes                                | Yes                                   |

Table 4 – Task 3 Testing Results 
| Competition Run | Did The Robot Enter the Crater? | Did the Robot Exit the Crater? |
|-----------------|---------------------------------|--------------------------------|
| 1               | Yes                             | Yes                            |
| 2               | Yes                             | Yes                            |  
| 3               | Yes                             | Yes                            |
| 4               | Yes                             | Yes                            |
| 5               | Yes                             | No                             |

Table 5 – Task 4 Testing Results 
| Competition Run | Did The Robot Return to the Starting Area? |
|-----------------|--------------------------------------------|
| 1               | Yes                                        |
| 2               | Yes                                        |   
| 3               | Yes                                        |
| 4               | Yes                                        |
| 5               | No                                         |

Table 6 – Task 5 Testing Results 
| Competition Run | Did The Robot Deposit The Beacon? |
|-----------------|-----------------------------------|
| 1               | Yes                               |
| 2               | Yes                               |   
| 3               | Yes                               |
| 4               | Yes                               |
| 5               | Yes                               |

Table 7 – Task 6 Testing Results 
| Competition Run | Did The Robot Circle The Crater? |
|-----------------|----------------------------------|
| 1               | No                               |
| 2               | No                               |   
| 3               | No                               |
| 4               | No                               |
| 5               | Yes                              |

Interpretation and Conclusions: 

The Robot was able to complete Tasks 1, 2, and 5 repeatably as expected. Task 6 was only successful once. This is because task 6 was not attempted until the fifth competition run after the robot had successfully secured 100 points by completing Tasks 1, 2, 3, 4, and 5 during the fourth competition run. Task 6 was successfully completed during competition run five, however, The robot was not able to successful exit the crater after completely circling around it. Because the robot could not exit the crater, it would not be able to return to the starting area. 

Jeston Orin to Arduino Communication 

The Jetson Orin was not developed and integrated in time for the competition deadline, however, after the competition some time was allocated to continue working on the Jetson's programs. The master program would still remain unfunctional, however, the Jetson was able to communicate commands to the Arduino. 

Purpose and Justification: 

The original concept for the robot would require the Jetson Orin to transmit speed and position values to the Arduino in order for the robot to operate successfully. If the Jetson Orin was part of the final robot's implementation, its ability to communicate and send commands to the Arduino would be critical for the robot's successful operation.  

Detailed Procedure: 

The two functioning programs the Jetson had were its communication and motor control programs. The former is responsible for connecting to the Arduino and transmitting and receiving the serial data between the Arduino and the Jetson. The Latter program is responsible for determining the speed and position values for the robot's motors. By manually entering the speed and position values of the motors in the motor control program, the communication program would then transmit that data to the Arduino.  

For each test, The speed and position values of the motors where manually set in the motor control program. Then the program was compiled and ran while being connected to the Arduino. The response of the robot's motors was then recorded. A successful test is defined by the robot's motors moving at the speed set and moving to the position set by the motor control program. A failed test is defined by the robot's motors not responding to the input speed and position values. The robot's drive motors speed ranges from a value of -225 to 225. The robotic arm's base and joint motors can be set to a position ranging from 0 to 180 degrees from their zero positions. The robotic arm's claw functions as an on/off device. It is either fully open (a command value of 1) or fully closed (a command value of 0). Below is a table that contains the speed and position values that were sent to the Arduino during testing. 

Table 8 – Motor Speed and Position Setpoints 

Table 8 – Motor Speed and Position Setpoints
| Trial | Drive Motor 1 Speed | Drive Motor 2 Speed | Drive Motor 3 Speed | Drive Motor 4 Speed | Arm Base Position | Arm Joint Position | Claw Position |
|-------|---------------------|---------------------|---------------------|---------------------|-------------------|--------------------|---------------|
| 1     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |
| 2     | 0                   | 0                   | 0                   | 0                   | 0                 | 60                 | 1             |
| 3     | 0                   | 0                   | 0                   | 0                   | 90                | 60                 | 1             |
| 4     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |
| 5     | 100                 | 100                 | 100                 | 100                 | 0                 | 0                  | 0             |
| 6     | -100                | -100                | -100                | -100                | 0                 | 0                  | 0             |

Expected Results: 

The Arduino will be able to respond to the Jetson's speed and position commands. During the programing phase, the Jetson's logger function confirmed that it was able to connect to the Arduino. This indicated that the Jetson should be able to send motor and position commands to the Arduino.  

Actual Results: 

Below is a table contating the testing results. These test were conducted after the IEEE SECON competition in the Capstone Lab. 

Table 9 – Motor Speed and Position Results 
Table 9 – Motor Speed and Position Results
| Trial | Drive Motor 1 Speed | Drive Motor 2 Speed | Drive Motor 3 Speed | Drive Motor 4 Speed | Arm Base Position | Arm Joint Position | Claw Position |
|-------|---------------------|---------------------|---------------------|---------------------|-------------------|--------------------|---------------|
| 1     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |
| 2     | 0                   | 0                   | 0                   | 0                   | 0                 | 60                 | 1             |
| 3     | 0                   | 0                   | 0                   | 0                   | 90                | 60                 | 1             |
| 4     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |
| 5     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |
| 6     | 0                   | 0                   | 0                   | 0                   | 0                 | 0                  | 0             |


Interpretation and Conclusions: 

Based on the collected data, the Jetson was partially successful with transmitting data to the Arduino. The Jetson was able to command the robotic arm's motors to a given position; however, it could not command the drive motors to move at a given speed. It is unclear as to what is causing this issue. Two potential theories emerged during testing. The first being that the e-stop may have been pressed during testing. This was not the case, given that the e-stop would remove power from all of the robot's motors when pressed, including the robotic arm's motors. The second theory was that the robot's battery was not charged fully. This could have caused the drive motors to not receive the power that they needed to operate. After letting the battery charge to full, the same experiment was conducted again to similar results. With these two theories ruled out, the likely cause of the issue may be due to a hidden programing mistake with the Arduino's communication program, given that the Jetson was able to successful send commands and manipulate the robotic arm but not the drive motors. 
# Navigation and Object Detection Subsystems 

### Purpose and Justification: 

The experiment to install and test ROS 2 was designed in order to ensure that the robot could move autonomously without crashing into anything. 

### Detailed Procedure: 

In order to operate ROS 2 Nav2 here were the steps that were taken on my Windows 11 laptop: 

Install Windows Subsystem for Linux (WSL) in PowerShell Admin 

Install Ubuntu 24.04 

Create a default Unix account 

Install WSL in Visual Studio (VS) Code Extensions 

Open the Ubuntu terminal and install the ROS version: Kilted kaiju binary package 

Upgrade Ubuntu and set locale 

Install ROS 2 source package and configure repositories 

Install development tool then update the repository cache 

Install ROS-Base: Communication libraries, message packages, command line tools 

Add ROS to .bashrc 

Setup ROS environment then ensure script loaded properly 

Initialize rosdep 

Install build tools 

Install standard Ubuntu development toolchain and cMake 

Install ROS 2 c++ package dependencies 

Install Nav 2 packages 

Test installation 

Install Turtlebot 3 & 4 packages 

Optional test installation again in virtual workspace 

Create package 

Create ROS 2 workspace 

Open workspace in Ubuntu and make .cpp file called “task_manager” under the src dropdown 

Run the example on the ROS 2 website 

Clone the Linorobot2 repository 

Copy and rename the folders with your robot name: example (linorpbpt2_bringup) to (secon_robot_bringup) 

Create the robot description and the robot nav, bringup, and slam 

Create folder structure for hardware 

Source ROS 2 

Clean and rebuild the package 

Test the navigation capabilities without hardware by allowing ROS to initiate your hardware interface 

Integrate ROS 2 onto Jetson 

It’s important to ensure that the Jetson and ROS are operating on the same language (c++, python, etc.) 

### Expected Results: 

The initial hypothesis was that a SLAM Map would be generated by the Object Detection system, and ROS 2 Nav2 would peruse the arena to create a map of the board. The robot would then navigate the course autonomously using ROS and avoid obstacles such as the antennas while collecting the ducks in the arena. 

### Actual Results: 

Present data collected during the experiments in an organized, easy-to-interpret format (tables, graphs, charts). 

The Object Detection system underwent experimental tests to demonstrate that the software could work with the camera to correctly identify differences between the Astro-Ducks, the antenna and the operator’s fingers. The results were conclusive with a few kinks. The system was able to correctly pose the shape of the ducks and the antennas. But upon registering the operator’s hand, the system misjudged it for a pack of hot dogs. Calibrations of the system prompted the system to yield inconclusive results for the operator’s hand. 

A rubber duck on a toy airplane

AI-generated content may be incorrect.   A computer screen with a yellow duck

AI-generated content may be incorrect. 

 

 

The Navigation system underwent experimental tests to demonstrate that the software could establish communication between the nodes in the Ubuntu terminal. The results yielded results that proved that the nodes could talk and listen to each other to receive information. 

 

 

 

### Interpretation and Conclusions: 

Provide a detailed analysis explaining the significance of the results. 

State whether results matched your expectations and explain any discrepancies. 

 

### Purpose: 

Clearly state the exact criteria you intend to measure. 

Criteria should align directly with your project's critical requirements or detailed design objectives. 

Consider the customer's viewpoint: what features or performance attributes are most important from their perspective? 

Anticipate that your instructor may require additional criteria for comprehensive evaluation. 

The criteria our team intended to measure in terms of Detection and Navigation was: 

Time taken to map the arena 

The number of Astro-Ducks identified and collected 

Navigation to the Lunar landing area 

Identification of the antennas and its respective tasks 

Avoidance of collisions with the antenna’s (minus the pushbutton and pressure plate task which the judges viewed collision as necessary) 

Identification of the crater and successful navigation into and out of the crater. 

Navigation back to the start zone 

### Procedure: 

Provide detailed, step-by-step instructions outlining how the experiment will be conducted. 

Include specifics such as required equipment, environmental conditions, and preparation steps. 

Procedures should be detailed enough to ensure repeatability and clarity. 

The procedure established to determine if our robot accomplished its purpose was: 

Map the arena 

Navigate to the Astro-Ducks and drive them to the Lunar Landing Area 

Navigate to the antennas and complete the pushbutton and pressure plate task using the force of the robot 

Identification of the antennas and its respective tasks 

Avoidance of collisions with the antenna’s (minus the pushbutton and pressure plate task which the judges viewed collision as necessary) 

Identification of the crater and successful navigation into and out of the crater. 

Navigation back to the start zone 

 

# Low-Level Controller and Power Subsystems 

### Purpose and Justification: 

The experimental validation of the Low-Level Controller (LLC) and Power subsystems was conducted to verify reliable drivetrain operation and electrical safety, correct movement, and emergency response performance. These tests directly evaluate whether the robot satisfies critical success criteria, such as controlled motion, stable power delivery, and instantaneous emergency stop (E-stop) activation in accordance with the 2026 IEEE  SoutheastCON (SECON) Hardware competition requirements. The successful validation of these criteria demonstrates that both subsystems can support safe, controlled, and competition-compliant operation through accurately executing motion commands, continuously maintaining electrical safety, and immediate manual override to halt  

 

the robot could move, which is necessary to complete all tasks within the 3-minute period. As well as, if the E-stop immediately halted all processes to prioritize the safety of all spectators, equipment, and the surrounding environment. 

  

### Detailed Procedure: 

 

The first test was to connect the motors to varying voltages to check the performance. Though the robot’s power subsystem housed an 11.1V battery, DC Buck Converters were added to the design to provide 24V to the motor drivers. Checking that the motor could safety intake 24V of power was vital to ensure the robot could navigate the arena without overheating or shutting down. This test was especially essentially as the LLC susbsytem was designed with environmental and socio-economic consciousness in mind, and multiple parts were reused from previous capstone projects. A  ------ power supply was connected to the voltage and ground wires of the 150:1 motors. From there, the dial was used to test the performance of varying voltages such as 3.3V, 5V, 10V, and 12V. Once that test was completed, the motors were connected to the Arduino to test the bi-directional capabilities of the DC gearmotors used under the robot. 

 

The second test was to identify the proper activation of the Emergency-stop button (E-stop) required by competition rules and IEEE’s code of ethics. The E-stop's, main functionality was to ensure a manual override process was included in the robot that could immediately halt all processes in the event of unsafe conditions. This test would be conducted by initiating the robot and pressing down the button to see if all movement was halted and processes were terminated.  

 

A third test would be testing the communication of Jetson encoder data to the Jetson in the Global Controller for self-correcting navigation. This test was not conducted as the intended UAV did not come on time, and therefore the AI-detection software initially planned for robot movement was rendered obsolete.  

 

During all tests, the current was noted to ensure that the power subsystem was not overloaded, as it would also be providing power to all other subsystems of the robot.  

 

### Expected Results: 

 

For the first test, it is expected that the motor can run on 12 volts continuously as that was what was specified by the manufacturer. The competition’s max time was 3 minutes, so the motor had to withstand that length of time. When directional instructions were added, it was expected that the motor would rotate forward and backward for the specified amount of time written in the Arduino’s code.  

 

For the second test, it is expected that the E-stop successfully stops all movement of the robot during each trial. As well as the E-stop can be easily push down and cannot be reset until it is both turned and pulled up to ensure intentional resets. This is vital to ensure the robot cannot accidentally be turned on again before all dangerous conditions have been neutralized and mitigated. 

 
### Actual Results: 

Table X - 
| Tested Voltage      | Did the motor Continously Run? |
|---------------------|--------------------------------|
| 3.3 V               | No                             |
| 5 V                 | No                             |
| 10 V                | Yes                            |
| 12 V                | Yes                            |

Table Y -
| Type of Instruction | Successful? |
|---------------------|-------------|
| Forward             | Yes         |
| Backward            | Yes         |
| Waiting/Stopped     | Yes         |

Table Z -
| Robot Movement      | Successful? |
|---------------------|-------------|
| Forward             | Yes         |
| Backward            | Yes         |
| Turning             | Yes         |
|Circling The Crater  | Yes         |

# Conclusion 

Statement of Contributions 

 
