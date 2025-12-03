# Power Subsystem


The Power Subsystem is responsible for storing, converting, distributing, and protecting electrical energy for the entire robot. It supplies:

- 	The 24 V drivetrain motors (Pololu 150:1 37D 24 V metal gearmotors with encoders).
-	The motor drivers (Pololu Dual VNH5019 motor driver shield).
-	The Jetson Orin Nano global controller, and Arduino nano as the supporting microcontroller.
-	All sensors, servos, and auxiliary electronics listed in the PowerSheet.



## The subsystem:


1.	Accepts energy from a removable 3S LiPo pack (Zeee 11.1 V 5200 mAh, 80 C, XT60 connector). 
2.	Uses a boost converter to generate a regulated 24 V bus (VBUS_HI) for the motors and downstream converters.
3.	Uses buck converters from the 24 V bus to generate lower-voltage rails (3.3V, 5V, 7.4V, 12V, 19V). Contacts made through screw terminals, allowing for test points on each rail.
4.	Implements a hard emergency stop (E-Stop) and fusing that can safely isolate the battery from all loads. 
5.	Monitors voltage and current and reports power-health information to the controller.





## Specifications and Constraints

### Electrical Specifications
- Source: Zeee 3S LiPo battery
--	Nominal voltage: 11.1 V (3S1P)
--	Voltage range: 9.0–12.6 V
--	Capacity: 5200 mAh (5.2 Ah)
o	Discharge rating: 80 C (max current >> system needs)
o	Connector: XT60 hard-case pack amazon.com+1
•	24 V bus (VBUS_HI):
o	Nominal output: 24 V DC
o	Regulation: 24 V ± 5 % under loads from 0A to 8A
o	Continuous power rating: ≥ 192 W
o	Peak power rating (e.g., during motor stall / acceleration): ≥ 576 W
•	Drivetrain load:
o	Motors: Pololu 150:1 37Dx73L mm 24 V with 64 CPR encoder and helical pinion
o	No-load at 24 V: ~0.1 A
o	Stall current at 24 V: 3 A per motor Pololu+1
o	Number of motors: 4
o	Total stall current @ 24 V: 3 A × 4 motors = 12 A
•	19V rail (Jetson Orin):
o	Voltage: 19 V ± 5 %
o	Continuous current: ≥ 5 A jetson
o	Short-term peak: ≥ 7 A
•	12V rail (L298N):
o	Voltage: 12V ± 5 %
o	Continuous current: ≥ 3 A
o	Peak current: ≥ 6A
•	7.4V- rail (L298N for 2 Normal Servos)
o	Voltage: 7.4V ± 5 %
o	Continuous current: ≥ 5.1 A
o	Peak current: ≥ 10.2 A
•	5V rail (5V- HSR-1425CR)
o	Voltage: 5V ± 5 %
o	Continuous current: ≥ 0.7 A
o	Peak current: ≥ 1.4 A
•	3.3V rail (STM32F405, nRF51822, BMI088, BMP388, APDS9960)
o	Voltage: 3.3V ± 5 %
o	Continuous current: ≥ 0.133 A
o	Peak current: ≥ 0.266A
•	Efficiency & runtime targets:
<img width="468" height="648" alt="image" src="https://github.com/user-attachments/assets/8af28c16-c827-44a0-9263-6955c2d3c228" />



## Specifications and Constraints

This section should provide a list of constraints applicable to the subsystem, along with the rationale behind these limitations. For instance, constraints can stem from physics-based limitations or requirements, subsystem prerequisites, standards, ethical considerations, or socio-economic factors.

The team should set specifications for each subsystem. These specifications may require modifications, which must be authorized by the team. It could be necessary to impose additional constraints as further information becomes available.

Every subsystem must incorporate at least one constraint stemming from standards, ethics, or socio-economic factors.


## Overview of Proposed Solution

Describe the solution and how it will fulfill the specifications and constraints of this subsystem.


## Interface with Other Subsystems

Provide detailed information about the inputs, outputs, and data transferred to other subsystems. Ensure specificity and thoroughness, clarifying the method of communication and the nature of the data transmitted.


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
