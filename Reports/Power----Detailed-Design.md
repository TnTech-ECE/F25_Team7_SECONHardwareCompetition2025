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
- **Source: Zeee 3S LiPo battery**
  - Nominal voltage: 11.1 V (3S1P)
  - Voltage range: 9.0–12.6 V
  - Capacity: 5200 mAh (5.2 Ah)
  - Discharge rating: 80 C (max current >> system needs)
  - Connector: XT60 hard-case pack (amazon.com)

- **24 V bus (VBUS_HI):**
  - Nominal output: 24 V DC
  - Regulation: 24 V ± 5% under loads from 0 A to 8 A
  - Continuous power rating: ≥ 192 W
  - Peak power rating (e.g., motor stall / acceleration): ≥ 576 W

- **Drivetrain load:**
  - Motors: Pololu 150:1 37Dx73L mm 24 V with 64 CPR encoder and helical pinion
  - No-load at 24 V: ~0.1 A
  - Stall current at 24 V: 3 A per motor
  - Number of motors: 4
  - Total stall current @ 24 V: 3 A × 4 = **12 A**

- **19 V rail (Jetson Orin):**
  - Voltage: 19 V ± 5%
  - Continuous current: ≥ 5 A
  - Short-term peak: ≥ 7 A

- **12 V rail (L298N):**
  - Voltage: 12 V ± 5%
  - Continuous current: ≥ 3 A
  - Peak current: ≥ 6 A

- **7.4 V rail (L298N for 2 normal servos):**
  - Voltage: 7.4 V ± 5%
  - Continuous current: ≥ 5.1 A
  - Peak current: ≥ 10.2 A

- **5 V rail (5V – HSR-1425CR):**
  - Voltage: 5 V ± 5%
  - Continuous current: ≥ 0.7 A
  - Peak current: ≥ 1.4 A

- **3.3 V rail (STM32F405, nRF51822, BMI088, BMP388, APDS9960):**
  - Voltage: 3.3 V ± 5%
  - Continuous current: ≥ 0.133 A
  - Peak current: ≥ 0.266 A

- **Efficiency & runtime targets:**

## Safety, Standards, and Ethical Constraints

- **Over-current protection:**
  - Input fuse sized to ~1.25–1.5× nominal battery current to protect wiring and downstream circuitry.

- **Emergency Stop:**
  - E-Stop must remove power from all motors, actuators, and servos by interrupting the high-current path from the battery positive after the main fuse.

- **LiPo-specific safety:**
  - Pack must not be over-charged or over-discharged and must be charged with a proper balance charger.
  - Battery must be mechanically protected to reduce risk of puncture or short.
  - Pack must be removed from robot during charging. Robot is prohibited from functioning during charging.

- **Standards:**
  - The design follows the intent of:
    - **ISO 13849-1** — safety of machinery (safety-related parts of control systems)
    - **IEC 60950-1** — IT equipment safety  
    (in terms of fusing, emergency stop, and creepage/clearance)

- **Socio-economic:**
  - Power subsystem BOM must remain within a budget of **$400 USD** to keep the project feasible for a student team.

---

# Physical and Integration Constraints

- Maximum envelope for the power subsystem: **4 × 8 × 7 in** within the chassis.
- Must accommodate at least one **Zeee 3S 5200 mAh pack**.
- Use **XT60** for battery connection and appropriately rated **Phoenix Contact** connectors for the 24 V bus and major rails.
- Wire gauge, PCB traces, and connector ratings must meet or exceed calculated continuous and peak currents with ~20% margin.

---

# Overview of Proposed Solution

The chosen architecture:


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
