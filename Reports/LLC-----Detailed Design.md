### **DETAILED DESIGN**

##Function of the Subsystem The Low-Level Controller (LLC) subsystem is
responsible for generating and executing real-time motion control for
the ground unit robot during the IEEE 2026 SoutheastCon (SECON) Hardware
Competition. Unlike the high-level perception and decision-making
performed by the Global Controller subsystem, the LLC prioritizes direct
control of electrical components relevant to the real-time movement and
safety protocols of the ground robot. The inclusion of a local
controller reduces both the electrical and computational strain on the
Global controller. By utilizing a microcontroller-based solution, the
LLC can provide stable operation of motor circuitry, rapid fault
response through continuous sensor sampling, and consistent safety
protocol enforcement. Assigning these features to the LLC allows the
Global Controller to focus on accurate higher-level autonomy and
processing. The LLC controls the robot's drive system by receiving
high-level motion commands from the Jetson in the Global subsystem and
translating them into low-level actuator signals, such as speed and
direction, to be executed in real-time. In addition, the LLC
continuously monitors actuator output, component condition, and fault
detection sensors to signal emergency stop conditions if necessary.
Fundamentally, the LLC acts as an intermediary between the Global
Controller and the robot's electromechanical hardware. This detailed
design outlines the LLC's role in ensuring reliable motion control and
operational safety of the robot.

##Specifications and Constraints This section of the detailed design
outlines the different operational, navigational, electrical, ethical,
and professional constraints that significantly impact and direct the
design of the LLC subsystem. These constraints are derived from physical
and electrical limitations, SECON Hardware Competition rules, IEEE
standards, and ethical expectations.

Autonomous Operation Constraints • The LLC shall fully support
autonomous operation after the start state is initiated, in accordance
with the SECON General Vehicle Requirements. • The LLC shall remain
autonomous for the entire three-minute duration allotted for each team
during the competition \[2\]. • Through a closed feedback loop, the LLC
shall provide reliable sensor data to support autonomous motion
correction or emergency stop (E-stop) activation, if necessary \[2\].
Communication Interface Constraints • The LLC shall receive high-level
motion commands from the Global Controller's Jetson over wired USB
communication, as well as send serial telemetry back to the Jetson
\[3\]. • The LLC shall interface with the Mechanical Engineering
subsystem to execute drivetrain related processes. • The LLC shall
interface with the Power subsystem for power delivery, common ground,
and E-stop. Electrical Constraints • The LLC shall isolate the
lower-powered logic of the microcontroller and sensor from the
high-powered components. • The LLC shall NOT power motors directly from
Jetson in the Global Controller subsystem. • The LLC shall operate
within the appropriate power specifications provided by each component
manufacturer. • The LLC shall maintain electrical and communication
compatibility between itself and relevant subsystems, such as the Power
and Global subsystems. • The LLC shall connect all components to a
common ground to prevent reference drift. Ethical Constraints • The LLC
shall prioritize human safety over ALL tasks and objectives, in
accordance with the SECON safety regulations \[2\]. • The LLC shall
continuously monitor actuator output and fault detection sensors to
support immediate halting of all processes via the E-stop feature in
unsafe conditions \[2\]. • The LLC design shall adhere to ethical IEEE
standards by incorporating conscientious solutions that prioritize human
and environmental well-being \[1\]. Motion Constraints • The LLC
effectively controls two motor drivers to generate movement for the
four-wheel drivetrain, in collaboration with the Mechanical Engineering
subsystem. • The LLC shall support controlled speed (acceleration and
deceleration), and directional movement to minimize potential antenna,
boundary, or duck collision, as well as the subsequent loss of points
\[2\]. • The LLC shall independently regulate wheels to preemptively
mitigate undesired movement, such as veering caused by uneven speed,
torque, or traction. Performance Constraints • The LLC shall generate
and execute motor control signals in response to direction and movement
commands from the Jetson, to complete all required tasks within the
specified three-minute time limit \[2\]. • The LLC shall respond to
motion deviations and fault conditions in a timely manner to ensure
efficient and smooth movement throughout the arena. • The LLC shall
maintain stable motor performance during challenging terrain, including
the downhill motion into the crater, which has the potential to generate
back electromotive force (back- EMF). Professional Standards,
Socio-economical Standards, and Responsibility Constraints • The LLC
shall be designed with emphasis on modularity to support individual and
incremental testing. • As well as supporting environmentally sustainable
and cost-efficient design practices, all reused components in the LLC
shall undergo individual testing to verify continuous compliance with
manufacturer's specifications. • The LLC design shall incorporate
redundancy and fallback strategies to improve subsystem's reliability
and flexibility, while minimizing unsafe conditions and points of
failure.

##Overview of Proposed Solution The proposed LLC subsystem solution
implements a microcontroller-based architecture centered around an
Arduino Mega 2560 Rev3 to receive, translate, and execute high-level
motion commands from the Jetson, while simultaneously monitoring
actuator feedback from the local shaft-mounted sensors. The LLC bridges
the gap between high-level autonomy and low-level actuation by enabling
real-time motor control, closed-loop feedback, and safety logic
independent of the Global Controller's computational load.

For efficiency, the Arduino Mega is connected directly to the Jetson via
a wired USB Type A to Type B cable. This interface provides both a 5V
logic power supply and bidirectional data transmission. It is imperative
to note that while the Jetson is viable to power the Arduino, it is
electrically isolated from all high-current components in the LLC.
High-level motion parameters, such as speed and direction, are
transmitted over the USB. Once received, the LLC translates the
instructions into pulse-width modulation (PWM) and direction (DIR)
signals that drive the motor circuitry in real time. The Arduino Mega
can also send telemetry and fault status updates back to the Global
Controller. If the Jetson is found to be insufficient, a DC Drok Voltage
Regulator connected directly to the Power Subsystem has been chosen as a
fallback solution. Motor actuation and control is achieved through two
Pololu Dual VNH5019 motor drivers. The dual motor drivers interface the
Arduino with the four DC Gearmotors that comprise the robot's
drivetrain. The Arduino issues PWM and DIR signals to the motor drivers,
which then switch the high-current battery voltage (VBAT) supplied by
the Power subsystem through H-bridge circuits. This protects the lower
powered components, while also providing sufficient and steady power for
safe operation. High-torque 172:1 DC gearmotors are implemented to
satisfy the electrical and mechanical requirements of the Pololu Dual
motor drivers. The DC gearmotors also comfortably support the torque
performance required for the robot's weight and crater traversal tasks.
The selected gearmotors prioritize controllability and torque over
maximum speed in order to comply with SECON safety objectives,
especially while navigating through uneven terrain. Selecting drivetrain
components from the same manufacturer was an intentional design decision
to reduce integration risk and increase system compatibility. Although
this component selection limits top speed, it provides improved closed
loop-control and aligns with safety and reliability requirements. To
ensure safety and motion accuracy, the LLC has a continuous local
feedback monitoring system using the encoders directly integrated with
the DC gearmotors. Optional bump sensors and external rotary encoders
are included in the design as a possible fallback mechanism in case
additional collision awareness is required beyond the Navigation or
Object Detection subsystems. Encoder data allows real-time monitoring
that can record speed and direction performance to correct for uneven
terrain, overcurrent, stalled motors, generation of back-EMF, and drift
independently for each wheel. The independent sensing of each wheel
allows for more precise correction in response. In especially dire
circumstances, the LLC can assert the E-stop feature and instantaneously
halt all processes to prioritize the safety of all spectators,
equipment, and the surrounding environment. To promote safety and
reliability, the LLC is designed with a modular and redundant
architecture. Components are tested individually and independently
before they are slowly implemented into the design. Incremental
integration allows for the verification of compatibility with other
devices. This approach reduces risk, follows professional engineering
practice, and enables quicker and easier expansion, if necessary, in the
future.

##Interface with Other Subsystems Externally, the LLC exchanges command
and telemetry data with the Global Controller and interfaces with the
Power Subsystems for high-current supply and E-stop access. Internally,
the LLC outputs motor control signals and receives closed-loop feedback
from shaft-mounted sensors.\*\* ###Inputs### -**Global Controller
(Jetson) -\> LLC (Arduino Mega 2560 Rev3): provides high-level motion
commands calculated using the Navigation and Object Detection
subsystems. -Interface: USB Type-A (Jetson) to USB Type-B (Arduino)
-Signal: Digital (USB serial) -Data: command packets of motion, desired
speed, turning, and stop -**Power (Battery) -\> LLC (Pololu Dual
VNH5019): supplies high-current power needed for drivetrain actuation
and provides safety override **-Interface: Wired power bus (VBAT) and
Ground -Signal: Analog DC power -Data: N/A -**Bump Sensors \| internal
(OPTIONAL \| Omron SS-5GL): bolsters feedback system if further
collision detection is required **-Interface: GPIO (wired) -Signal:
Digital (binary) -Data: Events (flags) -**Encoder Sensors \| internal
(OPTIONAL \| Pololu Romi Encoder Pair Kit): enables closed-loop feedback
if increased precision is required **-Interface: GPIO (wired) -Signal:
Quadrature digital (A/B Channels) -Data: encoder counts to derive
rotation, direction, and speed -**DC Buck Converter \| internal
(OPTIONAL \| DROK LM2596 DC): delivers voltage-regulated logic to the
LLC if Jetson USB is found insufficient\*\* -Interface: Power pins
(V`<sub>`{=html}IN`</sub>`{=html}, V`<sub>`{=html}OUT`</sub>`{=html},
GND) -Signal: Analog Power (DC voltage) -Data: N/A

###Outputs### -**LLC (Arduino Mega 2560 Rev3) -\> Global Controller
(Jetson): provides drivetrain status for monitoring and high-level
adjustment** -Interface: USB Type-B (Arduino) to USB Type-C (Jetson)
-Signal: Digital (USB serial) -Data: telemetry -**LLC (Arduino Mega 2560
Rev3) -\> Motor Drivers\| internal (Pololu Dual VNH5019): actual
execution of physical robot movement** -Interface: GPIO (wired) -Signal:
Digital (PWM/DIR) -Data: PWM duty cycle and DIR logic -**Motor Drivers
(Pololu Dual VNH5019) -\> DC Gearmotors -\> Mechanical: convert
electrical power into mechanical motion** -Interface: High-current
wiring -Signal: H-bridge (switched VBAT) -Data: N/A

##Buildable Schematic

Figure 1- Electrical Interconnections and Motor Control Diagram (LLC)
This figure illustrates the interfaces within the LLC and with other
relevant subsystems. This includes power and common ground from the
Power subsystem and the command-report data exchange with the Global's
Jetson. The figure goes in-depth on the signal types and data transfer
between different components such as the motor drivers, motors, and
sensors. Dashed lines represent connections of alternate design
solutions. An incomplete Mechanical Engineering Subsystem is also
referenced. The Arduino will directly connected with components in the
Mechanical subsystem, such as servos.

<p align="center">
Figure 2 -- Arduino Mega 2560 Rev3 Pinout Diagram \[5\] This figure is
an official Arduino diagram that identifies the pin locations and
electrical characteristics of the digital I/O, PWM, and power rails used
by the LLC to interface with the encoder sensors, motor drivers, and
Global Controller subsystem.
</p>
<p align="center">
Figure 3 -- Pololu Dual VNH5019 Motor Driver Shield Interface Diagram
\[6\] This figure shows the manufacturer-provided interface diagram,
which illustrates required logic and power connections between the motor
driver shield and the LLC's Arduino Mega microcontroller.
</p>
<p align="center">
Figure 4 -- Pololu DC Metal Gearmotor (172:1 Reduction) \[7\] This
figure depicts the high-torque brushed DC gearmotor selected for the
robot's drivetrain. The 172:1 gear ratio prioritizes safe autonomous
operation, controllability, and reliable task execution during the
SECON.
</p>
<p align="center">
Figure 5 -- Pololu Romi Encoder Pair Kit \[8\] This figure depicts the
Romi magnetic quadrature encoder used to provide real-time closed-loop
feedback for each motor, enabling continuous motion correction and fault
detection. As the gearmotor has an integrated encoder version, the Romi
is listed as a backup.
</p>

##Flowchart

<p align="center">
Figure 6 - Full Microcontroller Operational Flowchart (LLC)
</p>

Flowchart Walkthrough: • LLC initializes and waits for autonomous start
signal from Global Subsystem • Valid motion command prompts the Arduino
to analyze and translate desired movements, such as speed and direction
• Internal Encoder (and optional bump sensor) samples monitor wheel
speed, direction, and any collision events • LLC computes PWM/DIR from
feedback data • If unsafe conditions are discovered, LLC reduces motor
output or triggers E-stop • Motor commands are continuously updated
until stop command or fault occurs

<p align="center">
Figure 7 - Microcontroller Operational Flowchart (LLC) Close Up Part 1
</p>
<p align="center">
Figure 8 - Microcontroller Operational Flowchart (LLC) Close Up Part 2
</p>

##BOM (Bill of Materials) \| Component \| Manufacturer\| Qty \| Part\|
Unit \| Total \| Reference \| \| --- \| --- \| --- \| --- \| --- \| ---
\| --- \| \|Microcontroller -- Arduino Mega 2560 Rev3 \| Arduino \| 1 \|
A000067 \| \$49.90 \| \$49.90\| \[5\] \| \|Motor Driver -- Dual VNH5019
Motor Driver Shield for Arduino \| Pololu \| 2 \| 2507 \| \$79.95 \|
\$159.90 \| \[6\] \| \|Motor -- 172:1 Metal Gearmotor 25Dx56L mm HP 6V
Pololu\| Pololu \| 4\* \| 1577 \| \$37.95 \| \$151.80 \| \[7\] \|
\|Optional\| \| \| \| \| \| \| \| --- \| --- \| --- \| --- \| --- \| ---
\| --- \| \|Encoder -- Romi Encoder Pair Kit Pololu \| Pololu \| 2† \|
3542 \| \$9.95 \| \$19.90 \| \[8\] \| \|Voltage Regulator - Step-Down
Adjustable DC-DC Switching Buck Converter \| Addicore \| TBD \| LM2596
\| \$2.48 \| TBD \| \[9\] \| \|Bump Switch -Omron SS-5GL \| Omron \| TBD
\| SS-5GL \| \$1.29 \| TBD \| \[10\] \|

<p align="center">
Figure 9 - Table of Materials and Estimated Costs (LLC)
</p>

BOM (notes) The total estimated cost of the LLC subsystem's necessary
components sum to approximately \$209.80. This excludes optional
fallback components, and the components already acquired by the
Mechanical Subsystem. \* Four DC motors are included in the BOM for
inventory purposes. All four have already been acquired by the
Mechanical subsystem team. †Although four encoders would be necessary
for the fallback design solution, only two kits would be needed. Each
Pololu Romi Encoder Pair Kit includes two magnetic encoder boards,
resulting in one mounted on each shaft.

Analysis *Dynamic Motion and Drivetrain Analysis* The drivetrain of the
LLC prioritizes controllability and safe autonomous function in
alignment with the SECON competition requirements and IEEE ethical
responsibilities. The selection of a 172:1 gearmotor supports these
goals by enabling smooth operation during high-load conditions, such as
the uneven terrain of the 2' diameter crater in Area 4 \[2\]. Along with
smooth continuous movement, the high torque available at low speeds
allows for controlled turns, ensuring the robot stays upright and fully
functional throughout the allotted competition time. This gear reduction
allows motors to operate within a stable region of their performance,
preemptively reducing undesirable conditions such as motor stalling and
excessive current draw, which can result in thermal stress on motors,
motor driver H-bridges, and sudden acceleration within the crater
causing back-EMF generation. Lower operation speeds also allow for
improved PWM accuracy at the Arduino. Smaller duty cycle changes produce
more predictable velocity adjustments. This is especially critical for
autonomous operation and navigation, where minor changes could result in
boundary violations or collisions. The use of integrated encoders allows
for independent control of all four wheels, allowing the correction of
mechanical discrepancies, such as uneven wheel friction or misalignment.
This independence improves motion accuracy and easy repetition during
autonomous navigation. Overall, the torque-focused drivetrain satisfies
stable, predictable, and safe controlled movement that satisfies SECON
performance requirements while minimizing electrical, thermal, and
mechanical risk.

*Control and Feedback Analysis* Closed-loop feedback is obtained through
the encoders integrated into the gearmotors. This encoder-based feedback
system allows for continuous monitoring of wheel speed and direction
status, allowing real time error correction on all four wheels through
PWM duty-cycle corrections. As a result, drift, loss of traction, or
load imbalance can be identified and corrected dynamically during
operation. The feedback system significantly improves the accuracy of
the robot's movement, especially compared to open-loop control that
would not allow for self-correction during the event. The ability to
dynamically correct discrepancies is particularly well suited during
low-speed maneuvers and uneven terrain transitions. Independent control
of each wheel further accentuates stability during turning and slope
maneuvers. This accounts for back-EMF, where gravity-induced
acceleration causes the wheels to spin faster than the code instructed.
The closed-loop encoder feedback allows for the identification and
compensation of this effect, maintaining stable descent and preventing
runaway motion. The microcontroller-based architecture allows encoder
sampling, as well as resulting motor control, to operate independently
of the Global Controller's processing load. This separation ensures
reduced stress on the Global Controller's Jetson and rapid response at
the low-level. Assigning real-time control to the Arduino Mega in a
low-level control environment reduces the impact of communication
latency while allowing high-level commands to occur asynchronously and
independently of the high stakes, time-critical local control loops.
This ensures consistent and reliable motor response, even if the Jetson
experiences temporary lag or computational load spikes. Telemetry allows
the Global Controller to monitor drivetrain health without interfering
in real-time execution. In the event of abnormal behavior, the LLC can
reduce motor output, or, more drastically, assert an emergency stop
condition without relying on higher-level intervention. The closed-loop
design and local architecture provide motion correction and fault
response, ensuring stable autonomous performance and satisfying ethical
responsibilities.

*Electrical Safety, Power, and Fault Detection Analysis* The LLC
electrical design isolates high-current motor actuation from low-power
communication and logic circuitry to ensure safe and reliable operation.
Motor power is supplied directly from the Power subsystem through two
dual motor driver shields, while logic-level control is handled by the
Arduino Mega. This separation prevents excessive current draw or
hazardous voltage fluctuations. The Arduino Mega receives logic power
via a USB connection with the Global Controller's Jetson, providing a
simple yet electrically isolated design solution. As a precaution, an
optional LM2596 DC buck converter is included as an alternate design
solution if USB power is deemed unstable or insufficient during testing.
A common ground maintained across all LLC components ensures consistent
voltage reference levels and minimizes communication errors. Safety is
another significant consideration in the LLC design solution. The LLC's
power architecture operates in accordance with the Occupational Safety
and Health Administration's (OSHA) electrical guidelines by maintaining
operating voltages well below the OSHA-defined 50-volt hazard threshold
and by isolating high-current pathways from logic and communication
circuitry \[11\]. OSHA standards emphasize the significance of
minimizing exposure to hazardous voltages to reduce the risk of shock,
overheating, and fire in public environments. The low-speed, high-torque
drivetrain limits kinetic energy, reducing risk to spectators,
equipment, and the environment, in accordance with the IEEE ethical
considerations. Encoder feedback enables early detection of non-ideal
conditions, such as stalled motors, unexpected reversals, loss of
motion, non-responsiveness, excessive deceleration, or unintended
movement. One critical concern is back-EMF. Downhill motion or rapid
deceleration can cause DC motors to act as generators and produce
regenerative voltage. The motor driver H-bridges have hardware-level
protection that can safely respond to these conditions. The early
detection enables corrective action before damage occurs or hazardous
behavior compounds. In addition, encoder feedback proactively monitors
and reduces PWM output to minimize the need of regenerative
intervention. Optional bump sensors were also included in the design as
a fallback solution, providing additional fault detection in case more
monitoring is needed. Electrical isolation, layered fault detection, and
protective control strategies ensure safe operation and reliable
performance by preemptively minimizing faults and quick correction
before drastic escalation.

*Design Resilience and Overall Functionality* The modular nature of the
LLC design supports incremental testing, allowing for individual
components to be independently and gradually verified before full system
integration. This approach reduces integration risk and enables
efficient troubleshooting. It also supports any modifications, to
existing or new components, without fundamental redesign. The use of
widely supported, well documented, and consistent manufacturers further
reduces integration risk and simplifies troubleshooting. The LLC
subsystem design demonstrates compliance with all identified ethical,
electrical, and performance constraints. Combining a torque-focused
drivetrain, closed-loop motor control system, electrical and computation
load balance, and layered safety mechanisms, the design is well suited
to execute real-time tasks during the SECON competition. Modular and
redundant architecture further supports a resilient design that is safe
and reliable for autonomous robot movement.

##References \[1\] IEEE, "IEEE Code of Ethics \| IEEE," Ieee.org, 2020.
Available: https://www.ieee.org/about/corporate/governance/p7-8 \[2\]
IEEE, "2026 IEEE SoutheastCon Hardware Competition Ruleset," Jun. 15,
2025. Available:
http://www.tech-uofm.info/announcements/2026%20IEEE+SoutheastCon+Hardware+Competition+Ruleset_Draft_upload.pdf
\[3\] NVIDIA, "NVIDIA Jetson AGX Orin," NVIDIA. Available:
https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/
\[4\] "Conceptual Design," Github.com, 2026. Available:
https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Team%207%20Conceptual%20Design.md.
\[5\] Arduino, "Arduino Mega 2560 Rev3," Arduino Official Store, 2021.
Available: https://store.arduino.cc/products/arduino-mega-2560-rev3
\[6\] "Pololu Dual VNH5019 Motor Driver Shield for Arduino," Pololu.com,
2025. Available: https://www.pololu.com/product/2507 \[7\] "172:1 Metal
Gearmotor 25Dx56L mm HP 6V," Pololu.com, 2026. Available:
https://www.pololu.com/product/1577/. \[8\] "Romi Encoder Pair Kit, 12
CPR, 3.5-18V," Pololu.com, 2017. Available:
https://www.pololu.com/product-info-merged/3542 \[9\] Addicore,
"Addicore LM2596 Adjustable DC-DC Switching Buck Converter," Addicore.
Available:
https://www.addicore.com/products/lm2596-step-down-adjustable-dc-dc-switching-buck-converter
\[10\] DigiKey, "SS-5GL \| DigiKey Electronics," DigiKey Electronics,
2026. Available:
https://www.digikey.com/en/products/detail/omron-electronics-inc-emc-div/SS-5GL/272367?gclid=53c6f457a2cd1c91fd799d26b90be633&gclsrc=3p.ds&msclkid=53c6f457a2cd1c91fd799d26b90be633.
\[11\] M. Duvall, "Guarding requirements for 50 volts or more DC. \|
Occupational Safety and Health Administration," www.osha.gov, Sep. 04,
2015. Available:
https://www.osha.gov/laws-regs/standardinterpretations/2015-09-04
