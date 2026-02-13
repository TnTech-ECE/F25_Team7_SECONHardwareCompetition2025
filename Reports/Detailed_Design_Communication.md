# Communication Subsystem:

## Function of the Subsystem:

This subsystem deals with wireless data transfer between the robot, UAV, and Earth.  The data being transferred includes flight control data from the robot to the UAV through the Crazyradio PA, Camera data from the UAV to the robot through WiFi, and satellite data from the robot to the UAV through the Crazyradio PA, then from the UAV to the Earth through IR LEDs.

# Specifications and Constraints:

The Earth uses a library called IRremote that is compatible with certain microcontrollers [1]
The UAV lets us control it through the Crazyradio PA [2]
The UAV has to transmit the satellite data to the Earth [1]
All communication between the UAV, robot, and Earth has to be wireless
The range for each wireless system needs to reach across the longest part of the arena
The Wifi is capable of supporting a 20fps video data stream using a standard video stream (5-8 Mbps) [7]
While using radio frequencies, we will avoid using frequencies deemed within the illegal range  from the FCC and ITU-R[8]

# Overview of proposed Solution:
The robot will connect with the UAV in 2 ways: sending flight controls and satellite data through the Crazyradio PA, and collecting camera data through WiFi (hosted on the camera). The connection will run with HTTP protocol in mind (here is an example with it in action) [9].  When the Jetson sends info about the satellite data over the PA, the UAV will send that data through IR LEDs that are attached and pointed at the earth. We will use the IRremote library [3] on the UAV to transmit the data, ensuring that we are sending to register 0xBB and using the 4-bit codes provided by the game document. [1] There will be 5 IR LEDs pointed in the direction Earth will be in 10-degree deviations to maximize the area covered in light to ensure that the information gets through. We will use the IO1 pin to turn on/off the transistor and have all 5 IR LEDs wired in parallel. This will require a capacitor connected to the system to have enough current to last the data transfer. We will test this design using the test arena that we are building for this project.  However, there is no way for the UAV to know whether or not the data was sent successfully, so we will shoot for a 95% success rate when testing.

# Interface:

This subsystem interfaces directly with the Global Controller through the wifi module with an HTTP connection, and Crazyradio PA on the Jetson computer through USB 3.2 Type A. We will be using the UAV's given API to control its position (Flight Control).  This subsystem also communicates with Earth through IR LEDs that are connected to the UAV through the Vcc (3V) pin for power and the IO8 pin for control using the IRRemote Library.

# Schematics:

<img width="781" height="768" alt="Communication_Schematic" src="https://github.com/user-attachments/assets/63063a83-ebb8-4a90-ad58-471c0642da10" />


# Flowchart:

<img width="866" height="467" alt="Communication_Flowchart" src="https://github.com/user-attachments/assets/10ee9e35-9636-4338-9a71-4acc65f4118b" />

# Pseudocode:

<img width="932" height="410" alt="DDC PseudoCode" src="https://github.com/user-attachments/assets/e97bb30f-3ce0-4e62-bb8c-809f71e5c147" />

# BOM:

| Name of the Part: | Manufacturer: | Distributor:| Part Number: | Distributor Part Number: | Quantity: | Price: | Website:| Component Name: |
|---|---|---|---|---|---|---|---|---|
| IR LEDs | Vishay Semiconductor Opto Division | Digikey | TSHF5210 | 751-1210-ND | 5 | $3.45 | https://www.digikey.com/en/products/detail/vishay-semiconductor-opto-division/TSHF5210/1681345 | D1-5 | 
| 33 Ohm Resistor | YAGEO | Digikey | MFR50SFTE52-22R| 13-MFR50SFTE52-22RCT-ND | 5 | $0.50 | https://www.digikey.com/en/products/detail/yageo/MFR50SFTE52-33R/9151419| R2-6 |
| 10k Ohm Resistor | YAGEO | Digikey | MFR-25FRF52-10K|13-MFR-25FRF52-10KCT-ND | 1 |0.1 | https://www.digikey.com/en/products/detail/yageo/MFR-25FRF52-10K/14626 | R1 |
| 47uF Capacitor | Chinsen (Elite)|Digikey|SM1C470MP20511U|4191-SM1C470MP20511UCT-ND | 1 | 0.12 | https://www.digikey.com/en/products/detail/chinsan-elite/SM1C470MP20511U/16496724 | C1 |
| Transistor | Microchip Technology |Digikey|VN3205N3-G|VN3205N3-G-ND | 1 | $1.67 | https://www.digikey.com/en/products/detail/microchip-technology/VN3205N3-G/4902407 | M1 |
| Crazyradio PA | Bitcraze | Bitcraze | 114990112 | 114990112 | 1 | $38.00 | https://store.bitcraze.io/products/crazyradio-pa | N/A |

Total: $43.84

# Software Prototyping:

Prototyping will start with getting a WiFi connection with the UAV and Jetson and sending data between them. When we get photos, next will be testing the IRremote library on the UAV.  This can be done using the test arena, which can give us feedback on how the data transmission is working. Finally, we will test flight control communication using flight control data from the navigation subsystem.

# Analysis:

Looking at the datasheets and library descriptions, all of these tools are compatible with each other; the IRremote library works with STM32 microcontrollers, the Crazyradio PA and program both work with the Jetson computer [2], and the camera is compatible with the Jetson computer [4]. The only thing that needs to be designed is the IR transmission to the Earth. I used the recommended IR LED listed in the game manual [1]; having 5 LEDs in parallel will ensure any error in flight can still transmit the necessary data. The LEDs have a current(max) of 100mA and a forward voltage of 1.4V [5], so we will be inputting 50mA per IR LED. According to the STM32 datasheet [6], the Vcc is 3V with a current draw of 100mA, so we will be introducing a capacitor to supply temporary extra current when it is pushing data.  Assuming a duty cycle of 50% and a length of time of 0.5ms, there needs to be 125mA for 0.5ms, which is 62.5uJ, which gives us a final capacitance of 20.3uF.  We are rounding this to 47uF to give some buffer and to use a standard capacitor. With the forward voltage and current, we need to give each LED a 32 Ohm resistor, which we will change to a 33 Ohm resistor to keep it standardized.

References:
[1] IEEE Region 3, IEEE SoutheastCon 2026 Hardware Competition Ruleset, IEEE Region 3, 2025. Available: https://ieee-region3.org/southeastcon

[2] Bitcraze, “Crazyradio PA 2.4 GHz USB dongle,” Rev3, 2022-06-21, Available: https://www.bitcraze.io/documentation/hardware/crazyradio_pa/crazyradio_pa-datasheet.pdf 

[3] Ken Shirriff, “Arduino-IRremote,” GitHub, 2009. [Online]. Available: https://github.com/Arduino-IRremote/Arduino-IRremote 

[4] Team 7, “Team 7 Conceptual Design,” GitHub, 2025. [Online]. Available: https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Conceptual_Design/Reports/Team%207%20Conceptual%20Design.md 

[5] Vishay, “High Speed Infrared Emitting Diode, 890 nm, Surface Emitter Technology,” 91000, 01-Jan-2005, Available: https://www.vishay.com/docs/81313/tshf5210.pdf 

[6] STMicro, “STM32F405xx STM32F407xx,” DS8626 Rev 10, 2024, Available: https://www.st.com/resource/en/datasheet/dm00037051.pdf 

[7] K. Fann, “What Streaming Bandwidth Do You Need? How to Calculate,” BroadbandNow, Sep. 17, 2025. https://broadbandnow.com/guides/streaming-bandwidth 

[8] G. Hardesty, “Legal & Illegal Frequencies & Channels In the United States,” Data-alliance.net, Dec. 29, 2025. https://www.data-alliance.net/blog/legal-illegal-frequencies?srsltid=AfmBOoqBw0Xxdx2-uO0sYl1e4qBVuz7ditVThJeH2fmH4tndBrsy5nIl 
[9]“Camera Usage for Sense Version | Seeed Studio Wiki,” Seeedstudio.com, 2025. https://wiki.seeedstudio.com/xiao_esp32s3_camera_usage/ 
