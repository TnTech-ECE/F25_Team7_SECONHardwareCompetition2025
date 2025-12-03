# Object Detection Detailed Design


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

  - **Inputs to Object Detection**
    - Low-resolution grayscale frames (160x120) for SLAM initalization
    - High-resolution RGB frames (320x240 or 640x480) for object detection

  - **Communication Method**
    - Wi-Fi UDP or TCP stream using the drone's ESP32-S3
    - Images are packeted and fowarded to robot's communication subsytem

  - **Purpose**
    - Provide real-time ariel imagery of the areana
    - Detect the Astro-Ducks, antennas, crater, and basic arena geometry
    - Initialize SLAM map for global controller subsystem

### *Interface with Communication Subsystem:*

  - **Inputs to Object Detection**
    - Image packets from the Crazyflie 2.1+ drone
    - Robot Intel RealSense camera frames
    - Status messages from global controller

  - **Outputs from Object Detection**
    - Compressed detection results
        - Object class (duck, antenna, dish, ...)
        - Bounding box coordinates (x_min, y_min, width, height)
        - Confidence score
        - Object height classification
        - LED color classification (red, blue, green, white)
    - Feature points for SLAM map
        - ORB/FAST keypoints
        - Known landmark coordinates (crater rim, walls, antennas)

  - **Communication Method**
    - ROS2 topics or TCP messaging to the communicatins stack
    - JSON or Protobuf for lightweight transmission

  - **Purpose**
    - Allows global controller to merger detected objects with navigation and SLAM data
    - Maintain consistent communucation between drone, robot sensors, and processing nodes.

### *Interface with Global Controller:*

  - **Outputs to Global Controller**
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

  - **Communication Method**
    - ROS2 topic: /object_detection/results
    - Global Controller publishes callbacks
        - "Naviagate to this coordinate"
        - "This object belongs to task #3"
        - "Update global map"

  - **Purpose**
    - Alows mission-level decision making
    - Support path planning and object prioritization
    - Enables SLAM updates based on visual beaconing

### *Interface with Navigation Subsystem:*

  - **Outputs from Object Detection**
    - Approximate (x,y) coordinates of detected objects
        - Converted from image-space to world-space using camera calibration
    - Obstacle detections
        - Flags geometric shapes or unexpected objects in the competition arena
    - Distance-to-object estimates

  - **Communication Method**
    - ROS2 TF messages
    - Pose updates to "/nav/object_positions"

  - **Purpose**
    - Allows Navigation to generate a path directly to the antenna of duck
    - Real-time obstacle avoidance
    - Link visual detections with wheel odometry and  IMU data


### *Interface with Local Controller:*

  - **Outputs**
    - Final target pixel alignment values
        - Used for fine-positioning near an object
        - Example: "Object is 24 pixels left of center. Rotate left 3 deg"
    - Target distance
        - Enables precise approach maneuvers when stopping in front of an antenna

  - **Communication Method**
    - ROS2 topic or serial pass-through direct message

  - **Purpose**
    - Allow the robot to precisely align itself with the antenna for task completion
    - Ensure smooth docking and payload-level precision

### *Interface with Safety and Power:*

  - **Outputs**
    - Sends flag if camera fails or image quality drops

  - **Purpose**
    - Fail-safe operation
    - Allows the robot to fall back to default waypoint navigation if camera becomes unavailable


## Buildable Schematic 

Below are the images of the psuedocode for the main object detection algorithms that shall be used by the robot and drone respectively.

### Algorithm 1: Drone ESP32-S3 Image Capture and Transmission

    BEGIN

    initialize_camera_esp32()
    initialize_wireless_link()

    running = TRUE

    WHILE running == TRUE DO

        frame = capture_camera_frame()

        IF frame == NULL THEN
            CONTINUE
        ENDIF

        frame_compressed = compress_frame_to_jpeg(frame)

        metadata.timestamp  = get_current_time()
        metadata.frame_id   = get_next_frame_id()
        metadata.drone_pose = read_drone_pose()

        packet = build_image_packet(frame_compressed, metadata)
        send_packet_to_ground_robot(packet)

        running = check_flight_running_flag()

    ENDWHILE

    shutdown_camera_esp32()
    shutdown_wireless_link()

    END


### Algorithm 2: Robot Main Object Detection Loop

    BEGIN

    initialize_camera()
    initialize_drone_link()
    model = load_yolov5_nano_model()

    running = TRUE

    WHILE running == TRUE DO

        local_frame = capture_local_frame()
        drone_frame = receive_drone_frame_if_available()

        IF drone_frame exists THEN
            frame_to_process = drone_frame
        ELSE
            frame_to_process = local_frame
        ENDIF

        IF frame_to_process == NULL THEN
            CONTINUE
        ENDIF

        input_tensor = preprocess_image(frame_to_process)
        raw_detections = run_yolov5_inference(model, input_tensor)

        processed = process_and_estimate_pose(raw_detections, frame_to_process)

        FOR each detection IN processed DO
            msg = build_detection_message(detection)
            send_to_global_controller(msg)
            send_to_navigation(msg)
        ENDFOR

        running = check_system_running_flag()

    ENDWHILE

    shutdown_camera()
    shutdown_drone_link()
    unload_model(model)

    END


### ALgorithm 3: Object Detection Post-Processing and Pose Estimation


    BEGIN

    detections = []

    FOR each det IN raw_detections DO
        IF det.score < CONFIDENCE_THRESHOLD THEN CONTINUE
        IF det.class NOT IN {DUCK, ANTENNA, TASK_OBJECT} THEN CONTINUE
        APPEND det TO detections
    ENDFOR

    detections = non_max_suppression(detections, NMS_IOU_THRESHOLD)

    processed = []

    FOR each det IN detections DO

        u = (det.bbox.x_min + det.bbox.x_max) / 2
        v = (det.bbox.y_min + det.bbox.y_max) / 2

        pose_world = image_point_to_world(u, v, frame)
        pose_filtered = temporal_filter(det.class, pose_world)

        record.type = det.class
        record.bbox = det.bbox
        record.confidence = det.score
        record.pose_world = pose_filtered

        APPEND record TO processed

    ENDFOR

    RETURN processed

    END


## Flowchart

### Drone Object Detection Flowchart:
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/DroneObjectDetectionFlowchart.png)


### Robot Object Detection Flowchart: 
![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/RobotObjectDetectionFlowchart.png)

![image](https://github.com/TnTech-ECE/F25_Team7_SECONHardwareCompetition2025/blob/Object-Detection----Detailed-Design/Reports/Poster%20Template/Images/Robot2ObjectDetectionFlowchart.png)


## BOM

| Item                          | Quantity | Cost Per  | Total     | URL |
|------------------------------|----------|-----------|-----------|-----|
| Intel RealSense D435 RGBD    | 1        | $322.50   | $322.50   | [Link](https://www.digikey.com/en/products/detail/realsense/82635AWGDVKPMP/9926003gclid=e94e5a4519a81e0a76cf336a387962ef&gclsrc=3p.ds&msclkid=e94e5a4519a81e0a76cf336a387962ef) |
| Garmin LIDAR-Lite v4 LED     | 3        | $59.99    | $179.97   | [Link](https://www.garmin.com/en-US/p/610275/) |
| Photoresistor GL12528 (set)  | 10       | $22.98    | $22.98    | [Link](https://www.amazon.com/Juried-Engineering-Photoresistor-GL12528-Sensitive/dp/B08F3WPNPF) |
| Seeed Studio XIAO ESP32-S3 Sense | 1    | $12.59    | $12.59    | [Link](https://www.seeedstudio.com/XIAO-ESP32S3-Sense-p-5639.html) |
|  | || **$538.04**||


## Analysis

Deliver a full and relevant analysis of the design demonstrating that it should meet the constraints and accomplish the intended function. This analysis should be comprehensive and well articulated for persuasiveness.

## References

