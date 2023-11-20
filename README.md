# gnss_preprocessing

## FlowChart

```mermaid
flowchart TD
    A[Start] --> B[Initialize ROS Node: gnss_preprocessing]
    B --> C[Create GnssPreprocessingCore Object with Initial GPS Coordinates]
    C --> D[Set Up ROS Publishers and Subscriber]
    D --> E[Main ROS Loop]
    E --> F[Check if ROS is OK]
    F -->|Yes| G[Wait for GNSS Data Callback]
    G --> E
    F -->|No| H[End]

    subgraph GnssPreprocessingCore
        I[Constructor: Initialize Variables and Set Up ROS Publishers and Subscriber]
        J[gnssCallback: Process GNSS Data]
        K[Convert Lat/Lon/Alt to ECEF Coordinates]
        L[Convert ECEF to ENU Coordinates]
        M[Publish GNSS Pose and Path]
        N[Transform Broadcast: /odom to /base_link]
    end

    I --> J
    J --> K
    K --> L
    L --> M
    M --> N

```

## State Transition Diagram

```mermaid
stateDiagram-v2
    [*] --> Uninitialized
    Uninitialized --> Initialized: Constructor
    Initialized --> ProcessingGNSSData: gnssCallback()
    ProcessingGNSSData --> Initialized: After Callback Processing
    ProcessingGNSSData --> [*]: Destructor
    Initialized --> [*]: Destructor
```

## 

```mermaid
sequenceDiagram
    participant Main
    participant ROS
    participant GnssPreprocessingCore
    participant ROS_NodeHandle
    participant ROS_Publisher
    participant ROS_Subscriber
    participant ROS_TF_Broadcaster
    participant Callback

    Main->>GnssPreprocessingCore: Instantiate with (lat0, lon0, hig0)
    activate GnssPreprocessingCore
    GnssPreprocessingCore->>ROS_NodeHandle: Create NodeHandle
    GnssPreprocessingCore->>ROS_Publisher: Advertise /gnss_pose and /gnss_path
    GnssPreprocessingCore->>ROS_Subscriber: Subscribe to /fix
    GnssPreprocessingCore->>ROS_TF_Broadcaster: Setup odom_to_baselink_broadcaster
    deactivate GnssPreprocessingCore

    loop ROS spin loop
        Main->>ROS: ros::spinOnce()
        ROS->>ROS_Subscriber: Check for new messages on /fix
        alt New message received
            ROS_Subscriber->>GnssPreprocessingCore: gnssCallback(new message)
            activate Callback
            Callback->>GnssPreprocessingCore: Process message
            GnssPreprocessingCore->>ROS_Publisher: Publish to /gnss_pose and /gnss_path
            GnssPreprocessingCore->>ROS_TF_Broadcaster: Send transform (odom_to_baselink_trans)
            deactivate Callback
        end
    end

    Main->>GnssPreprocessingCore: Destructor call
    deactivate GnssPreprocessingCore
```
