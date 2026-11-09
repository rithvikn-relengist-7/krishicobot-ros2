# Architecture and data flow

## 1. Mobile-base navigation

```mermaid
flowchart LR
    SCAN["LaserScan /scan"] --> N["BugNavigator"]
    ODOM["Odometry /odom"] --> N
    N -->|geometry_msgs/Twist| VEL["/cmd_vel"]
    VEL --> DRIVER["Mobile base driver"]
```

The node estimates the robot pose from odometry, checks valid LiDAR ranges against a configurable minimum obstacle distance, and selects either direct waypoint tracking or a simple wall-following response. Waypoints are currently hard-coded examples.

## 2. RGB-D perception

```mermaid
flowchart LR
    C["RGB Image"] --> HSV["HSV threshold + spatial ROI"]
    HSV --> CONTOUR["Contour candidates"]
    D["Depth Image"] --> ROI["Depth ROI sampling"]
    CONTOUR --> PROJECT["Pinhole projection"]
    ROI --> PROJECT
    TF["Camera-to-base transform"] --> TRANSFORM["TF2 transform"]
    PROJECT --> TRANSFORM
    TRANSFORM --> FRAME["Broadcast candidate TF frame"]
```

The detector finds colour/region candidates, samples nearby depth, projects the pixel/depth measurement using camera intrinsics and transforms the point into the target frame.

## 3. UR5 waypoint controller

```mermaid
flowchart LR
    TF["base_link → ee_link TF"] --> ERR["Position + quaternion error"]
    WP["Configured waypoint"] --> ERR
    ERR --> LIMIT["Linear/angular speed limits"]
    LIMIT --> PUB["/delta_twist_cmds"]
    PUB --> DRIVER["UR5 driver"]
```

The controller moves through a sequence of example poses and pauses briefly when a waypoint tolerance is met. Motion is disabled by default in this repository.

## Integration boundary

These are three separate ROS nodes. The supplied material does not establish that all three were launched together or that their interfaces were integrated into one complete deployed system.
