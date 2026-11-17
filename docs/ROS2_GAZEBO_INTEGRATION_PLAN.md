# AstroMesh-X — ROS 2 & Gazebo Integration Architecture Plan

## 1. Executive Summary & Host Environment Audit

During Milestone 0 and Milestone 4 of the 72-Hour Engineering Sprint, the host operating environment was probed to evaluate the physical availability of Linux virtualization, ROS 2, and Gazebo.

### 1.1 Empirical Environment Diagnostic
- **Operating System:** Windows 11 Home / Pro (x86_64)
- **Toolchain:** MSYS2 UCRT64 GCC 16.2.0, Ninja 1.13.2, CMake 4.4.4, Python 3.14.0
- **Virtualization Diagnostic:**
  ```powershell
  wsl -d Ubuntu-22.04 -- uname -a
  ```
  **Output Observed:**
  ```text
  Wsl/Service/E_UNEXPECTED
  ```
- **Finding:** Windows Subsystem for Linux (WSL2) encounters a catastrophic hypervisor/VM service exception (`E_UNEXPECTED`), indicating disabled Windows virtualization drivers, missing Hyper-V virtualization extensions, or corrupted LxssManager services on the host machine.
- **Native Windows State:** No native Windows ROS 2 Humble/Iron installation or Gazebo Fortress binaries exist in Windows PATH.

### 1.2 Non-Destabilization Mandate
Per Sprint Rule 10 ("Check WSL2/Ubuntu/ROS 2/Gazebo availability before attempting installation; do not destabilize working Windows builds") and AGENTS.md, attempting invasive host Windows Hyper-V driver reinstallations or multi-gigabyte unverified package installations risks rendering the working native toolchain inoperable.

Instead, AstroMesh-X:
1. Operates autonomously and deterministically on native Windows via compiled standalone C++17 executables (`rover_daemon.exe`, `sim_runner.exe`) communicating over real IPv4 UDP network sockets.
2. Provides this rigorous, complete architectural specification and drop-in package blueprint for deploying AstroMesh-X into standard ROS 2 Humble and Gazebo Fortress environments once a Linux or Docker host is provisioned.

---

## 2. ROS 2 Package Architecture

AstroMesh-X is organized into three standard ROS 2 packages adhering to ROS 2 Humble and REP-2004 guidelines:

```text
ros2_ws/src/astromesh/
├── astromesh_interfaces/           # Custom IDL message and service definitions
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── msg/
│       ├── LandmarkObservation.msg
│       ├── LandmarkObservationArray.msg
│       ├── IbltSync.msg
│       ├── CandidateProposal.msg
│       ├── CandidateResponse.msg
│       ├── BoundaryState.msg
│       └── SwarmTelemetry.msg
├── astromesh_core/                 # Core SLAM and Distributed Networking Node
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── include/astromesh_core/
│   │   └── rover_slam_node.hpp
│   └── src/
│       ├── rover_slam_node.cpp
│       └── main.cpp
└── astromesh_gazebo/               # Gazebo Fortress simulation models, worlds, and launch
    ├── CMakeLists.txt
    ├── package.xml
    ├── launch/
    │   ├── lunar_world.launch.py
    │   └── multi_rover_swarm.launch.py
    ├── urdf/
    │   ├── rover_chassis.xacro
    │   └── rover_sensors.xacro
    └── worlds/
        └── lunar_shackleton_crater.sdf
```

---

## 3. Interface Definitions (`astromesh_interfaces`)

### 3.1 `LandmarkObservation.msg`
```idl
# Topological visual/geometric landmark observation
uint16 schema_version
uint16 rover_id
uint32 local_index
uint64 descriptor_hash
geometry_msgs/Point local_coordinates
builtin_interfaces/Time stamp
```

### 3.2 `IbltSync.msg`
```idl
# 1 KB Invertible Bloom Lookup Table fingerprint for candidate discovery
uint16 sender_rover_id
uint16 target_rover_id
uint32 num_cells       # 32 cells (standard AstroMesh-X envelope)
uint32 num_probes      # 3 hash probes
uint32 protocol_id     # Version 1
uint8[] serialized_cells # 1024 bytes (32 cells x 32 bytes/cell)
builtin_interfaces/Time stamp
```

### 3.3 `CandidateProposal.msg`
```idl
# Proposed loop closure candidates based on peeled IBLT symmetric difference
uint16 sender_rover_id
uint16 target_rover_id
uint32 keyframe_index
LandmarkObservation[] candidate_observations
```

### 3.4 `CandidateResponse.msg`
```idl
# Outcome of Kabsch RANSAC metric verification and DSU edge promotion
uint16 sender_rover_id
uint16 target_rover_id
bool verified
uint32 inlier_count
float64 inlier_rmse
geometry_msgs/Pose2D relative_transform
uint32 sender_keyframe
uint32 target_keyframe
```

### 3.5 `BoundaryState.msg`
```idl
# 80-byte canonical CORD boundary state packet
uint16 protocol_version
uint16 sender_rover_id
uint16 target_rover_id
uint32 node_local_index
uint32 sequence_number
builtin_interfaces/Time stamp
geometry_msgs/Pose2D pose
geometry_msgs/Twist velocity
uint64 checksum
```

---

## 4. Node Wrapper Architecture (`astromesh_core::RoverSlamNode`)

The ROS 2 node encapsulates the pure C++ `DistributedReconciliationAgent` and `RoverNode`, mapping standard ROS 2 topic streams to the AstroMesh-X topological SLAM engine:

```mermaid
flowchart TD
    subgraph ROS 2 Ecosystem
        ODOM["/rover_X/odom (nav_msgs/Odometry)"]
        VISION["/rover_X/landmarks (LandmarkObservationArray)"]
        PUB_PG["/rover_X/pose_graph (visualization_msgs/MarkerArray)"]
        PUB_CLUSTERS["/rover_X/dsu_clusters (diagnostic_msgs/DiagnosticStatus)"]
    end

    subgraph AstroMesh-X ROS 2 Node
        NODE["RoverSlamNode (Lifecycle Node)"]
        AGENT["DistributedReconciliationAgent"]
        PG["PoseGraph & DSU"]
        IBLT["InvertibleBloomFilter (32 cells)"]
        RANSAC["GeometricVerifier2D (Kabsch)"]
    end

    subgraph Distributed Network
        UDP["Native UDP Socket / ROS 2 DDS Topic Bridge"]
        PEERS["Remote Rover Peer Nodes"]
    end

    ODOM -->|Keyframe delta| NODE
    VISION -->|Feature vectors| NODE
    NODE --> AGENT
    AGENT --> IBLT
    AGENT --> PG
    AGENT --> RANSAC
    AGENT <-->|DTN Frames (Sync, Proposal, Ack, Boundary)| UDP
    UDP <--> PEERS
    PG --> PUB_PG
    PG --> PUB_CLUSTERS
```

### Key Topics & Services
| Topic Name | Type | Direction | Description |
| :--- | :--- | :--- | :--- |
| `~/odom` | `nav_msgs/Odometry` | In | Odometry pose steps for keyframe admission |
| `~/landmarks` | `astromesh_interfaces/LandmarkObservationArray` | In | Visual/spatial landmark detections |
| `~/mesh_in` | `astromesh_interfaces/IbltSync` | In | Peer IBLT discovery messages |
| `~/mesh_out` | `astromesh_interfaces/IbltSync` | Out | Outbound IBLT discovery messages |
| `~/pose_graph_markers` | `visualization_msgs/MarkerArray` | Out | RViz2 keyframe nodes, odometry links, loop closures |
| `~/dsu_status` | `diagnostic_msgs/DiagnosticArray` | Out | Topological partition count and cluster members |

---

## 5. Gazebo Simulation Environment (`astromesh_gazebo`)

### 5.1 Lunar Terrain World (`lunar_shackleton_crater.sdf`)
- **Physics Engine:** DART / ODE with lunar gravity ($g = -1.62\,\text{m/s}^2$).
- **Regolith Model:** Terramechanics friction parameterization ($\mu_s = 0.8$, $\mu_k = 0.6$).
- **Illumination:** Low-angle horizontal collimated solar directional light with dynamic shadows modeling lunar polar crater illumination.
- **Elevation Mesh:** High-resolution digital elevation model (DEM) representing an $80\,\text{m} \times 80\,\text{m}$ lunar crater terrain.

### 5.2 Lunar Rover Model (`astromesh_rover.urdf.xacro`)
- **Kinematics:** 4-wheel differential-drive / rocker-bogie chassis ($0.8\,\text{m} \times 0.6\,\text{m}$, mass: $45.0\,\text{kg}$).
- **Actuation Plugin:** `libgazebo_ros_diff_drive.so` publishing odometry to `~/odom` and listening to `~/cmd_vel`.
- **Sensors:**
  - Forward-facing stereo camera pair publishing synchronized depth images.
  - Planar LIDAR scanner publishing to `~/scan` for obstruction mapping.
  - Simulated visual landmark extractor emitting detected feature descriptors with localized 2D Cartesian offsets.

---

## 6. Swarm Multi-Rover Launch Blueprint

```python
# multi_rover_swarm.launch.py (Reference Implementation)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    rovers = [
        {"id": 1, "x": "0.0", "y": "0.0", "yaw": "0.0", "port": 19401},
        {"id": 2, "x": "10.0", "y": "0.0", "yaw": "0.0", "port": 19402},
        {"id": 3, "x": "5.0", "y": "8.66", "yaw": "1.04", "port": 19403},
    ]

    actions = []
    for r in rovers:
        group = GroupAction([
            PushRosNamespace(f"rover_{r['id']}"),
            Node(
                package="astromesh_core",
                executable="rover_slam_node",
                name="slam_node",
                parameters=[{
                    "rover_id": r["id"],
                    "udp_port": r["port"],
                    "iblt_cells": 32,
                    "iblt_probes": 3,
                    "enable_dtn": True,
                    "ransac_inlier_dist": 0.50,
                }],
                output="screen"
            )
        ])
        actions.append(group)

    return LaunchDescription(actions)
```

---

## 7. Migration & Deployment Instructions

When transitioning to a Linux host (e.g., Ubuntu 22.04 LTS or a cloud VM with Docker):

1. **Prerequisites:**
   ```bash
   sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs
   ```
2. **Build Workspace:**
   ```bash
   mkdir -p ~/astromesh_ws/src
   cd ~/astromesh_ws
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
3. **Run Multi-Rover Simulation:**
   ```bash
   source install/setup.bash
   ros2 launch astromesh_gazebo multi_rover_swarm.launch.py
   ```
4. **Visualize Topological Map Convergence in RViz2:**
   ```bash
   rviz2 -d $(ros2 pkg prefix astromesh_gazebo)/share/astromesh_gazebo/rviz/swarm.rviz
   ```

