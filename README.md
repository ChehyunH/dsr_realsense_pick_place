# DSR RealSense Pick & Place

**Full perception-to-control pipeline on a Doosan E0509 collaborative robot.**  
Intel RealSense RGB-D + custom-trained YOLOv8 + ROS 2 → physical pick & place.

---

## Demo

[![Pick and Place Demo](https://img.youtube.com/vi/4IkEsHyye2E/maxresdefault.jpg)](https://youtu.be/4IkEsHyye2E)

> 📹 **[Watch on YouTube](https://youtu.be/4IkEsHyye2E)**

---

## Overview

This ROS 2 package integrates computer vision, coordinate transforms, and robot control into a single end-to-end system. A RealSense depth camera detects objects in 3D space, transforms their coordinates into the robot frame, and commands a Doosan E0509 arm to pick and place them autonomously.

**Key challenges solved:**
- TF calibration errors between `camera_color_optical_frame` and `base_link` — debugged systematically on real hardware
- Modbus RTU communication instability with the ROBOTIS RH-P12-Rn gripper — resolved with DRL-based serial control and automatic topic fallback
- Object yaw estimation using depth-band masking + PCA for gripper alignment to the object's long axis

---

## System Architecture

```
Intel RealSense D4xx
    │  RGB + Depth (aligned)
    ▼
[object_detector]  ──────────────────────────┐
  YOLOv8 detection (custom best.pt)           │
  MAD depth filtering                         │  /detected_objects (JSON)
  RealSense deproject                         │  /detection_debug_image
  TF transform (camera → base_link)           │
    │                                         ▼
    │  /selected_object_pose          [gui_node]
    │                                   Camera feed display
    ▼                                   Object selection buttons
[pick_place_node]                       State monitoring
  9-state state machine                        │
  Doosan service calls                         │ /selected_object_label
  Gripper control (Modbus RTU)  ◄──────────────┘
    │
    ▼
Doosan E0509 + ROBOTIS RH-P12-Rn Gripper
```

---

## State Machine

```
IDLE
 │  Move to home position
 ▼
DETECTING
 │  Receive /selected_object_pose + validate workspace
 ▼
PRE_PICK
 │  Move above object (pre_pick_z_offset = 0.12 m) + open gripper
 ▼
PICK
 │  Descend slowly (50 mm/s) to grasp height (pick_z_offset = 0.005 m) + close gripper
 ▼
LIFT
 │  Rise back to PRE_PICK height
 ▼
MOVE_TO_PLACE
 │  Move to above place position (pre_place_z_offset = 0.15 m)
 ▼
PLACE
 │  Descend to place_position + open gripper
 ▼
POST_PLACE
 │  Rise to safe height
 ▼
HOME → IDLE (next cycle)

Exception → ERROR (manual recovery required)
```

---

## Custom YOLOv8 Training

Rather than using the default COCO weights, we trained a custom YOLOv8 model on self-collected data for five target object classes:

| Class   | Training Strategy |
|---------|-------------------|
| cup     | Solo shots → mixed with other targets → mixed with non-targets + noise |
| pack    | Same 3-stage strategy |
| doll    | Same 3-stage strategy |
| pencil  | Same 3-stage strategy |
| tape    | Same 3-stage strategy |

**3-stage data collection:**
1. Each object photographed alone (basic appearance learning)
2. All five objects together (inter-class discrimination)
3. Mixed with unrelated objects (false positive suppression + noise augmentation)

Training used automatic augmentation: `mosaic`, `mixup`, `copy_paste`, rotation, flip, and scale variation.

> ⚠️ The trained weights file (`best.pt`) is excluded from this repository via `.gitignore`. Each team member must prepare their own weights file.

---

## Node Summary

| Node | Role |
|------|------|
| `object_detector` | Receives synchronised RGB + Depth from RealSense → YOLOv8 detection → MAD depth filtering → 3D coordinate transform → publish |
| `pick_place_node` | Receives target pose → executes pick & place via state machine |
| `gui_node` | Displays detection feed, object selection buttons, state monitoring |

---

## Package Structure

```
dsr_realsense_pick_place/
├── dsr_realsense_pick_place/
│   ├── object_detector.py      # RGB-D object detection node
│   ├── pick_place_node.py      # Pick & Place state machine node
│   └── gui_node.py             # PyQt5 GUI node
├── launch/
│   └── pick_place.launch.py    # Full system launch file
├── config/
│   └── pick_place_params.yaml  # Node parameters
├── requirements.txt
├── package.xml
└── setup.py
```

---

## Requirements

| Item | Version / Spec |
|------|----------------|
| OS | Ubuntu 22.04 LTS |
| ROS | ROS 2 Humble |
| Python | 3.10+ |
| Robot | Doosan E0509 (or virtual mode) |
| Camera | Intel RealSense D400 series (optional) |
| Gripper | ROBOTIS RH-P12-Rn (Modbus RTU over serial) |

---

## Installation

### 1. ROS 2 packages
```bash
sudo apt update
sudo apt install -y \
  ros-humble-cv-bridge \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-vision-msgs \
  python3-numpy \
  python3-opencv \
  python3-pyqt5
```

### 2. RealSense ROS 2 driver
```bash
sudo apt install -y ros-humble-realsense2-camera
```

### 3. Doosan ROS 2 packages
```bash
cd ~/ros2_ws/src
git clone <DOOSAN_REPOSITORY_URL> doosan-robot2
```
Required packages: `dsr_bringup2`, `dsr_msgs2`, `dsr_controller2`, `dsr_description2`, `dsr_moveit2`

### 4. Python dependencies
```bash
pip install ultralytics
# or
pip install -r ~/ros2_ws/src/dsr_realsense_pick_place/requirements.txt
```

### 5. Build
```bash
cd ~/ros2_ws
colcon build --packages-select dsr_realsense_pick_place
source install/setup.bash
```

---

## Running

### Virtual mode (no physical robot)
```bash
source ~/ros2_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 launch dsr_realsense_pick_place pick_place.launch.py mode:=virtual
```

### Real robot
```bash
source ~/ros2_ws/install/setup.bash
export QT_QPA_PLATFORM=xcb
ros2 launch dsr_realsense_pick_place pick_place.launch.py \
  mode:=real \
  host:=192.168.1.100
```

### Without RealSense (node connectivity test only)
```bash
ros2 launch dsr_realsense_pick_place pick_place.launch.py use_realsense:=false
```

---

## Key Configuration

### Camera TF calibration
The relative position between the camera and robot base is specified via launch arguments. Incorrect values will cause pick coordinates to be offset from the actual object position.

```bash
ros2 launch dsr_realsense_pick_place pick_place.launch.py \
  cam_tf_x:=0.450 cam_tf_y:=0.010 cam_tf_z:=0.620 \
  cam_tf_qx:=0.0 cam_tf_qy:=0.707 cam_tf_qz:=0.0 cam_tf_qw:=0.707
```

For precise calibration, the `easy_handeye2` package is recommended.

### YOLO settings (`config/pick_place_params.yaml`)
```yaml
object_detector:
  ros__parameters:
    use_yolo: true
    yolo_model: "best.pt"           # custom trained weights
    confidence_threshold: 0.5
    target_classes: ["cup", "pack", "doll", "pencil", "tape"]
```

### Gripper settings
```yaml
pick_place_node:
  ros__parameters:
    rh12_open_stroke: 700           # 0–700, 700 = fully open
    rh12_close_stroke: 0            # 0 = fully closed
    rh12_goal_current: 400          # gripping force
    rh12_allow_missing_service: true  # fallback to topic if service unavailable
```

---

## Debugging

```bash
# Monitor state machine
ros2 topic echo /pick_place_state

# Check pick target coordinates (robot base frame, metres)
ros2 topic echo /selected_object_pose

# View detection image
ros2 run rqt_image_view rqt_image_view /detection_debug_image

# Check TF transform
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

---

## Topics

### Published
| Topic | Type | Node | Description |
|-------|------|------|-------------|
| `/selected_object_pose` | `geometry_msgs/PoseStamped` | object_detector | Pick target coordinates |
| `/detected_objects` | `std_msgs/String` | object_detector | Detected objects (JSON) |
| `/detection_debug_image` | `sensor_msgs/Image` | object_detector | BBox overlay debug image |
| `/pick_place_state` | `std_msgs/String` | pick_place_node | Current state machine state |

### Subscribed
| Topic | Publisher | Subscriber |
|-------|-----------|------------|
| `/camera/camera/color/image_raw` | RealSense | object_detector |
| `/camera/camera/aligned_depth_to_color/image_raw` | RealSense | object_detector |
| `/selected_object_label` | gui_node | object_detector |
| `/selected_object_pose` | object_detector | pick_place_node |

---

## Common Issues

| Issue | Solution |
|-------|----------|
| GUI / RViz won't open | `export QT_QPA_PLATFORM=xcb` |
| TF transform failure (wrong pick position) | Check `cam_tf_*` values against actual camera position |
| YOLO weights not found | Pre-download: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"` |
| Gripper not moving | Set `rh12_allow_missing_service: true`; check `rh12_slave_id` |

---

## Team

4 contributors — developed as part of an AI & Robotics bootcamp at KG Kairos, Seoul.

---

## License

BSD-3-Clause — see [LICENSE](LICENSE) for details.
