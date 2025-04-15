# Autonomous ROV - Visual Servoing

This project implements a visual servoing system for an autonomous ROV using ROS 2 Iron. The project includes an object tracking algorithm and a visual control system to adjust the robot's movements based on tracked points in images.

## Prerequisites

- **ROS 2 Iron** (compatible with newer ROS 2 versions)
- **OpenCV** for image processing
- **cv_bridge** for conversion between OpenCV and ROS messages
- **numpy**
- **mavros_msgs**
- **geometry_msgs**
- **std_msgs**
- **Glade GTK**

## Install Dependencies

Ensure all dependencies are installed before compiling:

```bash
cd ~/ros2_ws
colcon build 
```

## Source the Workspace

Before launching the nodes, source your ROS 2 workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

## Launching the project

### 1. Launching with the GUI

You can Launch the projet by launching the GUI first :

```bash
ros2 launch autonomous_rov run_gui.launch
```
And clinking on the "Launch" button.

### 2. Relaunching the state machine with the GUI

You can Relaunch the state machine by clicking on the "Relaunch" button **after killing the corresponding terminal first**

### 3. Launching with terminal commands

To launch the project you can also use terminal commands by yourself following the order :

```
ros2 launch autonomous_rov run_mavros.launch
```
```
ros2 launch autonomous_rov run_gamepad.launch
```
```
ros2 launch autonomous_rov run_visual_servo.launch
```
```
ros2 launch autonomous_rov run_listener_sysmer.launch
```
# MyVisualServoingNode

This ROS2 node implements a visual servoing logic for a BlueROV, integrating various data sources (IMU, depth sensors, joystick, vision) and directly commanding the actuators using messages of type `OverrideRCIn`. It allows controlling the robot's movement, the camera tilt, and the light intensity based on visual tracking (e.g., for buoy detection and tracking).

## Main Features

- **Visual Servoing Calculation**  
  The node computes the error between a desired point and the tracked point (extracted from visual processing) and, using an interaction matrix and its pseudo-inverse, derives the velocity commands for the camera. These velocities are then transformed into commands for the robot via a homogeneous transformation.  
  PID gains (P, I, D) are used to adjust the control behavior.

- **Joystick Control**  
  The node integrates input from a game controller (via the `/joy` topic) to:  
  - Arm/disarm the motors (Start and Back buttons).  
  - Switch between manual and automatic modes (A and X buttons).  
  - Adjust the camera tilt (buttons used to increase/decrease the `cam_rc_value`).  
  - Control the lighting (buttons associated with increasing/decreasing `light_rc_value`).

- **State Management and State Machine**  
  An internal state machine manages the robot's behavioral strategy during visual tracking:  
  - **SEARCHING**: The robot searches for the buoy using a predefined movement strategy (spiral or figure eight).  
  - **TRACKING**: Once the buoy is detected, tracking mode is activated to follow the point of interest, with continuous recalculation of the visual correction.  
  - **LOST**: If the tracked point is no longer detected (after a certain delay), the robot switches to LOST mode and reorients towards the last known position before potentially returning to SEARCHING mode.  
  Transitions between these states are managed in the `update_state()` method, which controls the switch based on sensor data and visual tracking.

- **Integration of Sensors and Services**  
  The node subscribes to various topics to retrieve:  
  - Velocity commands (`cmd_vel`),  
  - IMU data (`imu/data`) to compute angles (roll, pitch, yaw),  
  - Relative depth (`global_position/rel_alt`),  
  - Data for desired and tracked points (`/bluerov2/desired_point` and `/bluerov2/tracked_point`),  
  - Sonar data (ping1d).  
  Service calls (for example, to arm/disarm via `cmd/command`, to switch modes via `set_mode`, or to set the stream rate via `set_stream_rate`) are also implemented.

## Architecture and Components

### Publishers

- **computed_error** (`Twist`): Publishes the calculated error for visual servoing.
- **rc/override** (`OverrideRCIn`): Sends PWM commands to the robot's actuators.
- **angle_degree** (`Twist`): Publishes angles (roll, pitch, yaw) expressed in degrees.
- **depth** (`Float64`): Publishes the relative depth with respect to the startup point.
- **angular_vel** and **linear_vel** (`Twist`): Publish angular and linear velocities.
- **visual_servoing/error** (`Float64MultiArray`): Sends the error vector used in the control.
- **visual_servoing/cam_speed** (`Twist`): Publishes the calculated camera speed.
- **visual_servoing/robot_speed** (`Twist`): Publishes the robot speed obtained after transformation.

### Subscribers

- **joy** (`Joy`): Receives joystick commands to arm/disarm, switch modes, and adjust camera/light.
- **cmd_vel** (`Twist`): Receives velocity commands in manual mode.
- **imu/data** (`Imu`): Processes orientation and angular velocity data.
- **global_position/rel_alt** (`Float64`): Receives relative depth data.
- **/bluerov2/desired_point** and **/bluerov2/tracked_point** (`Float64MultiArray`): Track the point of interest (e.g., a buoy) and calculate the visual servoing error.
- **tracked_segment** (`Float64MultiArray`): Used to determine if the buoy is sufficiently large (close) to halt forward movement.
- **ping1d/data** (`Float64MultiArray`): Receives sonar (pinger) data.

### Main Methods

- **compute_visual_servo**  
  Computes the error between the desired point and the tracked point, uses the interaction matrix and its pseudo-inverse to determine the camera's velocity commands, and then transforms these velocities into commands for the robot. These commands are sent via the `rc/override` topic.

- **joyCallback**  
  Manages joystick inputs to:  
  - Adjust the PWM values for the camera and lighting.  
  - Arm/disarm the robot.  
  - Switch between modes (manual or automatic).

- **velCallback**  
  Receives `cmd_vel` commands in manual mode and converts them into PWM commands for the robot.

- **OdoCallback**  
  Processes IMU data to calculate orientation angles and angular velocity, then publishes this information on the `angle_degree` and `angular_vel` topics.

- **Services and Utilities**  
  - `armDisarm`: Arms/disarms the robot via a MAVLink long command.  
  - `manageStabilize`: Switches between stabilized and manual modes.  
  - `setStreamRate`: Sets the MAVLink message stream rate.  
  - `mapValueScalSat`: Transforms a value between -1 and 1 into a PWM value (between 1100 and 1900).

---

# State Machine - Buoy Detection and Tracking

This module implements a state machine for detecting, recognizing, and tracking a buoy. The states are designed to handle different operational phases with robust transitions.

## **Main States**

### 1. **INIT**
- **Purpose**: Initial state before detection.
- **Transitions**:
  - If a point is detected (`tracked_point` is not null) → switch to `RECON`.
- **Actions**:
  - Set `buoy_detected` flag.
  - Record start time (`start_recon_time`).

### 2. **RECON**
- **Purpose**: Buoy recognition via camera control and visual verification.
- **Transitions**:
  - **Signal loss** → `REPOSITIONING` (if timeout occurs).
  - **Recognition complete** → `TRACKING` (after `recon_time`).
  - **Verification phase** → `CHECKING` (every 25% of total time).
- **Actions**:
  - Adjust camera tilt (`control_camera_tilt`).
  - Perform visual verification (`visual_circle`).

### 3. **CHECKING**
- **Purpose**: Obstacle verification during recognition.
- **Transitions**:
  - After `checking_time` → `REPOSITIONING`.
- **Actions**:
  - Check left/right sides (`check_side`) based on obstacle detection.
  - Increment iteration counter (`check_iteration`).

### 4. **REPOSITIONING**
- **Purpose**: Repositioning after signal loss or verification.
- **Transitions**:
  - If the point is reacquired → `RECON`.
- **Actions**:
  - Alternate left/right turns (`turn_side`) based on the last movement.

### 5. **SEARCHING**
- **Purpose**: Active buoy search.
- **Transitions**:
  - If the buoy is detected → `TRACKING`.
- **Actions**:
  - Initiate search protocol (`search_for_buoy`).

### 6. **TRACKING**
- **Purpose**: Continuous buoy tracking.
- **Transitions**:
  - **Signal loss** → `LOST` (if timeout occurs).
- **Actions**:
  - Adjust camera tilt (`control_camera_tilt`).
  - Perform visual servo control (`compute_visual_servo`).

### 7. **LOST**
- **Purpose**: Recovery after prolonged signal loss.
- **Actions**:
  - Reorient to the last known position (`reorient_to_last_known_position`).

---

## **Transition Logic**
- **Timeout**: Managed via `last_tracked_time` and `timeout_duration`.
- **Timing**: Timestamps (`time.time()`) for critical phases.
- **Error Handling**: Fallback to `REPOSITIONING` on local failures.

## **Logging**
- Informational messages (`INFO`) for standard transitions.
- Warnings (`WARN`) for critical events (e.g., signal loss).

> **Note**: Time parameters (`recon_time`, `checking_time`, etc.) are configurable to adapt to environmental conditions.

# Depth Control System Overview

This module implements a comprehensive depth control system for underwater vehicles, combining smooth trajectory planning, robust feedback control, and sensor data filtering to maintain precise depth regulation.

## Cubic Trajectory Generation

The system generates a smooth cubic polynomial trajectory to transition the vehicle's depth from an initial value to a desired final depth over a specified time interval. This approach ensures continuous position and velocity profiles, minimizing mechanical stress and abrupt movements during depth changes.

## PID Control with Buoyancy Compensation

A PID controller regulates the vertical position by computing the control force needed to reach the desired depth. The controller includes:

- **Proportional term** to correct the current depth error.
- **Integral term** to eliminate steady-state error, with anti-windup logic that resets the integral when the error and velocity are sufficiently small.
- **Derivative term** based on the difference between desired and actual vertical velocities, providing damping.
- **Buoyancy compensation** as a constant offset to counteract the vehicle's inherent floatability, improving control accuracy.

## Alpha-Beta Filter for Velocity Estimation

To estimate the vertical velocity (heave) from noisy depth measurements, the system uses an alpha-beta filter. This filter predicts the current depth based on previous estimates and updates the velocity estimate by blending the prediction with the new measurement. The filter balances responsiveness and noise rejection without the complexity of a full Kalman filter.

## Force to PWM Conversion

The computed control force (in kgf) is converted into a PWM signal to command the vertical thrusters. The conversion applies different scaling factors depending on the force direction (upward or downward) and clamps the PWM values within safe limits to protect the motors from damage.

## Integration and Real-Time Control

The depth control loop runs within a sensor callback function that:

- Receives the current depth measurement.
- Calculates the elapsed time since the last update.
- Generates the desired depth and velocity from the cubic trajectory.
- Estimates the current vertical velocity using the alpha-beta filter.
- Computes the control force via the PID controller.
- Converts the force to a PWM command.
- Sends the PWM command to the thrusters.

This real-time integration ensures smooth, stable, and responsive depth control, adapting continuously to sensor feedback and environmental conditions.

# Troubleshooting

### Nodes Are Not Connected

- Ensure that your nodes are correctly launched and that they communicate via the appropriate topics. You can check the topics with the command:

```bash
ros2 topic list
```

### Issues with Camera Images

- Make sure the camera is properly configured and that the corresponding topic is correctly published.
- Modify the image conversion according to the format used in your ROS 2 pipeline.

# YOLOv5Nano Integration with ROS2 for Buoy Detection

## **Prerequisites**

1. **Install Python dependencies**:
   
```bash
   pip install torch torchvision matplotlib numpy opencv-python
```

2. **Clone YOLOv5 Repository**:
   
```bash
   git clone https://github.com/ultralytics/yolov5.git
   cd yolov5
   pip install -r requirements.txt
```

---

## **Dataset Preparation**

1. **Organize your dataset**:
   - Create directories for training and validation data:
     
```bash
     mkdir -p ~/ros2_ws/src/autonomous_rov/yoloV5/train
     mkdir -p ~/ros2_ws/src/autonomous_rov/yoloV5/valid
```

   - Place your images and YOLO-format annotations (.txt files) into the corresponding directories:
     
```
~/ros2_ws/src/autonomous_rov/yoloV5/data/images/
     ├── train/
     │   ├── image1.jpg
     │   ├── image1.txt
     │   └── ...
     ├── valid/
     │   ├── image2.jpg
     │   ├── image2.txt
     │   └── ...
```

2. **Prepare the data.yaml file**:
   - Create a file named `data.yaml` in the `yoloV5` folder with the following content:
     
```yaml
     train: /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/data/images/train
     val: /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/data/images/valid


   names:
     0: buoy
```
The argument "names" represents the output class predicted by the CNN. Here we only detect buoys.

---

## **Train YOLOv5**

1. Navigate to the YOLOv5 directory:
   
```bash
   cd ~/ros2_ws/src/yolov5
```

2. Run the training command:
   
```bash
   python3 train.py --img 640 --batch 16 --epochs 50 --data /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/data/data.yaml --weights yolov5n.pt
```
We use YoloV5Nano for this case. It is fast and accurate enough for our work. If your tasks is more sophisticated, upgrade the model to a more powerful version.
3. Monitor training:
   - Training results (including the best model weights) will be saved in:
     
```
runs/train/exp/weights/
```
   - The best model is saved as `best.pt`. This is the one we will be using because there is an early stopping implemented in the training. It means that the learning will automatically stop after detecting a non positive progression or a overfitting case.

4. Optional: Use TensorBoard for real-time monitoring:
   
```bash
   tensorboard --logdir runs/train
```

---

## **Testing**

Once the training is complete, the generated `best.pt` model can be used directly in your ROS2 node for real-time buoy detection.
```
python3 detect.py --img /path/to/your/img
```
## **Acknowledgements**

- YOLOv5 by Ultralytics: [GitHub Repository](https://github.com/ultralytics/yolov5)
- ROS2 documentation: [ros.org](https://docs.ros.org)

