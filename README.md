# Autonomous ROV - Visual Servoing & Image Processing

This project implements a visual servoing system for an autonomous ROV using ROS 2 Iron. The project includes an object tracking algorithm and a visual control system to adjust the robot's movements based on tracked points in images.

## Prerequisites

- **ROS 2 Iron** (compatible with newer ROS 2 versions)
- **OpenCV** for image processing
- **cv_bridge** for conversion between OpenCV and ROS messages
- **numpy**
- **mavros_msgs**
- **geometry_msgs**
- **std_msgs**

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

## Launching the Nodes

### 1. Launch the Image Processing Node

The `image_processing_tracker` node is responsible for receiving camera images and processing them to detect and track an object.

```bash
ros2 launch autonomous_rov run_video.launch.py
```

### 2. State Machine 
The `state_machine` integrates the logics from `visual_servoing_node` into a finite state machine to get a full control of the ROV whatever the situation is.

```bash
ros2 launch autonomous_rov run_state_machine.launch
```

## Project Structure : image_processing_tracker.py

### Explanation of the HSV Script

The HSV script is designed to track an object using a color-based detection approach. Its workflow includes the following steps:

- **Conversion to HSV Space:**  
  The captured image (received as a ROS message) is first converted to an OpenCV format, then transformed from BGR to HSV. This transformation makes it easier to isolate specific colors.

- **Color Range Filtering:**  
  Using a defined HSV value (often obtained by clicking on the image) along with a set tolerance, a mask is created. This mask retains only those pixels whose colors fall within the desired range.

- **Tracked Point Detection:**  
  The script identifies the filtered pixels from the mask and computes their average to determine the center of the tracked object.

- **Segment Detection:**  
  By analyzing the contours generated from the mask, the script identifies the largest contour, computes a bounding box, and deduces the extreme points (left and right). These points are then converted into meters using camera calibration parameters, allowing an estimation of the object’s width (segment).

- **Data Publishing:**  
  The tracked point, the segment (represented by its width), and a desired point (defaulted to the center of the image or defined via a mouse click) are published on ROS topics. This data is used by other parts of the system, such as the visual control module.

---

### Improvements Made to the HSV Script

Several enhancements have been implemented to make HSV-based tracking more robust and accurate:

- **Conditional Display and Publishing:**  
  Overlays (annotations on the OpenCV image) and the publication of ROS messages occur only if both the tracked point and the segment are successfully detected. If either is missing, no overlay is drawn, preventing the display of erroneous information.

- **Vertical Alignment of the Tracked Point:**  
  The tracked point is now vertically aligned with the center of the segment. In other words, its vertical coordinate is adjusted to match that of the segment, ensuring visual consistency.

- **Segment Width Filtering:**  
  A check is performed to ensure that the segment’s width (calculated in meters) exceeds a minimum threshold (0.07 m). Segments failing to meet this threshold are rejected, reducing false detections or tracking of objects that are too small.

- **Tracked Point Velocity Control:**  
  The script calculates the movement speed of the tracked point between successive frames. If this speed exceeds a predefined threshold (e.g., 1 m/s), the new measurement is flagged as aberrant and ignored (or the last valid point is reused). This control helps prevent abrupt fluctuations caused by noise or detection errors.

These improvements enhance the reliability of HSV-based tracking, ensuring optimal integration with the autonomous ROV’s visual servoing system.


  **If you encounter issues with image display, ensure you correctly modify the image conversion code:**
  
```python
  try:
      # If using uncompressed images (sensor_msgs/Image)
      image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      # If using compressed images (sensor_msgs/CompressedImage)
      # image_np = self.bridge.compressed_imgmsg_to_cv2(msg)
```
# ImageProcessingWithYolo Node

This ROS2 node performs real-time image processing and object detection using a custom YOLOv5 model. It subscribes to a camera image topic, processes incoming frames to detect objects (e.g., buoys), and publishes tracking information such as the current detected object's position, the desired point, and a segment representing the object's width.

## Main Features

- **YOLOv5 Object Detection**  
  The node loads a custom YOLOv5 model via PyTorch and performs inference on incoming images. Detection results (bounding boxes, confidence scores, and class labels) are drawn on the image to provide visual feedback.

- **Real-Time Image Processing**  
  Using CvBridge, the node converts ROS image messages into OpenCV images, applies YOLO detection, and displays the annotated frames in an OpenCV window titled "Result". It also logs the inference time to monitor performance.

- **Coordinate Conversion**  
  The node includes a helper function to convert pixel coordinates to metric units based on predefined camera calibration parameters. This conversion is applied to both the detected object's position and the desired point.

## Publishers

- **tracked_point** (`Float64MultiArray`)  
  Publishes the metric coordinates of the current tracked object (e.g., the buoy) based on the center of the detection bounding box.

- **desired_point** (`Float64MultiArray`)  
  Publishes the desired tracking point in metric coordinates. This point is either the image center by default or set via a right-click by the user.

- **tracked_segment** (`Float64MultiArray`)  
  Publishes a segment defined by the left and right midpoints of the detected bounding box (converted to metric units), representing the object's width.

## Processing Workflow

1. **Image Reception**  
   The node subscribes to the `/bluerov2/camera/image` topic to receive raw image frames.

2. **Image Conversion**  
   ROS image messages are converted to OpenCV BGR images using CvBridge.

3. **Desired Point Selection**  
   The desired point is initially set to the image center but can be updated via a right-click, which triggers the mouse callback.

4. **Object Detection**  
   The image is converted from BGR to RGB and passed to the YOLOv5 model for inference. Detected objects are annotated with bounding boxes and labels.

5. **Segment and Point Calculation**  
   - The left and right midpoints of the first detected bounding box are calculated and drawn as a line.  
   - These points are converted from pixel coordinates to meters and published as the tracked segment.  
   - The center of the bounding box is determined and published as the current tracked point.

6. **Data Publishing and Display**  
   The node publishes the tracked segment, tracked point, and desired point (all in metric units) to their respective topics. The annotated image is then displayed in the "Result" window.

7. **Performance Logging**  
   The inference time is calculated and logged to help evaluate the node's real-time performance.

## Dependencies

- **ROS2**  
- **OpenCV**  
- **CvBridge**  
- **PyTorch** (for YOLOv5 inference)  
- **YOLOv5** (custom model loaded from a local repository)

This node is designed for robust real-time object detection and tracking in applications such as buoy tracking for underwater robotics.


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

