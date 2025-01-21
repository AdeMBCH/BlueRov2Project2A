# Autonomous ROV - Visual Servoing & Image Processing

This project implements a visual servoing system for an autonomous ROV using ROS 2 Iron. The project includes an object tracking algorithm and a visual control system to adjust the robot's movements based on tracked points in images.

## Project Structure

- **image_processing_tracker.py**: This node performs image processing, including object detection and point tracking. **Make sure the camera topic is correct. You can check it with the following command:**
  
```bash
  ros2 topic list
```

  **If you encounter issues with image display, ensure you correctly modify the image conversion code:**
  
```python
  try:
      # If using uncompressed images (sensor_msgs/Image)
      image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      # If using compressed images (sensor_msgs/CompressedImage)
      # image_np = self.bridge.compressed_imgmsg_to_cv2(msg)
```

- **visual_servoing_node.py**: This node implements the visual servoing system, computing the error between the tracked point and the desired point and adjusting the robot's position accordingly. **It controls the ROV's lateral movements, yaw, and forward motion using a proportional controller.**
- **launch**: Directory containing launch files to execute the nodes.

## Prerequisites

- **ROS 2 Iron** (compatible with newer ROS 2 versions)
- **OpenCV** for image processing
- **cv_bridge** for conversion between OpenCV and ROS messages

## Install Dependencies

Ensure all dependencies are installed before compiling:

```bash
cd ~/ros2_ws
colcon build --symlink-install
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
ros2 launch autonomous_rov image_process.launch.py
```

### 2. Launch the Visual Servoing Node

The `visual_servoing_node` receives tracking information and calculates motion commands for the robot based on the error.

```bash
ros2 launch autonomous_rov visual_servoing.launch.py
```

## Launch File Descriptions

The project includes two main launch files:

- **visual_servoing.launch.py**: Launches the visual servoing and image processing nodes.
- **image_processing.launch.py**: Allows specifying the camera topic and launches only the necessary nodes for image processing.

## Troubleshooting

### Nodes Are Not Connected

- Ensure that your nodes are correctly launched and that they communicate via the appropriate topics. You can check the topics with the command:

```bash
ros2 topic list
```

### Issues with Camera Images

- Make sure the camera is properly configured and that the corresponding topic is correctly published.
- Modify the image conversion according to the format used in your ROS 2 pipeline.

# YOLOv5 Integration with ROS2 for Buoy Detection

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
~/ros2_ws/src/autonomous_rov/yoloV5/
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
     train: /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/train
     val: /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/valid
     test: /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/test

     nc: 6
     names:
       - blue_ball
       - green-buoy
       - orange-buoy
       - red-buoy
       - water_target
       - yellow-buoy
```
---

## **Train YOLOv5**

1. Navigate to the YOLOv5 directory:
   
```bash
   cd ~/ros2_ws/src/yolov5
```

2. Run the training command:
   
```bash
   python3 train.py --img 640 --batch 16 --epochs 50 --data /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/data.yaml --weights yolov5s.pt
```

3. Monitor training:
   - Training results (including the best model weights) will be saved in:
     
```
runs/train/exp/weights/
```
   - The best model is saved as `best.pt`.

4. Optional: Use TensorBoard for real-time monitoring:
   
```bash
   tensorboard --logdir runs/train
```

---

## **Testing**

Once the training is complete, the generated `best.pt` model can be used directly in your ROS2 node for real-time buoy detection.

## **Acknowledgements**

- YOLOv5 by Ultralytics: [GitHub Repository](https://github.com/ultralytics/yolov5)
- ROS2 documentation: [ros.org](https://docs.ros.org)

