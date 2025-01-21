# Autonomous ROV - Visual Servoing & Image Processing

Ce projet implémente un système de commande visuelle pour un ROV autonome en utilisant ROS 2 Iron. Le projet inclut un algorithme de suivi d'objet et une commande visuelle pour ajuster les mouvements du robot en fonction des points suivis dans les images.

## Structure du projet

- image_processing_tracker.py : Ce nœud effectue le traitement d'images, y compris la détection d'objets et le suivi des points. **Assurez-vous que le topic de la caméra est correct. Vous pouvez le vérifier avec la commande suivante :**
  

bash
ros2 topic list


  **Si vous rencontrez des problèmes avec l'affichage des images, assurez-vous de modifier correctement le code de conversion des images :**
  

python
  try:
      # Si vous utilisez des images non compressées (sensor_msgs/Image)
      image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

      # Si vous utilisez des images compressées (sensor_msgs/CompressedImage)
      # image_np = self.bridge.compressed_imgmsg_to_cv2(msg)


- visual_servoing_node.py : Ce nœud implémente la commande visuelle, calculant l'erreur entre le point suivi et le point désiré, et ajustant la position du robot en conséquence. **Il contrôle les mouvements latéraux, le lacet (yaw) et l'avance (forward) du ROV à l'aide d'un correcteur proportionnel.**
- launch : Dossier contenant des fichiers de lancement pour exécuter les nœuds.

## Prérequis

- **ROS 2 Iron** (compatible avec les versions ROS 2 plus récentes)
- **OpenCV** pour le traitement d'image
- **cv_bridge** pour la conversion entre OpenCV et les messages ROS

## Installer les dépendances

Assurez-vous que toutes les dépendances sont installées avant de compiler :

bash
cd ~/ros2_ws
colcon build --symlink-install



## Source l'espace de travail

Avant de lancer les nœuds, sourcez votre espace de travail ROS 2 :

bash
source ~/ros2_ws/install/setup.bash



## Lancer les Nœuds

### 1. Lancer le nœud de traitement d'image

Le nœud image_processing_tracker est responsable de la réception des images de la caméra et du traitement pour détecter et suivre un objet.

bash
ros2 launch autonomous_rov image_process.launch.py



### 2. Lancer le nœud de commande visuelle

Le nœud visual_servoing_node reçoit les informations du point suivi et calcule les commandes de mouvement pour le robot en fonction de l'erreur.

bash
ros2 launch autonomous_rov visual_servoing.launch.py



## Description des fichiers de lancement

Le projet inclut deux fichiers de lancement principaux :

- **visual_servoing.launch.py** : Lance les nœuds de commande visuelle et de traitement d'image.
- **image_processing.launch.py** : Permet de spécifier le topic de la caméra et lance uniquement les nœuds nécessaires pour le traitement d'image.

## Dépannage

### Les nœuds ne sont pas connectés

- Assurez-vous que vos nœuds sont correctement lancés et qu'ils communiquent entre eux via les topics appropriés. Vous pouvez vérifier les topics avec la commande ros2 topic list.

### Problèmes avec les images de la caméra

- Assurez-vous que la caméra est bien configurée et que le topic correspondant est correctement publié.
- Modifiez la conversion des images selon le format utilisé dans votre pipeline ROS 2.

# YOLOv5 Integration with ROS2 for Buoy Detection
## **Prerequisites**

1. **Install Python dependencies**:
   

bash
   pip install torch torchvision matplotlib numpy opencv-python



2. **Clone YOLOv5 Repository**:
   

bash
   git clone https://github.com/ultralytics/yolov5.git
   cd yolov5
   pip install -r requirements.txt



---

## **Dataset Preparation**

1. **Organize your dataset**:
   - Create directories for training and validation data:
     

bash
     mkdir -p ~/ros2_ws/src/autonomous_rov/yoloV5/train
     mkdir -p ~/ros2_ws/src/autonomous_rov/yoloV5/valid


   - Place your images and YOLO-format annotations (.txt files) into the corresponding directories:
     

~/ros2_ws/src/autonomous_rov/yoloV8/
     ├── train/
     │   ├── image1.jpg
     │   ├── image1.txt
     │   └── ...
     ├── valid/
     │   ├── image2.jpg
     │   ├── image2.txt
     │   └── ...



2. **Prepare the data.yaml file**:
   - Create a file named data.yaml in the yoloV8 folder with the following content:
     

yaml
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



---

## **Train YOLOv5**

1. Navigate to the YOLOv5 directory:
   

bash
   cd ~/ros2_ws/src/yolov5



2. Run the training command:
   

bash
   python3 train.py --img 640 --batch 16 --epochs 50 --data /home/projet_sysmer/ros2_ws/src/autonomous_rov/yoloV5/data.yaml --weights yolov5s.pt



3. Monitor training:
   - Training results (including the best model weights) will be saved in:
     

runs/train/exp/weights/


   - The best model is saved as best.pt.

4. Optional: Use TensorBoard for real-time monitoring:
   

bash
   tensorboard --logdir runs/train



---

## **Testing**

Once the training is complete, the generated best.pt model can be used directly in your ROS2 node for real-time buoy detection.

## **Acknowledgements**

- YOLOv5 by Ultralytics: [GitHub Repository](https://github.com/ultralytics/yolov5)
- ROS2 documentation: [ros.org](https://docs.ros.org)
