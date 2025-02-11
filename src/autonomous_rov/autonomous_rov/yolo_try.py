#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import torch
import numpy as np
import time
import os
from cv_bridge import CvBridge
# Forcer l'utilisation du plugin xcb pour Qt
os.environ["XDG_SESSION_TYPE"] = "xcb"
os.environ["QT_QPA_PLATFORM"] = "xcb"


def overlay_box(image, box, label, conf, color=(0, 255, 0)):
    """
    Dessine un rectangle et affiche l'étiquette avec la confiance sur l'image.
    """
    x1, y1, x2, y2 = [int(v) for v in box]
    cv2.rectangle(image, (x1, y1), (x2, y2), color, 2)
    cv2.putText(image, f"{label} {conf:.2f}", (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)


class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        # Déclaration du paramètre pour le chemin de l'image
        self.declare_parameter('image_path', '/home/projet_sysmer/Pictures/Screenshots/test.png')
        self.image_path = self.get_parameter('image_path').value
        self.get_logger().info(f"Chemin de l'image fourni : {self.image_path}")

        # Chargement du modèle YOLOv5 custom
        self.get_logger().info("Chargement du modèle YOLOv5...")
        self.model = torch.hub.load('/home/projet_sysmer/ros2_ws/src/yolov5',
                                    'custom',
                                    path='/home/projet_sysmer/ros2_ws/src/yolov5/runs/train/exp3/weights/best.pt',
                                    source='local')
        self.model.conf = 0.5  # Seuil de confiance
        self.get_logger().info("Modèle chargé avec succès.")

        # Lancer le traitement de l'image
        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format
        cv2.namedWindow("image")  # Create the window before setting the mouse callback
        self.process_image()

    def process_image(self):
        # Charger l'image depuis le chemin passé en paramètre
        image = cv2.imread(self.image_path)
        if image is None:
            self.get_logger().error(f"Erreur : Impossible de charger l'image à partir de {self.image_path}")
            return

        # Conversion en RGB pour YOLOv5
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # Mesurer le temps d'inférence
        start_time = time.time()
        results = self.model(image_rgb)
        end_time = time.time()
        inference_time = (end_time - start_time) * 1000  # en ms
        self.get_logger().info(f"Temps d'inférence : {inference_time:.2f} ms")

        # Récupérer les détections : [x1, y1, x2, y2, confiance, classe]
        detections = results.xyxy[0].cpu().numpy()
        self.get_logger().info("Détections :")
        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            self.get_logger().info(
                f"Classe: {int(cls)} ({self.model.names[int(cls)]}) - Confiance: {conf:.4f} - Coordonnées: {x1:.4f}, {y1:.4f}, {x2:.4f}, {y2:.4f}")
            overlay_box(image, [x1, y1, x2, y2], self.model.names[int(cls)], conf)

        # Afficher l'image annotée dans une fenêtre OpenCV
        cv2.imshow("Result", image)
        self.get_logger().info("Appuyez sur une touche pour fermer la fenêtre...")
        cv2.waitKey(2)
        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    # Ici, le traitement est lancé lors de l'initialisation, on peut quitter après.
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
