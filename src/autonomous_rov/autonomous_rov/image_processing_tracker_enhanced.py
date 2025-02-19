#!/usr/bin/env python3

import csv
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Image, CompressedImage   # need to import CompressedImage to fit the topic used by bags
from mavros_msgs.msg import CameraImageCaptured
import numpy as np
import cv2
from cv_bridge import CvBridge
import time



# Hue: 170 (ranges from 0 to 179 in OpenCV)
# Saturation: 155 (ranges from 0 to 255)
# Value: 160 (ranges from 0 to 255)


# Clicks for feedback
set_desired_point = False
get_hsv = False
mouseX, mouseY = 0, 0
hsv_value = [0, 0, 0]

# camera parameters
u0 = 320
v0 = 240
lx = 455
ly = 455
kud = 0.00683
kdu = -0.01424



# convert a pixel coordinate to meters using defaut calibration parameters
def convertOnePoint2meter(pt):
    global u0, v0, lx, ly
    return (float(pt[0]) - u0) / lx, (float(pt[1]) - v0) / ly


# convert a list of pixels coordinates to meters using defaut calibration parameters
def convertListPoint2meter(points):
    global u0, v0, lx, ly

    if (np.shape(points)[0] > 1):
        n = int(np.shape(points)[0] / 2)
        point_reshaped = (np.array(points).reshape(n, 2))
        point_meter = []
        for pt in point_reshaped:
            pt_meter = convertOnePoint2meter(pt)
            point_meter.append(pt_meter)
        point_meter = np.array(point_meter).reshape(-1)
        return point_meter


# helper for OpenCV functions

def overlay_points(image, pt, r, g, b, text="", scale=1, offsetx=5, offsety=5):
    cv2.circle(image, (int(pt[0]), int(pt[1])), int(4 * scale + 1), (b, g, r), -1)
    position = (int(pt[0]) + offsetx, int(pt[1]) + offsety)
    cv2.putText(image, text, position, cv2.FONT_HERSHEY_SIMPLEX, scale, (b, g, r, 255), 1)


def click_detect(event, x, y, flags, param):
    global get_hsv, set_desired_point, mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDOWN:
        get_hsv = True
        mouseX, mouseY = x, y
    if event == cv2.EVENT_RBUTTONDOWN:
        set_desired_point = True
        mouseX, mouseY = x, y


class ImageProcessingNode(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.pub_tracked_point = self.create_publisher(Float64MultiArray, 'tracked_point', 10)
        self.pub_desired_point = self.create_publisher(Float64MultiArray, 'desired_point', 10)
        self.pub_tracked_segment = self.create_publisher(Float64MultiArray, 'tracked_segment', 10)

        self.subscription = self.create_subscription(
            Image,
            '/bluerov2/camera/image',
            self.cameracallback,
            10)

        self.last_tracked_point = None  # en mètres
        self.last_time = None
        self.max_speed = 1.5  # seuil de vitesse en m/s (à ajuster)

        self.bridge = CvBridge()  # CvBridge for converting ROS images to OpenCV format

        cv2.namedWindow("image")  # Create the window before setting the mouse callback
        cv2.setMouseCallback("image", click_detect)

        self.get_logger().info('Image processing node started')

    def cameracallback(self, msg):
        begin_time = time.time()

        global get_hsv, set_desired_point, mouseX, mouseY, hsv_value

        try:
            image_np = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Pour les images compressées, utilisez :
            # image_np = self.bridge.compressed_imgmsg_to_cv2(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la conversion de l'image: {e}")
            return

        if image_np is None:
            self.get_logger().error("L'image convertie est None")
            return

        image_height, image_width, image_channels = image_np.shape

        desired_point = [image_width / 2, image_height / 2]
        if set_desired_point:
            desired_point = [mouseX, mouseY]
            set_desired_point = False

        hsv = cv2.cvtColor(image_np, cv2.COLOR_BGR2HSV)
        if get_hsv:
            hsv_value = hsv[mouseY, mouseX]
            self.get_logger().info(f"HSV Value at ({mouseX},{mouseY}):{hsv_value}")
            get_hsv = False

        tolerance = np.array([10, 50, 50])
        lower_bound = np.clip(np.array(hsv_value) - tolerance, 0, 255)
        upper_bound = np.clip(np.array(hsv_value) + tolerance, 0, 255)

        mask = cv2.inRange(hsv, lower_bound, upper_bound)
        non_zero_pixels = cv2.findNonZero(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        tracked_point = None
        segment_width = None
        segment_y = None
        left_point = None
        right_point = None

        # Calcul du point suivi
        if non_zero_pixels is not None and len(non_zero_pixels) > 0:
            mean_point = np.mean(non_zero_pixels, axis=0, dtype=int)
            tracked_point = list(mean_point[0])

        # Calcul du segment uniquement si un point suivi est détecté et des contours sont présents
        if tracked_point is not None and contours:
            cx, cy = tracked_point
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            left_point = (x, cy)
            right_point = (x + w, cy)

            # Conversion en mètres
            left_point_meter = convertOnePoint2meter(left_point)
            right_point_meter = convertOnePoint2meter(right_point)
            dx = left_point_meter[0] - right_point_meter[0]
            dy = left_point_meter[1] - right_point_meter[1]
            current_width = np.sqrt(dx ** 2 + dy ** 2)
            # Validation du segment sur la largeur

            if current_width >= 0.07:
                segment_width = current_width

        current_time = time.time()
        valid_update = True
        if tracked_point is not None:
            current_point_meter = convertOnePoint2meter(tracked_point)
            if self.last_tracked_point is not None and self.last_time is not None:
                dt = current_time - self.last_time
                if dt > 0:
                    displacement = np.linalg.norm(np.array(current_point_meter) - np.array(self.last_tracked_point))
                    speed = displacement / dt
                    if speed > self.max_speed:
                        self.get_logger().info(f"Vitesse trop élevée: {speed:.2f} m/s, mise à jour ignorée")
                        valid_update = False
                        current_point_meter = self.last_tracked_point
                    else:
                        self.last_tracked_point = current_point_meter
                        self.last_time = current_time
                else:
                    self.last_tracked_point = current_point_meter
                    self.last_time = current_time
            else:
                self.last_tracked_point = current_point_meter
                self.last_time = current_time

        # Publier et afficher uniquement si le point et le segment sont valides ET la vitesse est acceptable
        if tracked_point is not None and segment_width is not None and valid_update:
            current_point_meter = list(current_point_meter)
            current_point_meter[1] = cy
            overlay_points(image_np, tracked_point, 0, 255, 0, 'current tracked buoy')
            cv2.line(image_np, (x, cy), (x + w, cy), (0, 0, 0), 5)
            current_point_msg = Float64MultiArray(data=current_point_meter)
            self.pub_tracked_point.publish(current_point_msg)
            segment_msg = Float64MultiArray(data=[segment_width])
            self.pub_tracked_segment.publish(segment_msg)

        # Toujours afficher le point désiré
        overlay_points(image_np, desired_point, 255, 0, 0, 'desired point')
        desired_point_meter = convertOnePoint2meter(desired_point)
        desired_point_msg = Float64MultiArray(data=desired_point_meter)
        self.pub_desired_point.publish(desired_point_msg)

        cv2.imshow("image", image_np)
        cv2.waitKey(2)
        end_time = time.time()
        elapsed_time = end_time - begin_time
        self.get_logger().warn(f"Computation with HSV took {elapsed_time*1000:.2f} milliseconds")


def main(args=None):
    rclpy.init(args=args)
    node = ImageProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
