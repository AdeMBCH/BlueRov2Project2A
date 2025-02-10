import rclpy
import numpy as np
import traceback

from struct import pack, unpack
from sensor_msgs.msg import Joy, Imu,FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.srv import EndpointAdd
from duplicity.config import current_time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Int16, Float64, Empty, String
from mavros_msgs.msg import OverrideRCIn, Mavlink
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time


class VisualServoing(Node):
    def __init__(self):
        super().__init__("visual_servoing_node")
        self.get_logger().info("This node is named visual_servoing_node")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1)

        #Subscribers creation
        self.desired_point_sub = self.create_subscription(Float64MultiArray,'/bluerov2/desired_point', self.desired_point_callback,10)
        self.tracked_point_sub = self.create_subscription(Float64MultiArray,'/bluerov2/tracked_point',self.tracked_point_callback,10)
        self.sub_segment = self.create_subscription(Float64MultiArray,'tracked_segment',self.segment_callback,10)
        self.subping = self.create_subscription(Float64MultiArray, "ping1d/data", self.pingerCallback,qos_profile=qos_profile)

        self.get_logger().info("Subscriptions succesfully created")

        # Publishers
        self.corrected_vel = self.create_publisher(Twist, 'computed_error', 10)
        self.overrideRCIN_publisher = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.angle_in_degree_publisher = self.create_publisher(Twist, 'angle_degree', 10)
        self.depth_publisher = self.create_publisher(Float64, 'depth', 10)
        self.angular_velocity_publisher = self.create_publisher(Twist, 'angular_vel', 10)
        self.linear_velocity_publisher = self.create_publisher(Twist, 'linear_vel', 10)
        self.error_publisher = self.create_publisher(Float64MultiArray, 'visual_servoing/error', 10)
        self.cam_speed_publisher = self.create_publisher(Twist, 'visual_servoing/cam_speed', 10)
        self.robot_speed_publisher = self.create_publisher(Twist, 'visual_servoing/robot_speed', 10)
        self.get_logger().info("Publishers succesfully created")




        # Adding default values to __init__

        self.desired_point = None
        self.tracked_point = None
        self.has_tracked_point = False
        self.buoy_detected = False

        self.tolerance_error = 0.10
        self.Z = 1.0

        self.P = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 2.0  ])
        self.I = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0001])
        self.D = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.001])

        self.motor_max_val = 1900
        self.motor_min_val = 1100
        self.Correction_yaw = 1500
        self.Correction_depth = 1500
        self.homogeneous_transform = np.eye(4)
        self.thruttle = 1500
        self.THRESHOLD_WIDTH = 0.12

        self.previous_error = [0.0, 0.0]
        self.error_integral = [0.0, 0.0]
        self.error_derivative = [0.0, 0.0]

        self.state = "SEARCHING"
        self.last_known_position = None
        self.search_mode = "spiral"
        self.start_time = time.time()
        self.timeout_duration = 1.0

        self.pinger_confidence = 100
        self.pinger_distance = 10
        self.max_pinger_distance = 1.0

    def pingerCallback(self, msg):
        self.pinger_distance = msg.data[0]
        self.pinger_confidence = msg.data[1]
        self.get_logger().info("pinger_distance =" + str(self.pinger_distance))

    def update_state(self):
        if self.state == "SEARCHING":
            if self.tracked_point is not None:
                self.buoy_detected = True
                self.state = 'TRACKING'
                self.get_logger().info(f"La bouée vient d'être retrouvée...")
                return

            self.search_for_buoy()

        elif self.state == "TRACKING":
            current_time = time.time()
            self.compute_visual_servo()
            if self.last_tracked_time and (current_time - self.last_tracked_time > self.timeout_duration):
                self.state = 'LOST'
                self.get_logger().warn("Tracked point lost! Switching to LOST state.")
                self.tracked_point = None

        elif self.state == "LOST":
            self.reorient_to_last_known_position()

    def search_for_buoy(self):
        cmd_vel = Twist()
        elapsed_time = time.time() - self.start_time

        if self.search_mode == "spiral":
            cmd_vel.linear.x = min(0.2 + elapsed_time * 0.01, 0.5)  # Avancer de plus en plus vite
            cmd_vel.angular.z = 0.5 / (1 + elapsed_time * 0.1)  # Diminuer la rotation avec le temps

        elif self.search_mode == "eight":
            cmd_vel.linear.x = 0.2  # Avancer
            cmd_vel.angular.z = 0.5 * (-1) ** int(elapsed_time % 2)

        self.get_logger().info(f"Searching buoy around {self.last_known_position}")
        self.robot_speed_publisher.publish(cmd_vel)

    def reorient_to_last_known_position(self):
        if self.tracked_point is not None:
            self.get_logger().info("La bouée a été retrouvée, arrêt de la réorientation.")
            self.state = "TRACKING"
            return

        if self.last_known_position is None or time.time() - self.last_tracked_time > 10:
            self.state = "SEARCHING"
            return

        self.get_logger().info(f"Réorientation vers {self.last_known_position}")
        error = self.last_known_position - np.array([0, 0])  # On suppose [0,0] comme centre caméra
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.2 if abs(error[0]) > self.tolerance_error else 0.0
        cmd_vel.angular.z = 0.2 if abs(error[1]) > self.tolerance_error else 0.0
        self.robot_speed_publisher.publish(cmd_vel)


    def segment_callback(self, msg):
        x1, y1, x2, y2 = msg.data
        current_width = x2 - x1

        if current_width < self.THRESHOLD_WIDTH:
            move = True
        else:
            move = False
        self.forward_move(move)

    def forward_move(self, move):
        if move == True:
            self.thruttle = 1600
            self.get_logger().info("La bouée est trop petite → Le robot doit avancer.")
        else:
            self.thruttle = 1500
            self.get_logger().info("La bouée est proche → Le robot reste en place.")

    def desired_point_callback(self, msg):
        if len(msg.data) >= 2:
            self.desired_point = np.array(msg.data)
        else:
            self.get_logger().warn("Received invalid desired point data!")

    def tracked_point_callback(self, msg):
        self.last_tracked_time = time.time()
        if len(msg.data) >= 2:
            self.tracked_point = np.array(msg.data)

            if self.desired_point is None:
                return

            self.last_known_position = self.tracked_point

            if self.state == "SEARCHING":
                self.state = "TRACKING"

            self.compute_visual_servo()
        else:
            self.get_logger().warn("Received invalid tracked point data!")

    def check_buoy_visibility(self):
        if self.tracked_point is None:
            if self.state == "TRACKING":
                self.state = "LOST"
                self.reorient_to_last_known_position()

    def compute_error(self):
        if self.desired_point is None or self.tracked_point is None:
            return np.zeros(2), np.zeros(2), np.zeros(2)
        else:

            error = self.desired_point - self.tracked_point
            error[1] =0.0
            error_array_msg = Float64MultiArray()
            error_array_msg.data = error.tolist()
            self.error_publisher.publish(error_array_msg)


            self.error_integral += error

            error_derivative = error - self.previous_error

            self.previous_error = error

            return error, self.error_integral, error_derivative

    def compute_visual_servo(self):

        error, error_integral, error_derivative = self.compute_error()

        currentL = self.compute_interaction_matrix(self.tracked_point, self.Z)
        desiredL = self.compute_interaction_matrix(self.desired_point, self.Z)
        meanL = (currentL + desiredL) / 2.0

        try:
            L_pinv = np.linalg.pinv(meanL)
            cam_speed = (self.P * L_pinv.dot(error)
                        +self.I * L_pinv.dot(error_integral)
                        +self.D * L_pinv.dot(error_derivative))

            # self.get_logger().error(f"Error is {error}")
        except:
            self.get_logger().error("Singular interaction matrix, cannot compute pseudo-inverse")
            self.error_integral = np.zeros(2)
            return

        # Publish camera velocity
        cam_speed_msg = Twist()
        cam_speed_msg.linear.x = cam_speed[0]
        cam_speed_msg.linear.y = cam_speed[1]
        cam_speed_msg.linear.z = cam_speed[2]
        cam_speed_msg.angular.x = cam_speed[3]
        cam_speed_msg.angular.y = cam_speed[4]
        cam_speed_msg.angular.z = cam_speed[5]
        self.cam_speed_publisher.publish(cam_speed_msg)

        # Transforming camera velocity to robot velocity
        robot_speed = self.transform_velocity(cam_speed, self.homogeneous_transform)

        # Publishing robot velocity
        robot_speed_msg = Twist()
        robot_speed_msg.linear.x = robot_speed[0]
        robot_speed_msg.linear.y = robot_speed[1]
        robot_speed_msg.linear.z = robot_speed[2]
        robot_speed_msg.angular.x = robot_speed[3]
        robot_speed_msg.angular.y = robot_speed[4]
        robot_speed_msg.angular.z = robot_speed[5]
        self.robot_speed_publisher.publish(robot_speed_msg)

        # self.get_logger().info(
        #     f"Robot speed before publishing: Linear X={robot_speed[0]}, Y={robot_speed[1]}, Z={robot_speed[2]}, "
        #     f"Angular X={robot_speed[3]}, Y={robot_speed[4]}, Z={robot_speed[5]}")

        # Map robot velocity to control commands
        cmd_vel = Twist()
        cmd_vel.linear.z = robot_speed[2]
        cmd_vel.angular.x = robot_speed[3]
        cmd_vel.angular.y = robot_speed[4]
        cmd_vel.angular.z = robot_speed[5]

        # Sending controls to robot
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)

        # self.get_logger().info(f"Yaw speed: {cmd_vel.angular.z}")
        # self.get_logger().info(f"lateral_left_right speed: {-cmd_vel.linear.y}")

        # Send commands to robot
        self.setOverrideRCIN(1500, 1500, 1500,
                              yaw_left_right, 1500, 1500)

        self.get_logger().info(f' Published new command to RCIN :'
                               f' yaw : {yaw_left_right}, '   #need to check
                               f' thruttle : {self.thruttle}, '     #okay
                               f' lateral lr : {lateral_left_right}')    #okay

    def transform_velocity(self, cam_speed, H):
        # Transform camera velocity to robot velocity using homogeneous transform H
        # For simplicity, assuming camera and robot frames are aligned
        # In practice, H should be the transform from camera to robot frame
        R = H[0:3, 0:3]
        vrobot_linear = R @ cam_speed[0:3]
        vrobot_angular = R @ cam_speed[3:6]
        vrobot = np.hstack((vrobot_linear, vrobot_angular))
        return vrobot

    def compute_interaction_matrix(self, points, Z):
        # Compute the interaction matrix for the given points
        num_points = len(points) // 2
        L = []
        for i in range(num_points):
            x = points[2 * i]
            y = points[2 * i + 1]
            L_point = np.array([
                [-1 / Z,      0, x / Z,  x * y, -(1 + x ** 2),      y],
                [     0, -1 / Z, y / Z, 1 + y ** 2,     -x * y,     -x]
            ])
            L.append(L_point)
        L = np.vstack(L)
        return L

    def mapValueScalSat(self, value):
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100
        return int(pulse_width)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward, channel_lateral):
        msg_override = OverrideRCIn()
        msg_override.channels = [1500] * 8
        msg_override.channels[0] = channel_roll      # Roll
        msg_override.channels[1] = channel_pitch     # Pitch
        msg_override.channels[2] = channel_throttle  # Throttle
        msg_override.channels[3] = channel_yaw       # Yaw
        msg_override.channels[4] = channel_forward   # Forward
        msg_override.channels[5] = channel_lateral   # Lateral

        # if self.pinger_distance > self.max_pinger_distance and self.pinger_confidence > 95:
        #     msg_override.channels[0] = 1500  # Roll
        #     msg_override.channels[1] = 1500  # Pitch
        #     msg_override.channels[2] = 1500  # Throttle
        #     msg_override.channels[3] = 1600  # Yaw
        #     msg_override.channels[4] = 1500  # Forward
        #     msg_override.channels[5] = 1500  # Lateral

        self.overrideRCIN_publisher.publish(msg_override)


def main(args=None):
    rclpy.init(args=args)
    visual_servo_node = VisualServoing()

    try:
        while rclpy.ok():
            rclpy.spin_once(visual_servo_node, timeout_sec=0.1)
            visual_servo_node.update_state()
    except KeyboardInterrupt:
        visual_servo_node.get_logger().info("Arrêt du visual servoing.")
        visual_servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()