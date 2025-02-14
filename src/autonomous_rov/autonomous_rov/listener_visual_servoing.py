#!/usr/bin/env python

import rclpy
import time
import traceback
import numpy as np
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from struct import pack, unpack
from std_msgs.msg import Int16, Float64, Empty, Float64MultiArray, String
from sensor_msgs.msg import Joy, Imu, FluidPressure, LaserScan
from mavros_msgs.srv import CommandLong, SetMode, StreamRate
from mavros_msgs.msg import OverrideRCIn, Mavlink
from mavros_msgs.srv import EndpointAdd
from geometry_msgs.msg import Twist


class MyVisualServoingNode(Node):
    def __init__(self):
        super().__init__("listenerMIR")
        self.get_logger().info("This node is named VisualServoingNode")

        self.ns = self.get_namespace()
        self.get_logger().info("namespace =" + self.ns)

        self.corrected_vel = self.create_publisher(Twist, 'computed_error', 10)
        self.overrideRCIN_publisher = self.create_publisher(OverrideRCIn, "rc/override", 10)
        self.pub_angle_degre = self.create_publisher(Twist, 'angle_degree', 10)
        self.depth_publisher = self.create_publisher(Float64, 'depth', 10)
        self.angular_velocity_publisher = self.create_publisher(Twist, 'angular_vel', 10)
        self.linear_velocity_publisher = self.create_publisher(Twist, 'linear_vel', 10)
        self.error_publisher = self.create_publisher(Float64MultiArray, 'visual_servoing/error', 10)
        self.cam_speed_publisher = self.create_publisher(Twist, 'visual_servoing/cam_speed', 10)
        self.robot_speed_publisher = self.create_publisher(Twist, 'visual_servoing/robot_speed', 10)
        self.get_logger().info("Publishers succesfully created")

        self.get_logger().info("ask router to create endpoint to enable mavlink/from publication.")
        # self.addEndPoint()

        self.armDisarm(False)  # Not automatically disarmed at startup
        rate = 25  # 25 Hz
        self.setStreamRate(rate)
        # self.manageStabilize(False)

        ###### SUBSCRIBERS #######
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subjoy = self.create_subscription(Joy, "joy", self.joyCallback, qos_profile=qos_profile)
        self.subcmdvel = self.create_subscription(Twist, "cmd_vel", self.velCallback, qos_profile=qos_profile)
        self.subimu = self.create_subscription(Imu, "imu/data", self.OdoCallback, qos_profile=qos_profile)

        self.subrel_alt = self.create_subscription(Float64, "global_position/rel_alt", self.RelAltCallback,
                                                   qos_profile=qos_profile)

        self.desired_point_sub = self.create_subscription(Float64MultiArray, '/bluerov2/desired_point',
                                                          self.desired_point_callback, 10)
        self.tracked_point_sub = self.create_subscription(Float64MultiArray, '/bluerov2/tracked_point',
                                                          self.tracked_point_callback, 10)
        self.sub_segment = self.create_subscription(Float64MultiArray, 'tracked_segment', self.segment_callback, 10)
        self.subping = self.create_subscription(Float64MultiArray, "ping1d/data", self.pingerCallback,
                                                qos_profile=qos_profile)
        self.get_logger().info("Subscriptions done.")

        # variables
        self.set_mode = [0] * 2
        self.set_mode[0] = True  # Mode manual
        self.set_mode[1] = False  # Mode automatic

        # Conditions
        self.init_a0 = True
        self.init_p0 = True
        self.arming = False
        self.move = False

        self.angle_roll_ajoyCallback0 = 0.0
        self.angle_pitch_a0 = 0.0
        self.angle_yaw_a0 = 0.0
        self.depth_wrt_startup = 0
        self.depth_p0 = 0

        self.pinger_confidence = 0
        self.pinger_distance = 0

        self.Vmax_mot = 1900
        self.Vmin_mot = 1100

        # corrections for control
        self.Correction_yaw = 1500
        self.Correction_depth = 1500

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

        self.error_integral = np.zeros(2)
        self.previous_error = np.zeros(2)
        self.error_derivative = np.zeros(2)

        self.state = "SEARCHING"
        self.last_known_position = None
        self.search_mode = "spiral"
        self.start_time = time.time()
        self.timeout_duration = 1.0

        self.pinger_confidence = 100
        self.pinger_distance = 10
        self.max_pinger_distance = 1.0

        ### values for cam and light control

        self.cam_rc_value = 1500 # from 1100 to 1900, neutral position is at 1500
        self.light_rc_value = 1100 # from 1100 to 1900, light off is 1100

        ##### timer for auto mode #####
        timer_period = 0.10  # 50 msec - 20 Hz
        self.auto_timer = self.create_timer(timer_period, self.auto_mode_callback)

    def auto_mode_callback(self):
        if self.set_mode[1]:
            self.update_state()

    def segment_callback(self, msg):
        if self.set_mode[1]:
            x1, y1, x2, y2 = msg.data
            current_width = x2 - x1

            if current_width < self.THRESHOLD_WIDTH:
                self.move = True
            else:
                self.move = False
            self.forward_move()
        else:
            return

    def forward_move(self):
        if self.move == True:
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

        else:
            self.get_logger().warn("Received invalid tracked point data!")

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
        light_rc = self.light_rc_value
        cam_rc = self.cam_rc_value
        self.setOverrideRCIN(1500, 1500, 1500,
                              yaw_left_right, 1500, 1500, light_rc, cam_rc)

        self.get_logger().info(f' Published new command to RCIN :'
                               f' yaw : {yaw_left_right}, '   #need to check
                               f' thruttle : {1500}, '     #okay
                               f' lateral lr : {1500}')    #okay

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

    def armDisarm(self, armed):
        # This functions sends a long command service with 400 code to arm or disarm motors
        if (armed):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info("arming requested, wait_for_service, timeout, result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 1.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            self.get_logger().info("just before call_async")
            resp = cli.call_async(req)
            self.get_logger().info("just after call_async")
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Arming Succeeded")
        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(CommandLong, 'cmd/command')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "disarming requested, wait_for_service, (False if timeout) result :" + str(result))
            req = CommandLong.Request()
            req.broadcast = False
            req.command = 400
            req.confirmation = 0
            req.param1 = 0.0
            req.param2 = 0.0
            req.param3 = 0.0
            req.param4 = 0.0
            req.param5 = 0.0
            req.param6 = 0.0
            req.param7 = 0.0
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("Disarming Succeeded")

    def manageStabilize(self, stabilized):
        # This functions sends a SetMode command service to stabilize or reset
        if (stabilized):
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            cli = self.create_client(SetMode, 'set_mode')
            result = False
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "stabilized mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "0"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to STABILIZE Succeeded")

        else:
            traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
            result = False
            cli = self.create_client(SetMode, 'set_mode')
            while not result:
                result = cli.wait_for_service(timeout_sec=4.0)
                self.get_logger().info(
                    "manual mode requested, wait_for_service, (False if timeout) result :" + str(result))
            req = SetMode.Request()
            req.base_mode = 0
            req.custom_mode = "19"
            resp = cli.call_async(req)
            # rclpy.spin_until_future_complete(self, resp)
            self.get_logger().info("set mode to MANUAL Succeeded")

    def setStreamRate(self, rate):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(StreamRate, 'set_stream_rate')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info("stream rate requested, wait_for_service, (False if timeout) result :" + str(result))

        req = StreamRate.Request()
        req.stream_id = 0
        req.message_rate = rate
        req.on_off = True
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("set stream rate Succeeded")

    def addEndPoint(self):
        traceback_logger = rclpy.logging.get_logger('node_class_traceback_logger')
        cli = self.create_client(EndpointAdd, 'mavros_router/add_endpoint')
        result = False
        while not result:
            result = cli.wait_for_service(timeout_sec=4.0)
            self.get_logger().info(
                "add endpoint requesRelAltCallbackted, wait_for_service, (False if timeout) result :" + str(result))

        req = EndpointAdd.Request()
        req.url = "udp://@localhost"
        req.type = 1  # TYPE_GCS
        resp = cli.call_async(req)
        rclpy.spin_until_future_complete(self, resp)
        self.get_logger().info("add endpoint rate Succeeded")

    def joyCallback(self, data):
        # Joystick buttons
        btn_arm = data.buttons[7]  # Start button
        btn_disarm = data.buttons[6]  # Back button
        btn_automatic_mode = data.buttons[2]  # X button
        btn_manual_mode = data.buttons[0]  # A button
        btn_light_up = data.buttons[4]
        btn_light_down = data.buttons[5]
        btn_cam_tilt_up = data.buttons[1]
        btn_cam_tilt_down = data.buttons[3]

        # Disarming when Back button is pressed
        if btn_cam_tilt_up == 1 and btn_cam_tilt_down == 0:
            self.cam_rc_value+=100
            if self.light_rc_value>1900:
                self.light_rc_value = 1900
        elif btn_cam_tilt_up ==0 and btn_cam_tilt_down ==1:
            self.cam_rc_value-=100
            if self.light_rc_value < 1100:
                self.light_rc_value = 1100

        if btn_light_up == 1 and btn_light_down ==0:
            self.light_rc_value+=100
            if self.light_rc_value > 1900:
                self.light_rc_value = 1900
        elif btn_light_up == 0 and btn_light_down == 1:
            self.light_rc_value-=100
            if self.light_rc_value < 1100:
                self.light_rc_value = 1100

        if (btn_disarm == 1 and self.arming == True):
            self.arming = False
            self.armDisarm(self.arming)

        # Arming when Start button is pressed
        if (btn_arm == 1 and self.arming == False):
            self.arming = True
            self.armDisarm(self.arming)

        # Switch manual, auto anset_moded correction mode
        if (btn_manual_mode and not self.set_mode[0]):
            self.set_mode[0] = True
            self.set_mode[1] = False
            self.get_logger().info("Mode manual")
        if (btn_automatic_mode and not self.set_mode[1]):
            self.set_mode[0] = False
            self.set_mode[1] = True


            self.get_logger().info("Mode automatic")

    def velCallback(self, cmd_vel):
        # Only continue if manual_mode is enabled
        if (self.set_mode[1]):
            return
        else:
            self.get_logger().info("Sending...")

        # Extract cmd_vel message
        roll_left_right = self.mapValueScalSat(cmd_vel.angular.x)
        yaw_left_right = self.mapValueScalSat(-cmd_vel.angular.z)
        ascend_descend = self.mapValueScalSat(cmd_vel.linear.z)
        forward_reverse = self.mapValueScalSat(cmd_vel.linear.x)
        lateral_left_right = self.mapValueScalSat(-cmd_vel.linear.y)
        pitch_left_right = self.mapValueScalSat(cmd_vel.angular.y)

        light_rc = self.light_rc_value
        cam_rc = self.cam_rc_value

        self.setOverrideRCIN(pitch_left_right, roll_left_right, ascend_descend, yaw_left_right, forward_reverse,
                             lateral_left_right, light_rc,cam_rc)

    def setOverrideRCIN(self, channel_pitch, channel_roll, channel_throttle, channel_yaw, channel_forward,
                        channel_lateral,ligth_rc, cam_rc):
        msg_override = OverrideRCIn()
        msg_override.channels = [1500] * 18
        msg_override.channels[0] = np.uint(channel_roll)  # Roll
        msg_override.channels[1] = np.uint(channel_pitch)  # Pitch
        msg_override.channels[2] = np.uint(channel_throttle)  # Throttle
        msg_override.channels[3] = np.uint(channel_yaw)  # Yaw
        msg_override.channels[4] = np.uint(channel_forward)  # Forward
        msg_override.channels[5] = np.uint(channel_lateral)  # Lateral
        msg_override.channels[6] = 1500
        msg_override.channels[7] = 1500
        msg_override.channels[8] = ligth_rc # light
        msg_override.channels[9] = cam_rc  # camera
        msg_override.channels[10] = 1500
        msg_override.channels[11] = 1500
        msg_override.channels[12] = 1500
        msg_override.channels[13] = 1500
        msg_override.channels[14] = 1500
        msg_override.channels[15] = 1500
        msg_override.channels[16] = 1500
        msg_override.channels[17] = 1500

        # if self.pinger_distance > self.max_pinger_distance and self.pinger_confidence > 95:
        #     msg_override.channels[0] = 1500  # Roll
        #     msg_override.channels[1] = 1500  # Pitch
        #     msg_override.channels[2] = 1500  # Throttle
        #     msg_override.channels[3] = 1600  # Yaw
        #     msg_override.channels[4] = 1500  # Forward
        #     msg_override.channels[5] = 1500  # Lateral

        self.overrideRCIN_publisher.publish(msg_override)

    def mapValueScalSat(self, value):
        # Correction_Vel and joy between -1 et 1
        # scaling for publishing with setOverrideRCIN values between 1100 and 1900
        # neutral point is 1500
        pulse_width = value * 400 + 1500

        # Saturation
        if pulse_width > 1900:
            pulse_width = 1900
        if pulse_width < 1100:
            pulse_width = 1100

        return int(pulse_width)

    def OdoCallback(self, data):
        orientation = data.orientation
        angular_velocity = data.angular_velocity

        # extraction of roll, pitch, yaw angles
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        sinp = 2.0 * (w * y - z * x)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        angle_roll = np.arctan2(sinr_cosp, cosr_cosp)
        angle_pitch = np.arcsin(sinp)
        angle_yaw = np.arctan2(siny_cosp, cosy_cosp)

        if (self.init_a0):
            # at 1st execution, init
            self.angle_roll_a0 = angle_roll
            self.angle_pitch_a0 = angle_pitch
            self.angle_yaw_a0 = angle_yaw
            self.init_a0 = False

        angle_wrt_startup = [0] * 3
        angle_wrt_startup[0] = ((angle_roll - self.angle_roll_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[1] = ((angle_pitch - self.angle_pitch_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi
        angle_wrt_startup[2] = ((angle_yaw - self.angle_yaw_a0 + 3.0 * math.pi) % (
                    2.0 * math.pi) - math.pi) * 180 / math.pi

        angle = Twist()
        angle.angular.x = angle_wrt_startup[0]
        angle.angular.y = angle_wrt_startup[1]
        angle.angular.z = angle_wrt_startup[2]

        self.pub_angle_degre.publish(angle)

        # Extraction of angular velocity
        p = angular_velocity.x
        q = angular_velocity.y
        r = angular_velocity.z

        vel = Twist()
        vel.angular.x = p
        vel.angular.y = q
        vel.angular.z = r
        self.angular_velocity_publisher.publish(vel)

        # Only continue if manual_mode is disabled
        if (self.set_mode[0]):
            return

        # Send PWM commands to motors
        # yaw command to be adapted using sensor feedback
        self.Correction_yaw = 1500

    def RelAltCallback(self, data):
        if (self.init_p0):
            # 1st execution, init
            self.depth_p0 = data
            self.init_p0 = False
        # setup depth servo control here
        # ...

        # update Correction_depth
        Correction_depth = 1500
        self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer

    def DvlCallback(self, data):
        u = data.velocity.x  # Linear surge velocity
        v = data.velocity.y  # Linear sway velocity
        w = data.velocity.z  # Linear heave velocity
        Vel = Twist()
        Vel.linear.x = u
        Vel.linear.y = v
        Vel.linear.z = w
        self.linear_velocity_publisher.publish(Vel)

    # works but at 4hz compared with 25 Hz for global_position/rel_alt !
    # /uas1/mavlink_source runs at more than 500 Hz !
    # try with pyvmavlink using an udp connection, to see if gain in hz
    def mavlink_callback(self, data):
        # Check if message id is valid (I'm using SCALED_PRESSURE2)
        if data.msgid == 137:
            # self.get_logger().info("=> In mavlink_callback, msgid 137 SCALED_PRESSURE2, Package: " + str(data))
            # Transform the payload in a python string
            p = pack("QQ", *data.payload64)
            # Transform the string in valid values
            # https://docs.python.org/2/library/struct.html
            time_boot_ms, water_press_abs, press_diff, temperature = unpack("Iffhxx", p)

            # a priori, in hPa (hectoPascal)
            # self.get_logger().info("water_press_abs=" + str(water_press_abs))

            rho = 1000.0  # 1025.0 for sea water
            g = 9.80665

            # Only continue if correction mode is activated
            # if (self.set_mode[0] or self.set_mode[1]):
            #	return

            pressure = water_press_abs * 100.0

            if (self.init_p0):
                # 1st execution, init
                self.depth_p0 = (pressure - 101100) / (rho * g)
                self.init_p0 = False

            self.depth_wrt_startup = (pressure - 101100) / (rho * g) - self.depth_p0
            msg = Float64()
            msg.data = self.depth_wrt_startup
            self.depth_publisher.publish(msg)

            # setup depth servo control here
            # ...

            # update Correction_depth

            Correction_depth = 1500
            self.Correction_depth = int(Correction_depth)
        # Send PWM commands to motors in timer

    def pingerCallback(self, data):
        self.pinger_distance = data.data[0]
        self.pinger_confidence = data.data[1]

    # self.get_logger().info("pinger_distance =" + str(self.pinger_distance))

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


def main(args=None):
    rclpy.init(args=args)
    visual_servo_node = MyVisualServoingNode()

    try:
        while rclpy.ok():
            rclpy.spin_once(visual_servo_node, timeout_sec=0.1)
    except KeyboardInterrupt:
        visual_servo_node.get_logger().info("Arrêt du visual servoing.")
        visual_servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()