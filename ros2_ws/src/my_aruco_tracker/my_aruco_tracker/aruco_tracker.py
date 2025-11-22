#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, LaserScan
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from cv_bridge import CvBridge
from tf2_ros import TransformBroadcaster

import cv2
import numpy as np
import math


class BarrelMultiTracker(Node):
    def __init__(self):
        super().__init__('barrel_multi_tracker')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        self.declare_parameter('min_area', 800.0)
        self.declare_parameter('uniqueness_distance', 0.6)

        self.declare_parameter('target_distance', 0.20)
        self.declare_parameter('approach_speed_max', 0.25)

        self.declare_parameter('angular_speed_search', 0.8)
        self.declare_parameter('angular_gain', 1.2)
        self.declare_parameter('linear_gain', 0.5)

        self.declare_parameter('avoid_distance', 0.35)
        self.declare_parameter('front_obs_angle_deg', 30.0)
        self.declare_parameter('laser_search_width_deg', 8.0)

        self.declare_parameter('hsv_lower', [5, 100, 100])
        self.declare_parameter('hsv_upper', [25, 255, 255])

        # Read parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.min_area = float(self.get_parameter('min_area').value)
        self.uniqueness_distance = float(self.get_parameter('uniqueness_distance').value)

        self.target_distance = float(self.get_parameter('target_distance').value)
        self.approach_speed_max = float(self.get_parameter('approach_speed_max').value)

        self.angular_speed_search = float(self.get_parameter('angular_speed_search').value)
        self.angular_gain = float(self.get_parameter('angular_gain').value)
        self.linear_gain = float(self.get_parameter('linear_gain').value)

        self.avoid_distance = float(self.get_parameter('avoid_distance').value)
        self.front_obs_angle_deg = float(self.get_parameter('front_obs_angle_deg').value)
        self.laser_search_width_deg = float(self.get_parameter('laser_search_width_deg').value)

        hsv_lower = self.get_parameter('hsv_lower').value
        hsv_upper = self.get_parameter('hsv_upper').value
        self.hsv_lower = np.array(hsv_lower, dtype=np.int32)
        self.hsv_upper = np.array(hsv_upper, dtype=np.int32)

        # State
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.last_scan = None

        self.tracked = []
        self.next_id = 1
        self.current_target = None

        # ROS interfaces
        self.tf_broadcaster = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, '/barrel_marker', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/barrel_pose', 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(Image, self.image_topic, self.image_cb, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_cb, 10)

        self.create_timer(0.08, self.control_loop)

        self.get_logger().info("BarrelMultiTracker started.")

    def camera_info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("Camera intrinsics loaded.")

    def scan_cb(self, msg: LaserScan):
        self.last_scan = msg

    def image_cb(self, msg: Image):
        if self.camera_matrix is None or self.last_scan is None:
            return

        img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, self.hsv_lower, self.hsv_upper)

        # clean mask
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)

        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            self.current_target = None
            return

        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)

        chosen = None
        for c in cnts:
            if cv2.contourArea(c) > self.min_area:
                chosen = c
                break

        if chosen is None:
            self.current_target = None
            return

        M = cv2.moments(chosen)
        if M['m00'] == 0:
            self.current_target = None
            return

        cx = int(M['m10'] / M['m00'])
        fx = self.camera_matrix[0, 0]
        cx_cam = self.camera_matrix[0, 2]
        bearing = math.atan2(cx - cx_cam, fx)

        scan = self.last_scan
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        n = len(scan.ranges)

        center_idx = int((bearing - angle_min) / angle_inc)
        half = int(math.radians(self.laser_search_width_deg) / angle_inc)

        idx0 = max(0, center_idx - half)
        idx1 = min(n - 1, center_idx + half)

        best_range = float('inf')
        best_idx = None

        for i in range(idx0, idx1 + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and r > scan.range_min and r < scan.range_max:
                if r < best_range:
                    best_range = r
                    best_idx = i

        if best_idx is None:
            self.current_target = None
            return

        angle_at = angle_min + best_idx * angle_inc
        dist = best_range

        x = dist * math.cos(angle_at)
        y = dist * math.sin(angle_at)

        assigned = None
        for tb in self.tracked:
            if math.hypot(tb['x'] - x, tb['y'] - y) < self.uniqueness_distance:
                tb['x'] = 0.9 * tb['x'] + 0.1 * x
                tb['y'] = 0.9 * tb['y'] + 0.1 * y
                assigned = tb
                break

        if assigned is None:
            # NEW BARREL
            bid = self.next_id
            self.next_id += 1

            tb = {'id': bid, 'x': x, 'y': y}
            self.tracked.append(tb)

            print(f"\n🔥 ARCUMARKER detected {bid} at x={x:.2f}, y={y:.2f}\n")

            self.publish_barrel_visuals(tb)

        self.current_target = min(self.tracked, key=lambda t: math.hypot(t['x'], t['y']))

    def publish_barrel_visuals(self, tb):
        # Pose
        pose = PoseStamped()
        pose.header.frame_id = "base_scan"
        pose.pose.position.x = tb['x']
        pose.pose.position.y = tb['y']
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

        # Cylinder Marker
        m = Marker()
        m.header = pose.header
        m.ns = "barrels"
        m.id = tb['id']
        m.type = Marker.CYLINDER
        m.action = Marker.ADD
        m.pose = pose.pose
        m.scale.x = 0.4
        m.scale.y = 0.4
        m.scale.z = 0.8

        color = ColorRGBA()
        color.r = 1.0
        color.g = 0.5
        color.b = 0.0
        color.a = 0.9
        m.color = color

        self.marker_pub.publish(m)

        # Text Marker
        t = Marker()
        t.header = pose.header
        t.ns = "barrel_text"
        t.id = tb['id'] + 1000
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose = pose.pose
        t.pose.position.z += 0.9
        t.scale.z = 0.25

        c2 = ColorRGBA()
        c2.r = 1.0
        c2.g = 1.0
        c2.b = 0.8
        c2.a = 1.0
        t.color = c2

        t.text = f"ARCUMARKER {tb['id']}"
        self.marker_pub.publish(t)

    def control_loop(self):
        # TF publish for each barrel
        for tb in self.tracked:
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = "base_scan"
            tf.child_frame_id = f"barrel_{tb['id']}"
            tf.transform.translation.x = tb['x']
            tf.transform.translation.y = tb['y']
            tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf)

        if self.current_target is None:
            cmd = Twist()
            cmd.angular.z = self.angular_speed_search
            self.cmd_pub.publish(cmd)
            return

        tx, ty = self.current_target['x'], self.current_target['y']
        dist = math.hypot(tx, ty)
        angle = math.atan2(ty, tx)

        # Obstacle avoidance
        if self._obstacle_front(self.avoid_distance):
            cmd = Twist()
            cmd.linear.x = -0.15   # back off
            self.cmd_pub.publish(cmd)
            return

        cmd = Twist()

        cmd.angular.z = -self.angular_gain * angle

        if dist > self.target_distance:
            cmd.linear.x = min(
                self.linear_gain * (dist - self.target_distance),
                self.approach_speed_max
            )
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    def _obstacle_front(self, threshold):
        if self.last_scan is None:
            return False

        scan = self.last_scan
        center = int((0 - scan.angle_min) / scan.angle_increment)
        half = int(math.radians(self.front_obs_angle_deg) / scan.angle_increment)

        i0 = max(0, center - half)
        i1 = min(len(scan.ranges) - 1, center + half)

        for i in range(i0, i1):
            if math.isfinite(scan.ranges[i]) and scan.ranges[i] < threshold:
                return True

        return False


def main(args=None):
    rclpy.init(args=args)
    node = BarrelMultiTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

