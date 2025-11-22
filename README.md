# Autonomous-Docking-Using-ArUco-Markers-

Short description
This repository contains a ROS 2 node that searches for orange barrels in a Gazebo world, treats each detected barrel as an “ARCUMARKER”, publishes their positions and TF frames to rviz2, visualizes them with markers and text, and drives the robot toward the nearest barrel while avoiding obstacles. It supports tracking multiple barrels and prints ARCUMARKER detected N once per new object.

Contents

scripts/ or my_aruco_tracker/ — Python node: barrel_multi_tracker.py (main)

package.xml, setup.py — ROS2 package files

README.md — this file

Example screenshot (uploaded): <img width="1740" height="877" alt="image" src="https://github.com/user-attachments/assets/3245ca96-fd54-4963-9548-74f8ce9d04c1" />


Screenshot (local path): <img width="1788" height="847" alt="image" src="https://github.com/user-attachments/assets/74237b72-9885-4674-a659-de81983548fb" />


Features

Continuous 360° searching when no target detected.

Barrel detection using camera color segmentation (HSV).

Distance/bearing fused from camera direction + LaserScan.

Unique ID assignment per barrel and persistent tracking.

TF frames barrel_1, barrel_2, ... published continuously.

RViz visualization: cylinder marker + text "ARCUMARKER N".

Simple proportional controller to align and approach nearest barrel.

Front-obstacle checking and safe backoff to avoid collisions.

Parameters exposed for easy tuning.

Prerequisites

Ubuntu (tested on Ubuntu 22.04)

ROS 2 Humble (or compatible) installed and sourced

OpenCV and cv_bridge for ROS 2

A Gazebo simulation that publishes:

/camera/image_raw (RGB)

/camera/camera_info

/scan (LaserScan)

tf2_ros and visualization_msgs available (ROS 2 packages)

Install dependencies (example):

sudo apt update
sudo apt install -y python3-opencv ros-humble-cv-bridge ros-humble-vision-msgs

(Replace humble with your ROS2 distro name if needed.)

Installation (in your ROS 2 workspace)

Clone or copy the package into your workspace src folder:

cd ~/ros2_ws/src
git clone  my_aruco_tracker

or if you already have the package files, just place them under ~/ros2_ws/src/my_aruco_tracker.

Build the workspace:

cd ~/ros2_ws
colcon build --packages-select my_aruco_tracker

Source the workspace:

source install/setup.bash
Run (basic)

Launch your Gazebo world (make sure it publishes /camera/image_raw, /camera/camera_info, /scan).

Run the node:

ros2 run my_aruco_tracker aruco_tracker

(or run the script directly: python3 scripts/arucu tracker.py after sourcing workspace)

Open RViz2:

rviz2

Set Fixed Frame to your robot base frame (e.g., base_link or base_scan).

Add displays:

TF (to see barrel_N frames)

Marker — topic: /barrel_marker

Pose — topic: /barrel_pose

LaserScan — topic: /scan (optional for debugging)

Image — topic: /camera/image_raw (optional)

Key Parameters (default values)

image_topic : /camera/image_raw

camera_info_topic : /camera/camera_info

scan_topic : /scan

cmd_vel_topic : /cmd_vel

min_area : 800.0 — minimum contour area to accept detection

uniqueness_distance : 0.6 m — how close a detection must be to consider "same barrel"

target_distance : 0.20 m — desired stopping distance from barrel

approach_speed_max : 0.25 m/s

angular_speed_search : 0.8 rad/s

angular_gain : 1.2

linear_gain : 0.5

avoid_distance : 0.35 m — threshold for front obstacle detection

laser_search_width_deg : 8.0 degrees — search window around camera bearing

hsv_lower : [5, 100, 100] — HSV lower bound (orange)

hsv_upper : [25, 255, 255] — HSV upper bound (orange)

You can set parameters at runtime or change defaults in the node.

Example: run with a changed target distance

ros2 run my_aruco_tracker aruco_tracker --ros-args -p target_distance:=0.25
How it works (brief)

Detect color in camera image using HSV mask; select largest orange contour as barrel candidate.

Compute direction (left/right) using camera intrinsics (principal point & focal length).

Match direction to LaserScan readings in a small angular window; choose the nearest valid range to compute position.

Convert to robot coordinates (x,y) and publish:

TF frame barrel_N

/barrel_pose (PoseStamped)

/barrel_marker (visual cylinder + text)

If barrel is far and aligned, robot moves toward it; always check front LaserScan and back off if obstacle detected.

If a barrel's position is within uniqueness_distance of a tracked barrel, update that tracked barrel instead of creating a new one.

Troubleshooting

No detections:

Confirm /camera/image_raw and /camera/camera_info are being published.

Tune HSV parameters (lighting in Gazebo can change colors).

Lower min_area if barrel appears small.

Laser scan mismatch:

If camera is mounted offset from laser, add an angle offset (modify code to add/subtract yaw).

Increase laser_search_width_deg if needed.

Robot doesn't move:

Check /cmd_vel topic is used by your robot base controller.

Ensure no other node is publishing to /cmd_vel and overriding commands.

Markers jitter:

Increase smoothing or increase repeated detection threshold.

Suggested improvements

Use a proper ArUco texture on the barrel model and use OpenCV ArUco detection.

Use depth camera data for direct 3D pose estimation (better Z).

Replace proportional simple controller with local planner or PID for smoother motion.

Implement Kalman filter for robust multi-target tracking.

Save tracked coordinates to a file for post-processing.

License

Add your chosen license (MIT, Apache, etc.) in a LICENSE file.

Acknowledgements

This work combines simple CV (color detection) and sensor fusion (camera + LIDAR) to provide robust detection and navigation in simulation. Useful ROS2 packages: cv_bridge, tf2_ros, visualization_msgs.
