# mid360_ros2_bridge

TCP → ROS 2 bridge for `simulate_python/mid360_lidar.py` (Process A). Republishes the
MID-360 LiDAR/IMU frames it receives over TCP as `/livox/lidar`, `/livox/imu`, and
`/clock`. Needs `rclpy`, which on this project's Python 3.12 side isn't available
(CycloneDDS's Python bindings don't build against Python 3.14's annotation changes) -
run this under a Python 3.14 (or whatever `rclpy` install you have) venv instead, as a
separate process from `simulate_python/unitree_mujoco.py`.

## Setup

```bash
uv venv --python 3.14 .venv
source .venv/bin/activate
uv pip install numpy
# rclpy/sensor_msgs/etc. come from the sourced ROS 2 install, not pip:
source /opt/ros/<distro>/setup.bash
```

## Run

```bash
source /opt/ros/<distro>/setup.bash
source .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # large PointCloud2 was unreliable on Fast-DDS
python mid360_tcp_to_ros2.py
```
