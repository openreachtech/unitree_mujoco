# mid360_ros2_bridge

TCP → ROS 2 bridges for `simulate_python/` (Process A, the sim/DDS process). Needs
`rclpy`, which on this project's Python 3.12 side isn't available (CycloneDDS's Python
bindings don't build against Python 3.14's annotation changes) - run these under a
Python 3.14 (or whatever `rclpy` install you have) venv instead, as separate processes
from `simulate_python/unitree_mujoco.py`.

- `mid360_tcp_to_ros2.py` - republishes `simulate_python/mid360_lidar.py`'s LiDAR/IMU
  frames as `/livox/lidar`, `/livox/imu`, and `/clock`.
- `lowstate_tcp_to_ros2.py` - republishes `simulate_python/lowstate_ros2_relay.py`'s
  LowState frames (the same fields `unitree_sdk2py_bridge.py` already sends over DDS as
  `rt/lowstate`) as `/lowstate`, `unitree_go/msg/LowState` - the exact type the real
  Go2 publishes, so nodes written against the real robot (e.g. go2_self_filter's
  `lowstate_to_joint_state_node`) work unchanged against this sim. Needs the
  `unitree_go` ROS 2 package (built from a ros2_ws overlay) sourced in addition to the
  base ROS 2 install.

## Setup

```bash
uv venv --python 3.14 .venv
source .venv/bin/activate
uv pip install numpy
# rclpy/sensor_msgs/unitree_go/etc. come from the sourced ROS 2 install(s), not pip:
source /opt/ros/<distro>/setup.bash
source <path-to-ros2_ws>/install/setup.bash  # only needed for lowstate_tcp_to_ros2.py
```

## Run

```bash
source /opt/ros/<distro>/setup.bash
source <path-to-ros2_ws>/install/setup.bash  # only needed for lowstate_tcp_to_ros2.py
source .venv/bin/activate
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # large PointCloud2 was unreliable on Fast-DDS
python mid360_tcp_to_ros2.py       # /livox/lidar, /livox/imu, /clock
python lowstate_tcp_to_ros2.py     # /lowstate
```
