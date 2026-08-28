"""MID-360 (MuJoCo-LiDAR) ray casting + IMU sampling for the Go2 simulate_python process.

Runs inside the same process/interpreter as the physics simulation (shares
mj_model/mj_data directly, no copying). Point cloud and IMU samples are sent
over a local TCP connection to a separate process that republishes them as
ROS 2 topics (see mid360_ros2_bridge/mid360_tcp_to_ros2.py). This process
(python3.12, unitree_sdk2py + cyclonedds 0.10.5, matching the CycloneDDS
version go2_ctrl links) intentionally does not link rclpy: CycloneDDS'
Python bindings do not build against Python 3.14's changed annotation
semantics, and rclpy on this machine is only available for the system
Python 3.14. Keeping DDS (for go2_ctrl) and ROS 2 (for
visualization/heightmap tooling) in separate processes sidesteps that.

Wire format (TCP, length-prefixed frames):
    1 byte  type: b'L' (lidar) or b'I' (imu)
    4 bytes big-endian payload length
    payload:
      lidar: '<dI' (stamp: float64, num_points: uint32) + num_points * 4 float32 (x, y, z, intensity)
      imu:   '<d10f' (stamp, qw, qx, qy, qz, gx, gy, gz, ax, ay, az)

Requires https://github.com/discoverse-dev/MuJoCo-LiDAR checked out
separately; point MUJOCO_LIDAR_SRC at its src/ directory if it is not a
sibling of this repo's parent directory (the default assumes a layout like
<workspace>/unitree_mujoco and <workspace>/third_party/MuJoCo-LiDAR).
"""

import os
import socket
import struct
import sys
import threading
import time

import matplotlib.pyplot as plt
import mujoco
import numpy as np

_DEFAULT_MUJOCO_LIDAR_SRC = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "..", "third_party", "MuJoCo-LiDAR", "src")
)
_MUJOCO_LIDAR_SRC = os.environ.get("MUJOCO_LIDAR_SRC", _DEFAULT_MUJOCO_LIDAR_SRC)
if not os.path.isdir(_MUJOCO_LIDAR_SRC):
    raise FileNotFoundError(
        f"MuJoCo-LiDAR src/ not found at '{_MUJOCO_LIDAR_SRC}'. Clone "
        "https://github.com/discoverse-dev/MuJoCo-LiDAR and set MUJOCO_LIDAR_SRC "
        "to its src/ directory."
    )
sys.path.insert(0, _MUJOCO_LIDAR_SRC)
from mujoco_lidar import MjLidarWrapper, scan_gen  # noqa: E402

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 8360
LIDAR_HZ = 10.0
IMU_HZ = 100.0
LIDAR_SITE = "livox_mid360"


class Mid360Lidar:
    """Owns the MID-360 ray caster and a best-effort TCP link to the ROS 2 bridge."""

    def __init__(self, mj_model, mj_data, locker):
        self.mj_model = mj_model
        self.mj_data = mj_data
        self.locker = locker

        self.livox = scan_gen.LivoxGenerator("mid360")
        self.lidar = MjLidarWrapper(
            mj_model,
            site_name=LIDAR_SITE,
            backend="cpu",
            cutoff_dist=30.0,
            args={"bodyexclude": mj_model.body("base_link").id},
        )
        self.gyro_id = mj_model.sensor("mid360_imu_gyro").id
        self.acc_id = mj_model.sensor("mid360_imu_acc").id
        self.quat_id = mj_model.sensor("mid360_imu_quat").id

        self._sock = None
        self._sock_lock = threading.Lock()

        self.num_rays = self.livox.sample_ray_angles()[0].shape[0]
        self.last_world_points = np.zeros((self.num_rays, 3), dtype=np.float32)

    def _sensor_slice(self, sensor_id):
        adr = self.mj_model.sensor_adr[sensor_id]
        dim = self.mj_model.sensor_dim[sensor_id]
        return self.mj_data.sensordata[adr : adr + dim]

    def _ensure_connected(self):
        if self._sock is not None:
            return True
        try:
            s = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=0.2)
            s.settimeout(None)
            self._sock = s
            return True
        except OSError:
            return False

    def _send(self, msg_type: bytes, payload: bytes) -> None:
        with self._sock_lock:
            if not self._ensure_connected():
                return
            try:
                self._sock.sendall(msg_type + struct.pack(">I", len(payload)) + payload)
            except OSError:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def publish_lidar(self) -> None:
        self.locker.acquire()
        try:
            theta, phi = self.livox.sample_ray_angles()
            theta = np.ascontiguousarray(theta, dtype=np.float32)
            phi = np.ascontiguousarray(phi, dtype=np.float32)
            self.lidar.trace_rays(self.mj_data, theta, phi)
            xyz = np.asarray(self.lidar.get_hit_points(), dtype=np.float32)
            stamp = self.mj_data.time
        finally:
            self.locker.release()

        self.last_world_points = xyz @ self.lidar.sensor_rotation.T + self.lidar.sensor_position

        xyzi = np.empty((xyz.shape[0], 4), dtype=np.float32)
        xyzi[:, :3] = xyz
        xyzi[:, 3] = 1.0
        header = struct.pack("<dI", stamp, xyzi.shape[0])
        self._send(b"L", header + xyzi.tobytes())

    def publish_imu(self) -> None:
        self.locker.acquire()
        try:
            gyro = np.array(self._sensor_slice(self.gyro_id), dtype=np.float32)
            acc = np.array(self._sensor_slice(self.acc_id), dtype=np.float32)
            quat = np.array(self._sensor_slice(self.quat_id), dtype=np.float32)  # w x y z
            stamp = self.mj_data.time
        finally:
            self.locker.release()

        payload = struct.pack(
            "<d10f",
            stamp,
            quat[0], quat[1], quat[2], quat[3],
            gyro[0], gyro[1], gyro[2],
            acc[0], acc[1], acc[2],
        )
        self._send(b"I", payload)


_CMAP = plt.get_cmap("hsv")


def init_lidar_scene(viewer, num_rays: int) -> None:
    """Allocate one sphere per ray in the passive viewer's user scene."""
    viewer.user_scn.ngeom = num_rays
    for i in range(num_rays):
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[i],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.01, 0, 0],
            pos=[0, 0, 0],
            mat=np.eye(3).flatten(),
            rgba=np.array([1, 0, 0, 0.8]),
        )


def update_lidar_scene(viewer, mid360: Mid360Lidar) -> None:
    """Move/recolor the scene spheres to the latest MID-360 hit points (by height)."""
    pts = mid360.last_world_points
    z = pts[:, 2]
    z_min, z_max = z.min(), z.max()
    z_norm = (z_max - z) / (z_max - z_min) if z_max > z_min else np.zeros_like(z)
    colors = _CMAP(z_norm)
    for i in range(mid360.num_rays):
        viewer.user_scn.geoms[i].pos[:] = pts[i]
        viewer.user_scn.geoms[i].rgba[:] = colors[i]


def run_lidar_thread(mid360: Mid360Lidar, is_running) -> None:
    """Rate-limited loop: call from a dedicated thread. `is_running` is a no-arg callable."""
    last_lidar = 0.0
    last_imu = 0.0
    while is_running():
        now = time.perf_counter()
        if now - last_lidar >= 1.0 / LIDAR_HZ:
            last_lidar = now
            mid360.publish_lidar()
        if now - last_imu >= 1.0 / IMU_HZ:
            last_imu = now
            mid360.publish_imu()
        time.sleep(0.001)
