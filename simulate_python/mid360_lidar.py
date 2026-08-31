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
IMU_HZ = 200.0  # matches the real MID-360's IMU output rate
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
        self.points_version = 0  # bumped in publish_lidar(); lets viewer skip redundant redraws

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
        # Pure computation, no mj_data dependency - do it before taking the physics
        # lock so mj_step() isn't blocked any longer than the actual raycast needs.
        theta, phi = self.livox.sample_ray_angles()
        theta = np.ascontiguousarray(theta, dtype=np.float32)
        phi = np.ascontiguousarray(phi, dtype=np.float32)

        self.locker.acquire()
        try:
            self.lidar.trace_rays(self.mj_data, theta, phi)
            xyz = np.asarray(self.lidar.get_hit_points(), dtype=np.float32)
            stamp = self.mj_data.time
        finally:
            self.locker.release()

        self.last_world_points = xyz @ self.lidar.sensor_rotation.T + self.lidar.sensor_position
        self.points_version += 1

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


# Only a subsample of rays gets a sphere in the MuJoCo viewer: looping over all 24000 in
# Python (per-element ctypes attribute writes, no numpy vectorization available for
# user_scn.geoms) was a meaningful chunk of frame time even after moving it off the
# physics lock. The full, un-subsampled cloud is unaffected - it still goes out over
# TCP/ROS 2 for rko_lio/heightmap_generator; this only thins out the local 3D preview.
VIEWER_POINT_STRIDE = 8
_FIXED_RGBA = np.array([1.0, 0.25, 0.0, 0.8], dtype=np.float32)  # flat color, no per-frame colormap

# heightmap_generator's crop region, in its own base_yaw_aligned frame (origin at
# base_link, X forward per yaw, Z up, roll/pitch ignored). Keep in sync with
# unitree_go2_locomotion_heightmap/config/heightmap_generator.yaml (x_min/x_max/y_min/y_max)
# by hand - there's no shared source between the two repos. Drawn as a flat plane (a thin
# box) at floor height rather than a 3D volume - only the XY footprint matters for a quick
# "is this in the right place" check.
_PLANE_HALF_THICKNESS = 0.003
_CROP_PLANES = {
    "heightmap_grid": (-0.6, 1.0, -0.5, 0.5, 0.0, (1.0, 0.0, 0.0, 0.15)),
}
_CROP_PLANE_NAMES = list(_CROP_PLANES.keys())


def init_lidar_scene(viewer, num_rays: int) -> int:
    """Allocate one sphere per displayed (subsampled) ray, plus one flat plane per entry
    in _CROP_PLANES marking heightmap_generator's crop regions (base_yaw_aligned: origin
    at base_link, X forward per yaw, floor-height). Returns the displayed ray count (the
    planes are extra, starting at index num_displayed).
    """
    num_displayed = (num_rays + VIEWER_POINT_STRIDE - 1) // VIEWER_POINT_STRIDE
    viewer.user_scn.ngeom = num_displayed + len(_CROP_PLANES)
    for i in range(num_displayed):
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[i],
            type=mujoco.mjtGeom.mjGEOM_SPHERE,
            size=[0.015, 0, 0],
            pos=[0, 0, 0],
            mat=np.eye(3).flatten(),
            rgba=_FIXED_RGBA,
        )
    for j, name in enumerate(_CROP_PLANE_NAMES):
        x_min, x_max, y_min, y_max, _z_center, rgba = _CROP_PLANES[name]
        mujoco.mjv_initGeom(
            viewer.user_scn.geoms[num_displayed + j],
            type=mujoco.mjtGeom.mjGEOM_BOX,
            size=[(x_max - x_min) / 2, (y_max - y_min) / 2, _PLANE_HALF_THICKNESS],
            pos=[0, 0, 0],
            mat=np.eye(3).flatten(),
            rgba=np.array(rgba, dtype=np.float32),
        )
    return num_displayed


def _yaw_only_rotmat(base_xmat: np.ndarray) -> np.ndarray:
    """Rotation matrix for base_link's yaw only (matches heightmap_generator's
    base_yaw_aligned frame, which ignores roll/pitch)."""
    yaw = np.arctan2(base_xmat[1, 0], base_xmat[0, 0])
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def update_lidar_scene(viewer, mid360: Mid360Lidar) -> None:
    """Move the (subsampled) scene spheres to the latest MID-360 hit points, and the
    crop planes to the robot's current (yaw-aligned) pose.

    No per-frame colormap: a fixed color set once in init_lidar_scene is enough for a
    "does the scan look right" preview, and skips an expensive per-point colormap lookup.
    """
    pts = mid360.last_world_points[::VIEWER_POINT_STRIDE]
    geoms = viewer.user_scn.geoms
    for i in range(pts.shape[0]):
        geoms[i].pos[:] = pts[i]

    base_id = mid360.mj_model.body("base_link").id
    base_pos = mid360.mj_data.xpos[base_id]
    base_mat = mid360.mj_data.xmat[base_id].reshape(3, 3)
    yaw_mat = _yaw_only_rotmat(base_mat)

    for j, name in enumerate(_CROP_PLANE_NAMES):
        x_min, x_max, y_min, y_max, z_center, _ = _CROP_PLANES[name]
        center_local = np.array([(x_min + x_max) / 2, (y_min + y_max) / 2, z_center])
        plane_geom = geoms[pts.shape[0] + j]
        plane_geom.pos[:] = base_pos + yaw_mat @ center_local
        plane_geom.mat[:] = yaw_mat  # geom.mat is (3,3) here, unlike mjv_initGeom's flat (9,) arg


def run_lidar_thread(mid360: Mid360Lidar, is_running) -> None:
    """Rate-limited loop: call from a dedicated thread. `is_running` is a no-arg callable.

    Gated on simulated time (mj_data.time), not wall-clock: the 24000-ray MID-360 cast
    plus physics plus viewer rendering is heavy enough that this sim runs well below
    real-time (observed ~0.3x realtime), so a wall-clock-based 10Hz gate actually fires
    every ~0.03s of *simulated* time instead of every 0.1s. RKO-LIO (running with
    use_sim_time:=true, driven by /clock from this same sim time) then sees erratic,
    often very short simulated intervals between scans, with correspondingly few IMU
    samples in between - this was a second cause of the "1 IMU message(s) in interval
    between two lidar scans" warning, on top of the thread-starvation bug fixed below.
    Gating on mj_data.time instead makes LIDAR_HZ/IMU_HZ mean what they say regardless
    of how slow real-time execution actually is.

    LiDAR-only: this raycasts 24000 rays and sends a large payload over TCP, which is
    slow enough (tens of ms) that sharing a thread with IMU publishing starved the IMU
    side down to the same cadence as the LiDAR. Run IMU on its own thread (run_imu_thread)
    so it keeps its own cadence regardless of how long a LiDAR publish takes.
    """
    last_lidar = -1.0
    while is_running():
        now = mid360.mj_data.time
        if now - last_lidar >= 1.0 / LIDAR_HZ:
            last_lidar = now
            mid360.publish_lidar()
        time.sleep(0.001)


def run_imu_thread(mid360: Mid360Lidar, is_running) -> None:
    """Rate-limited loop: call from its own dedicated thread, separate from the LiDAR.

    Gated on simulated time (mj_data.time) - see run_lidar_thread's docstring.
    """
    last_imu = -1.0
    while is_running():
        now = mid360.mj_data.time
        if now - last_imu >= 1.0 / IMU_HZ:
            last_imu = now
            mid360.publish_imu()
        time.sleep(0.001)
