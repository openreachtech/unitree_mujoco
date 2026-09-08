"""TCP -> ROS 2 bridge for the Go2 MID-360 sim pipeline.

Listens for the length-prefixed lidar/imu frames sent by
unitree_mujoco/simulate_python/mid360_lidar.py (running in a separate
python3.12 process alongside the DDS bridge to go2_ctrl) and republishes
them as:

  /livox/lidar  sensor_msgs/msg/PointCloud2  (x, y, z, intensity), frame_id "livox_frame"
  /livox/imu    sensor_msgs/msg/Imu                                frame_id "livox_frame"
  /clock        rosgraph_msgs/msg/Clock, driven by MuJoCo's mj_data.time (the sim time
                already carried in each TCP frame's payload, previously unused - message
                headers were wall-clock stamped instead)
  tf odom -> base_link, from MuJoCo's own ground-truth base_link pose (PORT_POSE) - a
                drop-in replacement for rko_lio's odom->base_link when its LIO estimate
                either isn't needed or, per online_imu_rate_node's lack of any recovery
                path from a "LiDAR scan delta exceeds 1 second" InputError, isn't
                reliably available at all. See run_pose_thread in mid360_lidar.py.

Any other node in the graph that wants to reason about message timing consistently with
the simulation (TF interpolation, LIO time-synchronization, rosbag replay speed, etc.)
should be launched with use_sim_time:=true so it picks up /clock instead of wall time.
This node itself must NOT set use_sim_time - it is the /clock source, not a consumer.

LiDAR and IMU each get their OWN TCP connection (PORT_LIDAR / PORT_IMU), each read by
its own dedicated thread here. They used to share a single connection/read loop: a
24000-point LiDAR frame (~384KB) takes measurably long to recv()+parse+publish as a
PointCloud2, and while that was happening the much smaller, much more frequent IMU
frames queued up behind it on the same TCP stream couldn't be read at all (classic
head-of-line blocking). That capped observed /livox/imu throughput at under 100Hz
instead of the real MID-360's 200Hz, even though mid360_lidar.py's IMU thread was
generating samples at close to the correct rate the whole time - separating the streams
let each side run at its own pace.

Run with:
    source /opt/ros/humble/setup.bash
    source mid360_ros2_bridge/.venv/bin/activate
    python mid360_ros2_bridge/mid360_tcp_to_ros2.py
"""

import socket
import struct
import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_ros import TransformBroadcaster

HOST = "127.0.0.1"
PORT_LIDAR = 8360
PORT_IMU = 8361
PORT_POSE = 8362
FRAME_ID = "livox_frame"
ODOM_FRAME_ID = "odom"
BASE_FRAME_ID = "base_link"


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf.extend(chunk)
    return bytes(buf)


class Mid360TcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("mid360_tcp_to_ros2")
        self.cloud_pub = self.create_publisher(PointCloud2, "/livox/lidar", 1)
        self.imu_pub = self.create_publisher(Imu, "/livox/imu", 10)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        # Highest sim_time seen from EITHER stream, for /clock only (see
        # _stamp_from_sim_time). Written from the lidar, imu and pose accept
        # threads, so guarded by its own lock.
        self._last_sim_time = 0.0
        self._clock_lock = threading.Lock()

        self._lidar_thread = self._start_listener(
            "lidar", PORT_LIDAR, self._lidar_read_loop
        )
        self._imu_thread = self._start_listener("imu", PORT_IMU, self._imu_read_loop)
        self._pose_thread = self._start_listener(
            "pose", PORT_POSE, self._pose_read_loop
        )

    def _start_listener(self, name: str, port: int, read_loop) -> threading.Thread:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, port))
        server.listen(1)
        self.get_logger().info(f"Listening for sim {name} on {HOST}:{port}")

        def accept_loop() -> None:
            while rclpy.ok():
                conn, addr = server.accept()
                self.get_logger().info(f"sim {name} connected from {addr}")
                # A new sim connection means unitree_mujoco.py (re)started, so
                # mj_data.time reset to ~0 (mj_resetDataKeyframe). Without
                # resetting this too, the old high-water mark would never let
                # a new, lower sim_time through the monotonic clamp below -
                # every /clock stamp would freeze at whatever value this was
                # left at by the previous sim run. Both the lidar and imu
                # connections reconnect together on a sim restart, so either
                # one arriving resets it for both.
                with self._clock_lock:
                    self._last_sim_time = 0.0
                try:
                    read_loop(conn)
                except (ConnectionError, OSError) as exc:
                    self.get_logger().warning(f"sim {name} connection lost: {exc}")
                finally:
                    conn.close()

        thread = threading.Thread(target=accept_loop, daemon=True)
        thread.start()
        return thread

    def _lidar_read_loop(self, conn: socket.socket) -> None:
        while rclpy.ok():
            header = recv_exact(conn, 5)
            (length,) = struct.unpack(">I", header[1:5])
            self._publish_lidar(recv_exact(conn, length))

    def _imu_read_loop(self, conn: socket.socket) -> None:
        while rclpy.ok():
            header = recv_exact(conn, 5)
            (length,) = struct.unpack(">I", header[1:5])
            self._publish_imu(recv_exact(conn, length))

    def _pose_read_loop(self, conn: socket.socket) -> None:
        while rclpy.ok():
            header = recv_exact(conn, 5)
            (length,) = struct.unpack(">I", header[1:5])
            self._publish_pose(recv_exact(conn, length))

    def _publish_lidar(self, payload: bytes) -> None:
        stamp, num_points = struct.unpack_from("<dI", payload, 0)
        points = np.frombuffer(payload, dtype=np.float32, offset=12, count=num_points * 4)
        points = points.reshape(num_points, 4)

        msg = PointCloud2()
        msg.header.stamp = self._stamp_from_sim_time(stamp)
        msg.header.frame_id = FRAME_ID
        msg.height = 1
        msg.width = int(num_points)
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points).tobytes()
        self.cloud_pub.publish(msg)

    def _publish_imu(self, payload: bytes) -> None:
        stamp, qw, qx, qy, qz, gx, gy, gz, ax, ay, az = struct.unpack("<d10f", payload)

        msg = Imu()
        msg.header.stamp = self._stamp_from_sim_time(stamp)
        msg.header.frame_id = FRAME_ID
        msg.orientation.w = float(qw)
        msg.orientation.x = float(qx)
        msg.orientation.y = float(qy)
        msg.orientation.z = float(qz)
        msg.angular_velocity.x = float(gx)
        msg.angular_velocity.y = float(gy)
        msg.angular_velocity.z = float(gz)
        msg.linear_acceleration.x = float(ax)
        msg.linear_acceleration.y = float(ay)
        msg.linear_acceleration.z = float(az)
        self.imu_pub.publish(msg)

    def _publish_pose(self, payload: bytes) -> None:
        stamp, x, y, z, qw, qx, qy, qz = struct.unpack("<d7d", payload)

        msg = TransformStamped()
        msg.header.stamp = self._stamp_from_sim_time(stamp)
        msg.header.frame_id = ODOM_FRAME_ID
        msg.child_frame_id = BASE_FRAME_ID
        msg.transform.translation.x = x
        msg.transform.translation.y = y
        msg.transform.translation.z = z
        msg.transform.rotation.w = qw
        msg.transform.rotation.x = qx
        msg.transform.rotation.y = qy
        msg.transform.rotation.z = qz
        self.tf_broadcaster.sendTransform(msg)

    def _stamp_from_sim_time(self, sim_time: float) -> Time:
        # Each message keeps its OWN true sim_time as its header stamp - never clamped or
        # overwritten by the other stream. rko_lio's ThreadedNode buffering (imu_callback/
        # lidar_callback in threaded_node.cpp) requires imu_buffer.back().time to exceed
        # lidar's own scan stamp before it can register that scan; the lidar and IMU
        # threads in mid360_lidar.py both read mj_data.time independently and this bridge
        # receives their TCP frames in send-order, not capture-order, so a lidar frame
        # (slow to raycast + serialize) can arrive here AFTER several newer-stamped IMU
        # frames. A single shared, monotonically-clamped "last sim time" used to bump that
        # late-arriving lidar frame's stamp UP to match the newest IMU seen so far, which
        # made it look newer than IMU samples that had truly already passed it - rko_lio
        # then needed IMU newer still, backlog built up in lidar_buffer, and scans were
        # eventually dropped with "Registration lidar buffer limit reached". Only /clock
        # (which other nodes use for node->now()/TF lookups, not for this ordering
        # invariant) needs a monotonically non-decreasing value, so the clamp lives there
        # only, separate from the per-message stamp returned below.
        sec = int(sim_time)
        nanosec = int(round((sim_time - sec) * 1e9))
        stamp = Time(sec=sec, nanosec=nanosec)
        with self._clock_lock:
            if sim_time > self._last_sim_time:
                self._last_sim_time = sim_time
                self.clock_pub.publish(Clock(clock=Time(sec=sec, nanosec=nanosec)))
        return stamp


def main() -> None:
    rclpy.init()
    node = Mid360TcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
