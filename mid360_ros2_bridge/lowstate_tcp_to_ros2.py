"""TCP -> ROS 2 bridge for the Go2 sim's LowState.

Listens for the length-prefixed LowState frames sent by
unitree_mujoco/simulate_python/lowstate_ros2_relay.py (running in the sim's
python3.12/unitree_sdk2py process, which cannot link rclpy - see that module's
docstring) and republishes them as:

  /lowstate  unitree_go/msg/LowState

using the exact same message type the real Go2's onboard computer publishes, so
any node written against the real robot (e.g. go2_self_filter's
lowstate_to_joint_state_node) works unchanged against this sim. Only the fields
the sim actually fills are set (motor_state[i].q/dq/tau_est/mode, imu_state.quaternion/
gyroscope/accelerometer, foot_force, wireless_remote); every other LowState field
(bms_state, tick, crc, ...) keeps its message default (zero) - there is no sim analog
for battery telemetry etc.

Run with:
    source /opt/ros/lyrical/setup.bash
    source /home/mori/ros2_ws/install/setup.bash
    source mid360_ros2_bridge/.venv/bin/activate
    python mid360_ros2_bridge/lowstate_tcp_to_ros2.py
"""

import socket
import struct
import threading

import rclpy
from rclpy.node import Node
from unitree_go.msg import LowState

HOST = "127.0.0.1"
PORT = 8361


def recv_exact(conn: socket.socket, n: int) -> bytes:
    buf = bytearray()
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("peer closed")
        buf.extend(chunk)
    return bytes(buf)


class LowStateTcpBridge(Node):
    def __init__(self) -> None:
        super().__init__("lowstate_tcp_to_ros2")
        self.lowstate_pub = self.create_publisher(LowState, "/lowstate", 10)

        self._server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._server.bind((HOST, PORT))
        self._server.listen(1)
        self.get_logger().info(f"Listening for sim on {HOST}:{PORT}")

        self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
        self._accept_thread.start()

    def _accept_loop(self) -> None:
        while rclpy.ok():
            conn, addr = self._server.accept()
            self.get_logger().info(f"sim connected from {addr}")
            try:
                self._read_loop(conn)
            except (ConnectionError, OSError) as exc:
                self.get_logger().warning(f"sim connection lost: {exc}")
            finally:
                conn.close()

    def _read_loop(self, conn: socket.socket) -> None:
        while rclpy.ok():
            header = recv_exact(conn, 5)
            msg_type = header[0:1]
            (length,) = struct.unpack(">I", header[1:5])
            payload = recv_exact(conn, length)
            if msg_type == b"S":
                self._publish_lowstate(payload)

    def _publish_lowstate(self, payload: bytes) -> None:
        offset = 0
        _stamp, num_motor = struct.unpack_from("<dB", payload, offset)
        offset += struct.calcsize("<dB")

        motor_floats = struct.unpack_from(f"<{3 * num_motor}f", payload, offset)
        offset += struct.calcsize(f"<{3 * num_motor}f")

        (have_imu,) = struct.unpack_from("<B", payload, offset)
        offset += struct.calcsize("<B")

        imu_floats = struct.unpack_from("<10f", payload, offset)
        offset += struct.calcsize("<10f")

        (have_foot_force,) = struct.unpack_from("<B", payload, offset)
        offset += struct.calcsize("<B")

        foot_force_floats = struct.unpack_from("<4f", payload, offset)
        offset += struct.calcsize("<4f")

        wireless_remote = payload[offset : offset + 40]

        msg = LowState()
        for i in range(num_motor):
            q, dq, tau_est = motor_floats[3 * i : 3 * i + 3]
            msg.motor_state[i].q = q
            msg.motor_state[i].dq = dq
            msg.motor_state[i].tau_est = tau_est
            # 1 == servo-on/under active control on the real robot; the sim always
            # runs its motors under PD control, so this is always true here.
            msg.motor_state[i].mode = 1

        if have_imu:
            qw, qx, qy, qz, gx, gy, gz, ax, ay, az = imu_floats
            msg.imu_state.quaternion = [qw, qx, qy, qz]
            msg.imu_state.gyroscope = [gx, gy, gz]
            msg.imu_state.accelerometer = [ax, ay, az]

        if have_foot_force:
            msg.foot_force = [int(round(f)) for f in foot_force_floats]

        msg.wireless_remote = list(wireless_remote)

        self.lowstate_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = LowStateTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
