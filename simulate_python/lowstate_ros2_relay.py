"""Relays the Go2 sim's LowState (already built for the `rt/lowstate` DDS publish in
unitree_sdk2py_bridge.py) to a ROS 2 `/lowstate` topic, as unitree_go/msg/LowState -
the exact type/field layout the real robot's onboard computer publishes.

Same reasoning as mid360_lidar.py: this process (python3.12, unitree_sdk2py +
cyclonedds 0.10.5) cannot link rclpy (built for the system Python 3.14), so the
LowState fields already assembled in UnitreeSdk2Bridge.PublishLowState() are sent
over a local TCP connection to a separate rclpy process
(mid360_ros2_bridge/lowstate_tcp_to_ros2.py) that republishes them as a real ROS 2
message. Only the fields the sim actually fills (motor_state[i].q/dq/tau_est/mode,
imu_state.quaternion/gyroscope/accelerometer, foot_force, wireless_remote) are sent;
every other LowState field (bms_state, tick, crc, ...) is left at its ROS 2 message
default (zero) - there is no sim analog for battery telemetry etc.

Wire format (TCP, length-prefixed frames):
    1 byte  type: b'S' (lowstate)
    4 bytes big-endian payload length
    payload: '<dB' (stamp: float64, num_motor: uint8)
             + num_motor * '<3f' (q, dq, tau_est), motor order matches motor_state[]
             + '<B' (have_imu: 0/1)
             + '<10f' (qw, qx, qy, qz, gx, gy, gz, ax, ay, az) - zero if have_imu is 0
             + '<B' (have_foot_force: 0/1)
             + '<4f' (FR, FL, RR, RL foot_force, Newtons) - zero if have_foot_force is 0
             + 40 bytes (wireless_remote, raw)
"""

import socket
import struct
import threading

BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 8361


class LowStateRelay:
    """Best-effort TCP client to the ROS 2 lowstate bridge; drops frames if unconnected."""

    def __init__(self):
        self._sock = None
        self._sock_lock = threading.Lock()

    def _ensure_connected(self) -> bool:
        if self._sock is not None:
            return True
        try:
            s = socket.create_connection((BRIDGE_HOST, BRIDGE_PORT), timeout=0.2)
            s.settimeout(None)
            self._sock = s
            return True
        except OSError:
            return False

    def _send(self, payload: bytes) -> None:
        with self._sock_lock:
            if not self._ensure_connected():
                return
            try:
                self._sock.sendall(b"S" + struct.pack(">I", len(payload)) + payload)
            except OSError:
                try:
                    self._sock.close()
                except OSError:
                    pass
                self._sock = None

    def publish(
        self,
        low_state,
        num_motor: int,
        have_imu: bool,
        have_foot_force: bool,
        stamp: float,
    ) -> None:
        motor_floats = []
        for i in range(num_motor):
            m = low_state.motor_state[i]
            motor_floats.extend((m.q, m.dq, m.tau_est))

        if have_imu:
            imu = low_state.imu_state
            imu_floats = [
                imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3],
                imu.gyroscope[0], imu.gyroscope[1], imu.gyroscope[2],
                imu.accelerometer[0], imu.accelerometer[1], imu.accelerometer[2],
            ]
        else:
            imu_floats = [0.0] * 10

        if have_foot_force:
            foot_force_floats = list(low_state.foot_force[0:4])
        else:
            foot_force_floats = [0.0] * 4

        payload = (
            struct.pack("<dB", stamp, num_motor)
            + struct.pack(f"<{3 * num_motor}f", *motor_floats)
            + struct.pack("<B", 1 if have_imu else 0)
            + struct.pack("<10f", *imu_floats)
            + struct.pack("<B", 1 if have_foot_force else 0)
            + struct.pack("<4f", *foot_force_floats)
            + bytes(low_state.wireless_remote)
        )
        self._send(payload)
