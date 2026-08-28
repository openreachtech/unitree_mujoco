"""Command go2_ctrl's Velocity/RLBase state to walk without a physical gamepad.

State_RLBase reads velocity commands from `lowstate->joystick` (see
observations.h velocity_commands), which unitree_sdk2py_bridge.py only fills
in from a real pygame joystick. This writes the same lx/ly/rx/ry values to
/tmp/go2_synthetic_joystick.txt, which the bridge now packs into
wireless_remote as a fallback when no joystick is attached.

Usage: python send_velocity_cmd.py <lx> <ly> <rx> <duration_s>
  ly > 0 = forward, lx > 0 = strafe left, rx > 0 = turn (same convention as
  observations.h: obs uses joystick.ly() for x, -joystick.lx() for y,
  -joystick.rx() for yaw).
"""

import sys
import time

SYNTHETIC_JOYSTICK_PATH = "/tmp/go2_synthetic_joystick.txt"

lx = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
ly = float(sys.argv[2]) if len(sys.argv) > 2 else 0.3
rx = float(sys.argv[3]) if len(sys.argv) > 3 else 0.0
duration = float(sys.argv[4]) if len(sys.argv) > 4 else 5.0

end = time.time() + duration
try:
    while time.time() < end:
        with open(SYNTHETIC_JOYSTICK_PATH, "w") as f:
            f.write(f"{lx} {ly} {rx} 0.0")
        time.sleep(0.05)
finally:
    with open(SYNTHETIC_JOYSTICK_PATH, "w") as f:
        f.write("0.0 0.0 0.0 0.0")

print(f"sent lx={lx} ly={ly} rx={rx} for {duration}s, then zeroed")
