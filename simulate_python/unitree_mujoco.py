import sys
import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config
from mid360_lidar import Mid360Lidar, init_lidar_scene, run_imu_thread, run_lidar_thread, run_pose_thread, update_lidar_scene

# CPython's default GIL switch interval (5ms) is right at IMU_HZ=200's own
# 5ms sampling period, so under any thread contention (sim/viewer/lidar
# threads all competing) the IMU thread doesn't get scheduled often enough to
# actually hit every 5ms slot. Shortening the interval lets the interpreter
# hand off the GIL between threads much more often, which is what real 200Hz
# sampling from a background Python thread needs.
sys.setswitchinterval(0.0005)

locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)


if config.ENABLE_ELASTIC_BAND:
    elastic_band = ElasticBand()
    if config.ROBOT == "h1" or config.ROBOT == "g1":
        band_attached_link = mj_model.body("torso_link").id
    else:
        band_attached_link = mj_model.body("base_link").id
    viewer = mujoco.viewer.launch_passive(
        mj_model, mj_data, key_callback=elastic_band.MujuocoKeyCallback
    )
else:
    viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    if config.USE_JOYSTICK:
        unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    if config.PRINT_SCENE_INFORMATION:
        unitree.PrintSceneInformation()

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        if config.ENABLE_ELASTIC_BAND:
            if elastic_band.enable:
                mj_data.xfrc_applied[band_attached_link, :3] = elastic_band.Advance(
                    mj_data.qpos[:3], mj_data.qvel[:3]
                )
        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
        else:
            # mj_step() is taking longer than SIMULATE_DT, i.e. this sim is running
            # below realtime - back-to-back mj_step() calls with no sleep at all
            # starve the other threads (imu/lidar/viewer) of any chance to run: a
            # single Python C-extension call like mj_step() only yields the GIL at
            # its own internal check points, so a tight loop of them dominates
            # scheduling. A zero-duration sleep still forces a GIL release/thread
            # switch opportunity between iterations, which is enough to let e.g.
            # run_imu_thread's 200Hz polling loop actually get scheduled instead of
            # having several of its 5ms sim-time windows coalesced into one call.
            time.sleep(0)


def PhysicsViewerThread():
    last_points_version = -1
    while viewer.is_running():
        # update_lidar_scene only touches mid360.last_world_points (a plain numpy
        # snapshot from the LiDAR thread), not mj_data, so it doesn't need the physics
        # locker - looping over 24000 points while holding it was stalling mj_step()
        # every VIEWER_DT (50Hz), which was a large part of why the sim ran at ~0.3x
        # realtime. Also skip it entirely when the scan hasn't changed since the last
        # sync (LIDAR_HZ, e.g. 10Hz, is well below VIEWER_DT's 50Hz).
        if mid360.points_version != last_points_version:
            last_points_version = mid360.points_version
            update_lidar_scene(
                viewer,
                mid360,
                show_points=config.ENABLE_LIDAR_POINT_VIZ,
                show_crop_plane=config.ENABLE_HEIGHTMAP_CROP_VIZ,
                show_fov=config.ENABLE_LIDAR_FOV_VIZ,
            )

        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    mid360 = Mid360Lidar(mj_model, mj_data, locker)
    init_lidar_scene(
        viewer,
        mid360,
        show_points=config.ENABLE_LIDAR_POINT_VIZ,
        show_crop_plane=config.ENABLE_HEIGHTMAP_CROP_VIZ,
        show_fov=config.ENABLE_LIDAR_FOV_VIZ,
    )
    lidar_thread = Thread(target=run_lidar_thread, args=(mid360, viewer.is_running))
    imu_thread = Thread(target=run_imu_thread, args=(mid360, viewer.is_running))
    pose_thread = Thread(target=run_pose_thread, args=(mid360, viewer.is_running))

    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
    if config.ENABLE_MID360_LIDAR:
        lidar_thread.start()
    imu_thread.start()
    pose_thread.start()
