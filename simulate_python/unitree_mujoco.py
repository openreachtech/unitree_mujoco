import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge, ElasticBand

import config
from mid360_lidar import Mid360Lidar, init_lidar_scene, run_imu_thread, run_lidar_thread, update_lidar_scene


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
            )

        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    mid360 = Mid360Lidar(mj_model, mj_data, locker)
    init_lidar_scene(
        viewer,
        mid360.num_rays,
        show_points=config.ENABLE_LIDAR_POINT_VIZ,
        show_crop_plane=config.ENABLE_HEIGHTMAP_CROP_VIZ,
    )
    lidar_thread = Thread(target=run_lidar_thread, args=(mid360, viewer.is_running))
    imu_thread = Thread(target=run_imu_thread, args=(mid360, viewer.is_running))

    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
    if config.ENABLE_MID360_LIDAR:
        lidar_thread.start()
    imu_thread.start()
