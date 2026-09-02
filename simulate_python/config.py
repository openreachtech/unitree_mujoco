ROBOT = "go2" # Robot name, "go2", "b2", "b2w", "h1", "go2w", "g1"
ROBOT_SCENE = "../unitree_robots/" + ROBOT + "/scene_2s1_gate_eval.xml" # Robot scene
DOMAIN_ID = 0 # Domain id (matches go2_ctrl's default; simulate/config.yaml uses 0 too)
INTERFACE = "lo" # Interface

USE_JOYSTICK = 0 # Simulate Unitree WirelessController using a gamepad
JOYSTICK_TYPE = "xbox" # support "xbox" and "switch" gamepad layout
JOYSTICK_DEVICE = 0 # Joystick number

PRINT_SCENE_INFORMATION = True # Print link, joint and sensors information of robot
ENABLE_ELASTIC_BAND = False # Virtual spring band, used for lifting h1

SIMULATE_DT = 0.005  # Need to be larger than the runtime of viewer.sync()
VIEWER_DT = 0.02  # 50 fps for viewer

# MuJoCo's native ray-vs-heightfield raycast (mj_multiRay -> mju_rayHfield) scales with
# hfield cell count, not something this project's code controls. Against the fine
# (500x800-cell) 2S1 shipyard heightfield it costs ~1.4s per 24000-ray scan, and since
# publish_lidar() holds the physics lock for the whole raycast, that stalls mj_step for
# just as long - set this False to skip starting the LiDAR thread entirely (IMU thread
# still runs) so locomotion can be evaluated on heavy terrain without that stall.
ENABLE_MID360_LIDAR = True

# The point-cloud sphere redraw in the MuJoCo viewer (up to ~3000 geoms after
# VIEWER_POINT_STRIDE subsampling) is a per-frame cost on top of the viewer's own
# rendering, separate from the raycast itself. Set False to skip it (the crop-plane
# marker still draws) - e.g. to isolate whether it's contributing to GPU contention
# with a concurrent raycast backend like taichi/Vulkan sharing the same GPU as the
# viewer's own rendering.
ENABLE_LIDAR_POINT_VIZ = False

# The red translucent flat-plane marker showing heightmap_generator's crop region
# (base_yaw_aligned footprint). Separate from ENABLE_LIDAR_POINT_VIZ - only 1 geom, so
# it's not a performance concern, just a visual toggle.
ENABLE_HEIGHTMAP_CROP_VIZ = False
