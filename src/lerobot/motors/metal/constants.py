"""Metal arm constants (CAN ids, motor types, joint limits). Pure data; no hardware, no ROS."""

METAL_JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
METAL_GRIPPER_NAME = "gripper"
GRIPPER_MAX_MM = 80.0

# name -> (send_id, recv_id, motor_type_str)
METAL_MOTOR_CONFIG: dict[str, tuple[int, int, str]] = {
    "joint1": (0x01, 0x11, "metal_jlo"),
    "joint2": (0x02, 0x12, "metal_j2"),
    "joint3": (0x03, 0x13, "metal_jlo"),
    "joint4": (0x04, 0x14, "metal_jlo"),
    "joint5": (0x05, 0x15, "metal_jhi"),
    "joint6": (0x06, 0x16, "metal_jhi"),
    "gripper": (0x07, 0x17, "metal_jhi"),
}

# degrees, from vendor motor_config.cpp position_min/max
METAL_JOINT_LIMITS_DEG: dict[str, tuple[float, float]] = {
    "joint1": (-160.0, 160.0),
    "joint2": (-180.0, 0.0),
    "joint3": (0.0, 180.0),
    "joint4": (-123.0, 81.0),
    "joint5": (-85.0, 85.0),
    "joint6": (-145.0, 145.0),
}

# Follower MIT gains (kp, kd) from vendor motor_config.cpp follow_mit_kp/follow_mit_kd.
# lerobot's MIT_KP_RANGE=(0,500) / MIT_KD_RANGE=(0,5) match the vendor exactly, so these
# copy over with identical meaning. Bus defaults (kp=10) are far too soft to hold the arm
# against gravity, so the follower sets these at connect for firm, responsive tracking.
METAL_FOLLOWER_GAINS: dict[str, tuple[float, float]] = {
    "joint1": (200.0, 3.0),
    "joint2": (500.0, 5.0),
    "joint3": (400.0, 5.0),
    "joint4": (200.0, 2.0),
    "joint5": (20.0, 0.1),
    "joint6": (20.0, 0.1),
    "gripper": (20.0, 0.1),
}
