from lerobot.motors.damiao.tables import MODEL_NAMES, MOTOR_LIMIT_PARAMS, MotorType
from lerobot.motors.metal.constants import (
    METAL_JOINT_LIMITS_DEG,
    METAL_JOINT_NAMES,
    METAL_MOTOR_CONFIG,
)


def test_metal_motor_types_registered():
    for name in ("metal_jlo", "metal_j2", "metal_jhi"):
        assert name in MODEL_NAMES.values()
    lim = {MODEL_NAMES[t]: MOTOR_LIMIT_PARAMS[t] for t in MotorType}
    assert lim["metal_jlo"] == (6.28, 10, 30)
    assert lim["metal_j2"] == (6.28, 10, 120)
    assert lim["metal_jhi"] == (6.28, 30, 20)


def test_metal_motor_config_ids_and_types():
    assert METAL_JOINT_NAMES == [
        "shoulder_pan",
        "shoulder_lift",
        "elbow_flex",
        "wrist_flex",
        "wrist_yaw",
        "wrist_roll",
    ]
    assert METAL_MOTOR_CONFIG["shoulder_pan"] == (0x01, 0x11, "metal_jlo")
    assert METAL_MOTOR_CONFIG["shoulder_lift"] == (0x02, 0x12, "metal_j2")
    assert METAL_MOTOR_CONFIG["wrist_yaw"] == (0x05, 0x15, "metal_jhi")
    assert METAL_MOTOR_CONFIG["gripper"] == (0x07, 0x17, "metal_jhi")


def test_joint_limits_present_for_all_joints():
    for j in METAL_JOINT_NAMES:
        lo, hi = METAL_JOINT_LIMITS_DEG[j]
        assert lo < hi
