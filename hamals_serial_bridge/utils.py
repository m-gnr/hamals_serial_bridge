import math

def yaw_to_quat(yaw: float):
    """
    Yaw (rad) → quaternion
    Planar robot: roll = pitch = 0
    """
    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)
    return (0.0, 0.0, qz, qw)