# hamals_serial_bridge/protocol.py

# ======================================================
# ROS → MCU
# ======================================================
def encode_cmd(v: float, w: float) -> str:
    """
    Encode cmd_vel to MCU protocol.

    Output:
      CMD v w\n
    """
    return f"CMD {v:.3f} {w:.3f}\n"


# ======================================================
# MCU → ROS
# ======================================================
def decode_line(line: str):
    """
    Decode a single protocol line.

    Expected:
      odom,x,y,yaw,v,w
    """

    if not line:
        return None

    line = line.strip()

    if not line.lower().startswith("odom,"):
        return None

    parts = line.split(',')
    if len(parts) != 6:
        return None

    try:
        return {
            "type": "odom",
            "x": float(parts[1]),
            "y": float(parts[2]),
            "yaw": float(parts[3]),
            "v": float(parts[4]),
            "w": float(parts[5]),
        }
    except ValueError:
        return None