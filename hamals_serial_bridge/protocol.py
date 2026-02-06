# hamals_serial_bridge/protocol.py

from typing import Optional, Dict


# ======================================================
# ROS → MCU
# ======================================================
def encode_cmd(v: float, w: float) -> str:
    """
    Encode velocity command for MCU.

    Format:
        CMD,v,w\n
    """
    return f"CMD,{v:.4f},{w:.4f}\n"


# ======================================================
# MCU → ROS
# ======================================================
def decode_line(line: str) -> Optional[Dict]:
    """
    Decode a single line received from MCU.

    Expected formats:
        ODOM,x,y,yaw,v,w

    Returns:
        dict or None
    """
    if not line:
        return None

    parts = line.split(',')
    if len(parts) == 0:
        return None

    msg_type = parts[0]

    # --------------------
    # ODOMETRY
    # --------------------
    if msg_type == 'ODOM' and len(parts) == 6:
        try:
            return {
                'type': 'odom',
                'x':   float(parts[1]),
                'y':   float(parts[2]),
                'yaw': float(parts[3]),
                'v':   float(parts[4]),
                'w':   float(parts[5]),
            }
        except ValueError:
            return None

    # UNKNOWN
    return None