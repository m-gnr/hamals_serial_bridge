# hamals_serial_bridge/protocol.py

# ======================================================
# CHECKSUM
# ======================================================

def compute_checksum(payload: str) -> int:
    """
    XOR checksum of payload (no $, no *).
    """
    cs = 0
    for c in payload:
        cs ^= ord(c)
    return cs


# ======================================================
# ROS → MCU
# ======================================================

def encode_cmd(v: float, w: float) -> str:
    """
    Encode cmd_vel to framed protocol:

      $CMD,v,w*CS\n
    """
    payload = f"CMD,{v:.3f},{w:.3f}"
    cs = compute_checksum(payload)
    return f"${payload}*{cs:02X}\n"


# ======================================================
# MCU → ROS
# ======================================================

def decode_line(line: str):
    """
    Decode framed protocol line.

    Expected:
      $ODOM,x,y,yaw,v,w*CS
    """

    if not line:
        return None

    line = line.strip()

    # Must start with $
    if not line.startswith("$"):
        return None

    # Split checksum
    try:
        body, cs_part = line[1:].split("*")
    except ValueError:
        return None

    # Validate checksum
    try:
        received_cs = int(cs_part, 16)
    except ValueError:
        return None

    calc_cs = compute_checksum(body)
    if calc_cs != received_cs:
        return None

    # Parse payload
    if not body.startswith("ODOM,"):
        return None

    parts = body.split(",")
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