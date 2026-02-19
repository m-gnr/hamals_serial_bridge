# hamals_serial_bridge/parser.py

from typing import List, Dict
from .protocol import decode_line


class LineParser:
    """
    Serial framed protocol parser.

    Responsibilities:
    - Accumulate partial serial input
    - Realign on '$'
    - Split by newline
    - Decode framed protocol lines
    - Track valid / invalid frame statistics
    """

    def __init__(self):
        self._buffer = ""

        # Statistics
        self.valid_frames = 0
        self.invalid_frames = 0
        self.bytes_received = 0

    def push(self, data: str) -> List[Dict]:
        messages: List[Dict] = []

        if not data:
            return messages

        self.bytes_received += len(data)
        self._buffer += data

        # Realign buffer to first '$'
        start_idx = self._buffer.find('$')
        if start_idx > 0:
            self._buffer = self._buffer[start_idx:]

        # Process complete frames only
        while '\n' in self._buffer:
            line, self._buffer = self._buffer.split('\n', 1)
            line = line.strip()

            if not line:
                continue

            decoded = decode_line(line)

            if decoded is not None:
                self.valid_frames += 1
                messages.append(decoded)
            else:
                # This was a complete line but invalid frame
                self.invalid_frames += 1

        return messages

    def reset(self):
        self._buffer = ""
        self.valid_frames = 0
        self.invalid_frames = 0
        self.bytes_received = 0