# hamals_serial_bridge/parser.py

from typing import List, Dict
from .protocol import decode_line


class LineParser:
    """
    Serial line buffer & parser.

    Responsibilities:
    - Accumulate partial serial input
    - Split by newline
    - Decode valid protocol lines
    """

    def __init__(self):
        self._buffer = ""

    def push(self, data: str) -> List[Dict]:
        """
        Push raw serial data.

        Args:
            data (str): Raw string from serial (may be partial)

        Returns:
            List[dict]: Decoded protocol messages
        """
        messages: List[Dict] = []

        if not data:
            return messages

        self._buffer += data

        while '\n' in self._buffer:
            line, self._buffer = self._buffer.split('\n', 1)
            line = line.strip()

            if not line:
                continue

            decoded = decode_line(line)
            if decoded is not None:
                messages.append(decoded)

        return messages

    def reset(self):
        """
        Clear internal buffer.
        """
        self._buffer = ""