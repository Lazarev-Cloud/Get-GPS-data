"""Serial port utility functions."""

import serial.tools.list_ports
from typing import List, Tuple

def get_available_ports() -> List[Tuple[str, str]]:
    """Get a list of available serial ports with descriptions.
    
    Returns:
        List of tuples (port, description)
    """
    ports = serial.tools.list_ports.comports()
    port_list = []
    for port in sorted(ports, key=lambda p: p.device):
        desc = f"{port.device}: {port.description}"
        port_list.append((port.device, desc))
    return port_list