"""Application settings and constants."""

# Serial communication
DEFAULT_BAUD_RATE = 9600
BAUD_RATES = ["4800", "9600", "19200", "38400", "57600", "115200"]

# UI settings
UPDATE_INTERVAL_MS = 500  # GUI update interval

# GPS settings
MAX_POSITION_HISTORY = 10

# CSV logging
CSV_LOG_HEADERS = [
    'HostTimestamp', 'FixTimestamp', 'Latitude', 'Longitude', 'Altitude',
    'Speed_KPH', 'SatellitesInView', 'SatellitesUsedCount', 'FixQuality',
    'HDOP', 'PDOP', 'VDOP', 'FixType', 'AccuracyEstimate_m', 'GeoidHeight',
    'GNSSSystemsUsed', 'RawNMEA'
]

# Fix type and quality mappings
FIX_TYPE_MAP = {
    0: "No Fix", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS",
    4: "RTK Fixed", 5: "RTK Float", 6: "Estimated (DR)",
    7: "Manual", 8: "Simulation"
}

FIX_QUALITY_MAP = {
    0: 'Invalid', 1: 'GPS fix (SPS)', 2: 'DGPS fix', 3: 'PPS fix',
    4: 'RTK Fixed', 5: 'RTK Float', 6: 'Estimated (DR)',
    7: 'Manual input', 8: 'Simulation'
}

# Data smoothing settings
SMOOTHING_NONE = "None"
SMOOTHING_AVG = "Moving Average"
SMOOTHING_KALMAN = "Kalman Filter"

# Try importing NumPy for Kalman filter
try:
    import numpy as np
    NUMPY_AVAILABLE = True
    SMOOTHING_METHODS = [SMOOTHING_NONE, SMOOTHING_AVG, SMOOTHING_KALMAN]
except ImportError:
    NUMPY_AVAILABLE = False
    SMOOTHING_METHODS = [SMOOTHING_NONE, SMOOTHING_AVG]
    print("WARNING: NumPy not found. Kalman filter smoothing disabled.")