import serial
import serial.tools.list_ports
import pynmea2
import time
import threading
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import datetime
import os
import csv
import webbrowser
import platform
from typing import Optional, List, Dict, Any, Tuple, Union, Set

# Optional dependency
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False
    print("WARNING: NumPy not found. Kalman filter smoothing disabled.")

# --- Constants ---
DEFAULT_BAUD_RATE = 9600
BAUD_RATES = ["4800", "9600", "19200", "38400", "57600", "115200"]
UPDATE_INTERVAL_MS = 500  # GUI update interval
MAX_POSITION_HISTORY = 10
CSV_LOG_HEADERS = [
    'HostTimestamp', 'FixTimestamp', 'Latitude', 'Longitude', 'Altitude',
    'Speed_KPH', 'SatellitesInView', 'SatellitesUsedCount', 'FixQuality', 'HDOP', 'PDOP', 'VDOP',
    'FixType', 'AccuracyEstimate_m', 'GeoidHeight', 'GNSSSystemsUsed',
    'RawNMEA'
]
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
SMOOTHING_NONE = "None"
SMOOTHING_AVG = "Moving Average"
SMOOTHING_KALMAN = "Kalman Filter"
SMOOTHING_METHODS = [SMOOTHING_NONE, SMOOTHING_AVG]
if NUMPY_AVAILABLE:
    SMOOTHING_METHODS.append(SMOOTHING_KALMAN)


# --- Helper Functions ---
def degrees_to_dms(degrees: Optional[float]) -> str:
    """Convert decimal degrees to Degrees Minutes Seconds string."""
    if not isinstance(degrees, (float, int)) or degrees == 0.0:
        return "--"
    is_negative = degrees < 0
    degrees = abs(degrees)
    d = int(degrees)
    m = (degrees - d) * 60
    return f"{'-' if is_negative else ''}{d}° {m:.4f}'"

def knots_to_kph(knots: Optional[Union[float, str]]) -> float:
    """Convert speed from knots to km/h."""
    try:
        return float(knots) * 1.852 if knots is not None else 0.0
    except (ValueError, TypeError):
        return 0.0

# --- GPS Reader Class ---
class GPSReader:
    """Handles serial communication, NMEA parsing, and data processing with improved data persistence."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD_RATE):
        """Initialize GPS reader."""
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.running = threading.Event()
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock() # Lock for accessing shared data

        # --- Data Storage (Persistent) ---
        self.current_data: Dict[str, Any] = self._get_default_data()
        self.position_history: List[Tuple[float, float, float]] = []
        self.max_history_size = MAX_POSITION_HISTORY
        self.smoothing_method = SMOOTHING_NONE

        # --- Logging ---
        self.log_file: Optional[object] = None
        self.csv_writer: Optional[csv.writer] = None
        self.log_enabled = False
        self.log_filename: Optional[str] = None

        # --- Kalman Filter ---
        self.kalman_filter: Optional[Dict[str, Any]] = None
        if NUMPY_AVAILABLE:
            self.initialize_kalman_filter()

    def _get_default_data(self) -> Dict[str, Any]:
        """Return a dictionary with initial default values for GPS data."""
        return {
            'host_timestamp': None, 'fix_timestamp': None,
            'lat': 0.0, 'lon': 0.0, 'altitude': 0.0,
            'raw_lat': 0.0, 'raw_lon': 0.0, 'raw_alt': 0.0, # Store raw values before smoothing
            'speed_kph': 0.0,
            'satellites_in_view': 0, 'satellites_used': [], # List of PRNs used in fix
            'fix_quality': 0, 'hdop': 0.0, 'pdop': 0.0, 'vdop': 0.0,
            'geoid_height': 0.0, 'fix_type': 0,
            'satellite_data': [], # Detailed list of dicts for satellites in view (from GSV)
            'gnss_systems_used': set(), # Set of strings like {"GPS", "GLONASS"}
            'accuracy_estimate': 0.0, 'raw_sentence': '',
        }

    # --- Kalman Filter Methods ---
    def initialize_kalman_filter(self):
        """Initialize Kalman filter state if NumPy is available."""
        if not NUMPY_AVAILABLE:
            self.kalman_filter = None
            return

        self.kalman_filter = {
            'x': np.zeros(3),       # State vector [lat, lon, alt]
            'P': np.eye(3) * 500,   # Initial state covariance (high uncertainty)
            'F': np.eye(3),         # State transition matrix (identity for static position model)
            'H': np.eye(3),         # Measurement matrix
            'Q': np.eye(3) * 0.05,  # Process noise covariance (model uncertainty)
            'R': np.eye(3) * 10.0   # Measurement noise covariance (sensor uncertainty)
        }
        print("Kalman filter initialized.")

    def _apply_kalman_filter(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Apply Kalman filter step. Returns last known filtered position if input is invalid."""
        if not self.kalman_filter or not NUMPY_AVAILABLE:
            return lat, lon, alt

        kf = self.kalman_filter
        last_filtered_pos = (float(kf['x'][0]), float(kf['x'][1]), float(kf['x'][2]))

        # If input is invalid (0,0), return the last known filtered position
        if lat == 0.0 or lon == 0.0:
            return last_filtered_pos

        try:
            x = kf['x']
            P = kf['P']
            F = kf['F']
            Q = kf['Q']
            H = kf['H']
            R = kf['R']
            z = np.array([lat, lon, alt]) # Measurement

            # --- Predict ---
            x_pred = F @ x
            P_pred = F @ P @ F.T + Q

            # --- Update ---
            y = z - H @ x_pred             # Measurement residual
            S = H @ P_pred @ H.T + R     # Residual covariance
            try:
                S_inv = np.linalg.inv(S)
            except np.linalg.LinAlgError:
                print("Kalman filter warning: Singular matrix S. Skipping update.")
                # Return previous state if update fails numerically
                return last_filtered_pos

            K = P_pred @ H.T @ S_inv      # Kalman gain
            x_new = x_pred + K @ y
            P_new = (np.eye(len(x)) - K @ H) @ P_pred

            # Save updated state
            kf['x'] = x_new
            kf['P'] = P_new

            return float(x_new[0]), float(x_new[1]), float(x_new[2])

        except np.linalg.LinAlgError:
            print("Kalman filter error: Singular matrix during calculation. Resetting filter.")
            self.initialize_kalman_filter() # Reset filter on numerical instability
            return lat, lon, alt # Return raw on error before reset state is used
        except Exception as e:
            print(f"Kalman filter error: {e}")
            return lat, lon, alt # Return raw on other errors

    # --- Moving Average Methods ---
    def _add_to_position_history(self, lat: float, lon: float, alt: float):
        """Add valid position to history buffer."""
        if lat == 0.0 or lon == 0.0: return # Don't add invalid points
        self.position_history.append((lat, lon, alt))
        # Maintain history size limit
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)

    def _get_averaged_position(self) -> Tuple[float, float, float]:
        """Calculate simple moving average of recent valid positions."""
        if not self.position_history:
            # Return last known raw position if history is empty
             with self.lock:
                 return self.current_data['raw_lat'], self.current_data['raw_lon'], self.current_data['raw_alt']

        n = len(self.position_history)
        avg_lat = sum(pos[0] for pos in self.position_history) / n
        avg_lon = sum(pos[1] for pos in self.position_history) / n
        avg_alt = sum(pos[2] for pos in self.position_history) / n
        return avg_lat, avg_lon, avg_alt

    # --- Connection and Configuration ---
    def connect(self) -> bool:
        """Connect to the serial port."""
        if self.ser and self.ser.is_open:
            print("Already connected.")
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Connected to {self.port} at {self.baud} baud")
            time.sleep(0.5) # Allow device to settle
            self._configure_gps_module()
            return True
        except serial.SerialException as e:
            print(f"ERROR: Failed to connect to {self.port}: {e}")
            self.ser = None
            return False
        except Exception as e:
            print(f"ERROR: An unexpected error occurred during connection: {e}")
            self.ser = None
            return False

    def _configure_gps_module(self):
        """Send configuration commands (UBX) to u-blox modules (optional)."""
        if not self.ser or not self.ser.is_open:
            print("WARN: Cannot configure, serial port not open.")
            return

        # Example UBX Commands (Hex strings converted to bytes)
        commands = {
            "Enable SBAS": "B5 62 06 16 08 00 01 01 03 00 00 00 00 00 29 8B",
            "Set Rate 5Hz": "B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A", # 200ms interval
            "Enable GNGGA": "B5 62 06 01 08 00 F0 00 01 01 01 01 01 01 04 4B",
            "Enable GNGLL": "B5 62 06 01 08 00 F0 01 00 01 00 00 00 00 00 46", # Disable GLL
            "Enable GNGSA": "B5 62 06 01 08 00 F0 02 01 01 01 01 01 01 06 55",
            "Enable GPGSV": "B5 62 06 01 08 00 F0 03 01 01 01 01 01 01 07 5A", # GPS
            "Enable GLGSV": "B5 62 06 01 08 00 F0 06 01 01 01 01 01 01 0A 6B", # GLONASS
            "Enable GNRMC": "B5 62 06 01 08 00 F0 04 01 01 01 01 01 01 08 5F",
            "Enable GNVTG": "B5 62 06 01 08 00 F0 05 01 01 01 01 01 01 09 64",
        }

        print("Attempting to send configuration commands to GPS module...")
        try:
            for name, hex_cmd in commands.items():
                try:
                    cmd_bytes = bytearray.fromhex(hex_cmd)
                    self.ser.write(cmd_bytes)
                    # print(f"  Sent: {name}") # Reduce verbosity
                    time.sleep(0.1)
                except ValueError:
                    print(f"  ERROR: Invalid hex string for command '{name}': {hex_cmd}")
                except Exception as write_err:
                     print(f"  ERROR writing command '{name}': {write_err}")
            print("GPS configuration commands sent (if module supports UBX).")
        except Exception as e:
            print(f"ERROR during GPS configuration: {e}")

    def disconnect(self):
        """Close the serial port."""
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
                print("Disconnected from GPS.")
            except Exception as e:
                print(f"ERROR during disconnect: {e}")
        self.ser = None

    # --- Logging Methods ---
    def start_logging(self, filename: str) -> bool:
        """Start logging GPS data to a CSV file."""
        self.stop_logging()
        try:
            # Use 'with' statement for automatic file closing on error/exit
            self.log_file = open(filename, 'w', newline='', encoding='utf-8')
            self.csv_writer = csv.writer(self.log_file)
            self.csv_writer.writerow(CSV_LOG_HEADERS)
            self.log_filename = filename
            self.log_enabled = True
            print(f"Logging started to {self.log_filename}")
            return True
        except IOError as e:
            print(f"ERROR: Cannot open log file '{filename}': {e}")
            self.log_file = None
            self.csv_writer = None
            self.log_enabled = False
            return False
        except Exception as e:
            print(f"ERROR: Unexpected error starting log: {e}")
            self.log_enabled = False
            return False

    def stop_logging(self):
        """Stop logging GPS data."""
        if self.log_enabled and self.log_file:
            try:
                self.log_file.close()
                print(f"Logging stopped. File saved: {self.log_filename}")
            except Exception as e:
                print(f"ERROR closing log file: {e}")
            finally:
                self.log_file = None
                self.csv_writer = None
                self.log_enabled = False
                self.log_filename = None

    # --- Data Parsing (Revised for Persistence) ---
    def _parse_gps_data(self, line: str):
        """
        Parse NMEA sentence and update internal data dictionary *only* with
        fields present in the current sentence. Maintains existing values
        for fields not present in the current sentence.
        """
        if not line.startswith('$') or len(line) < 6: # Basic NMEA check
            return False

        try:
            msg = pynmea2.parse(line)

            # --- Update Shared Data (Thread Safe) ---
            with self.lock:
                # Always update the raw sentence and host timestamp
                self.current_data['raw_sentence'] = line
                self.current_data['host_timestamp'] = datetime.datetime.now()

                # Update fix timestamp if available in the message
                if hasattr(msg, 'timestamp') and msg.timestamp:
                    # Combine current date with message time if date isn't included
                    # This assumes the GPS time is roughly correct UTC
                    try:
                        current_date = datetime.date.today()
                        self.current_data['fix_timestamp'] = datetime.datetime.combine(current_date, msg.timestamp)
                    except TypeError: # Handle cases where msg.timestamp might not be just time
                         if isinstance(msg.timestamp, datetime.time):
                              # Fallback if combine fails but it's a time object
                              try:
                                   self.current_data['fix_timestamp'] = datetime.datetime.now().replace(
                                        hour=msg.timestamp.hour, minute=msg.timestamp.minute,
                                        second=msg.timestamp.second, microsecond=msg.timestamp.microsecond)
                              except Exception: pass # Ignore if time replacement fails
                         elif isinstance(msg.timestamp, datetime.datetime):
                              self.current_data['fix_timestamp'] = msg.timestamp # Use directly if it's already datetime


                # --- Process different NMEA types ---
                if isinstance(msg, pynmea2.types.talker.GGA):
                    if msg.latitude: self.current_data['raw_lat'] = msg.latitude
                    if msg.longitude: self.current_data['raw_lon'] = msg.longitude
                    if msg.altitude is not None: self.current_data['raw_alt'] = msg.altitude # Allow 0 altitude
                    if msg.gps_qual is not None: self.current_data['fix_quality'] = int(msg.gps_qual)
                    if msg.num_sats:
                        try: # num_sats in GGA usually means satellites *used* for fix
                             num_sats_int = int(msg.num_sats)
                             # Update satellites_used list length notionally if GSA hasn't provided details yet
                             if not self.current_data['satellites_used'] or len(self.current_data['satellites_used']) != num_sats_int:
                                 # Placeholder if GSA is missing/late
                                 self.current_data['satellites_used'] = list(range(1, num_sats_int + 1)) # Not real PRNs!
                        except (ValueError, TypeError): pass
                    if msg.horizontal_dil: self.current_data['hdop'] = float(msg.horizontal_dil)
                    if msg.geo_sep is not None: self.current_data['geoid_height'] = float(msg.geo_sep)

                elif isinstance(msg, pynmea2.types.talker.RMC):
                    # RMC provides key status, speed, and sometimes position
                    if msg.latitude: self.current_data['raw_lat'] = msg.latitude
                    if msg.longitude: self.current_data['raw_lon'] = msg.longitude
                    if msg.spd_over_grnd is not None: self.current_data['speed_kph'] = knots_to_kph(msg.spd_over_grnd)
                    # RMC status ('A' = Active/valid, 'V' = Void/invalid) is crucial
                    if msg.status == 'V':
                        self.current_data['fix_quality'] = 0 # Force to invalid
                        self.current_data['fix_type'] = 0 # No fix
                        self.current_data['satellites_used'] = [] # Clear used sats on void status
                        # Also clear position history and potentially reset filter on 'V' status?
                        self.position_history = []
                        # if self.kalman_filter: self.initialize_kalman_filter() # Optional: reset filter on fix loss
                    elif msg.status == 'A' and self.current_data['fix_quality'] == 0:
                        self.current_data['fix_quality'] = 1 # Assume basic GPS fix if RMC is Active and no other quality info yet

                elif isinstance(msg, pynmea2.types.talker.GSA):
                    # GSA provides fix type (2D/3D), DOPs, and *actual* satellites used
                    if msg.mode_fix_type: self.current_data['fix_type'] = int(msg.mode_fix_type)
                    if msg.pdop: self.current_data['pdop'] = float(msg.pdop)
                    if msg.hdop: self.current_data['hdop'] = float(msg.hdop) # Prefer GSA's HDOP
                    if msg.vdop: self.current_data['vdop'] = float(msg.vdop)

                    # --- Correctly handle sv_ids ---
                    # msg.sv_ids is a tuple containing the PRNs as strings
                    valid_sv_ids = []
                    if hasattr(msg, 'sv_ids'):
                         valid_sv_ids = [int(svid) for svid in msg.sv_ids if svid] # Convert valid IDs to int
                    self.current_data['satellites_used'] = valid_sv_ids

                    # Infer GNSS systems from the PRNs used
                    current_systems: Set[str] = set()
                    for prn in valid_sv_ids:
                        if 1 <= prn <= 32: current_systems.add("GPS")
                        elif 33 <= prn <= 64: current_systems.add("SBAS") # WAAS/EGNOS etc. (Can refine by Talker ID: GP vs GA etc.)
                        elif 65 <= prn <= 96: current_systems.add("GLONASS")
                        # Add ranges for Galileo, BeiDou, QZSS etc. if needed
                        # elif 201 <= prn <= 235: current_systems.add("Galileo")
                        # elif 301 <= prn <= 336: current_systems.add("BeiDou")
                    self.current_data['gnss_systems_used'] = current_systems

                elif isinstance(msg, pynmea2.types.talker.GSV):
                    # GSV provides detailed info about satellites in view
                    try:
                        num_sv_in_view = int(msg.num_sv_in_view) if msg.num_sv_in_view else 0
                        # Update total count only if it increases (GSV messages might interleave)
                        self.current_data['satellites_in_view'] = max(num_sv_in_view, self.current_data['satellites_in_view'])

                        msg_num = int(msg.msg_num) if msg.msg_num else 1
                        # Clear previous satellite details *only* when receiving the first message of a sequence
                        if msg_num == 1:
                           self.current_data['satellite_data'] = []

                        sats_in_this_msg = []
                        for i in range(1, 5):  # GSV has fields for up to 4 satellites
                            prn_str = getattr(msg, f'sv_prn_num_{i}', None)
                            if prn_str: # Check if the PRN field exists and is not empty
                                try:
                                    sats_in_this_msg.append({
                                        'prn': int(prn_str),
                                        'elevation': int(getattr(msg, f'elevation_{i}', 0) or 0),
                                        'azimuth': int(getattr(msg, f'azimuth_{i}', 0) or 0),
                                        'snr': int(getattr(msg, f'snr_{i}', 0) or 0) # Treat empty SNR as 0
                                    })
                                except (ValueError, TypeError) as sat_parse_err:
                                     print(f"WARN: Could not parse satellite detail in GSV: {sat_parse_err} - PRN:'{prn_str}'")

                        # Append new sats, then de-duplicate based on PRN, keeping the latest entry
                        existing_prns = {sat['prn']: sat for sat in self.current_data['satellite_data']}
                        for sat in sats_in_this_msg:
                            existing_prns[sat['prn']] = sat # Add or overwrite with latest info
                        self.current_data['satellite_data'] = list(existing_prns.values())

                    except (ValueError, TypeError, AttributeError) as gsv_err:
                         print(f"WARN: Error processing GSV message: {gsv_err} - Line: {line}")


                elif isinstance(msg, pynmea2.types.talker.VTG):
                    # VTG provides Course and Speed Over Ground
                    if msg.spd_over_grnd_kmph is not None:
                         self.current_data['speed_kph'] = float(msg.spd_over_grnd_kmph)


                # --- Post-processing: Smoothing and Accuracy Estimation ---
                # Apply smoothing based on the *raw* position updated by GGA/RMC
                proc_lat, proc_lon, proc_alt = self.current_data['raw_lat'], self.current_data['raw_lon'], self.current_data['raw_alt']

                if self.smoothing_method == SMOOTHING_AVG:
                    self._add_to_position_history(proc_lat, proc_lon, proc_alt) # Add current raw pos to history
                    proc_lat, proc_lon, proc_alt = self._get_averaged_position() # Get smoothed pos
                elif self.smoothing_method == SMOOTHING_KALMAN:
                    proc_lat, proc_lon, proc_alt = self._apply_kalman_filter(proc_lat, proc_lon, proc_alt)

                # Update the final lat/lon/alt with the processed/smoothed values
                self.current_data['lat'] = proc_lat
                self.current_data['lon'] = proc_lon
                self.current_data['altitude'] = proc_alt

                # --- Estimate Accuracy (Heuristic) ---
                hdop = self.current_data['hdop']
                sats_used_count = len(self.current_data.get('satellites_used', []))
                if hdop > 0 and sats_used_count >= 4:
                    base_precision = 2.5 # meters (typical good conditions)
                    accuracy = hdop * base_precision
                    if sats_used_count < 5: accuracy *= 1.5
                    elif sats_used_count > 8: accuracy *= 0.8
                    self.current_data['accuracy_estimate'] = round(accuracy, 2)
                elif self.current_data['fix_quality'] == 0:
                     self.current_data['accuracy_estimate'] = 0.0 # No accuracy if no fix
                # else: keep previous estimate if fix quality > 0 but hdop/sats invalid? Or set to 0? Let's keep previous for now.


                # --- Log data if enabled ---
                if self.log_enabled and self.csv_writer:
                    try:
                        log_row = [
                            self.current_data['host_timestamp'].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] if self.current_data['host_timestamp'] else '',
                            self.current_data['fix_timestamp'].strftime("%H:%M:%S.%f")[:-3] if self.current_data['fix_timestamp'] else '',
                            f"{self.current_data['lat']:.7f}",
                            f"{self.current_data['lon']:.7f}",
                            f"{self.current_data['altitude']:.2f}",
                            f"{self.current_data['speed_kph']:.2f}",
                            self.current_data['satellites_in_view'],
                            len(self.current_data.get('satellites_used', [])),
                            FIX_QUALITY_MAP.get(self.current_data['fix_quality'], self.current_data['fix_quality']),
                            f"{self.current_data['hdop']:.2f}",
                            f"{self.current_data['pdop']:.2f}",
                            f"{self.current_data['vdop']:.2f}",
                            FIX_TYPE_MAP.get(self.current_data['fix_type'], self.current_data['fix_type']),
                            f"{self.current_data['accuracy_estimate']:.2f}",
                            f"{self.current_data['geoid_height']:.2f}",
                            ",".join(sorted(list(self.current_data['gnss_systems_used']))),
                            self.current_data['raw_sentence'] # Log the specific sentence that triggered this log entry
                        ]
                        self.csv_writer.writerow(log_row)
                        # self.log_file.flush() # Flush less often?
                    except Exception as log_err:
                        print(f"ERROR writing to log file: {log_err}")

            return True # Successfully parsed and processed

        except pynmea2.ParseError as e:
            # print(f"WARN: Could not parse NMEA sentence: {line} - {e}")
            return False
        except ValueError as e:
            # This might catch errors during int/float conversions within the parsing logic
            print(f"WARN: Value error processing NMEA data: {line} - {e}")
            return False
        except AttributeError as e:
             # Should be less common now with direct attribute checks, but catch just in case
             print(f"WARN: Attribute error processing NMEA data: {line} - {e}")
             return False
        except Exception as e:
            print(f"ERROR: Unexpected error processing NMEA data: {line} - {e}")
            # import traceback
            # traceback.print_exc() # More detailed error for debugging
            return False

    # --- Reading Thread ---
    def _read_loop(self):
        """Background thread task to continuously read from serial port."""
        print("Read thread started.")
        while self.running.is_set():
            line = None # Ensure line is reset each loop
            try:
                if self.ser and self.ser.is_open:
                    # Read one line, decode safely
                    try:
                        # Set a reasonable timeout on readline itself if needed,
                        # but the serial object timeout should handle most cases.
                        raw_line = self.ser.readline()
                        if raw_line:
                             line = raw_line.decode('ascii', errors='replace').strip()
                        else:
                             # Timeout occurred on readline, just loop again
                             time.sleep(0.01) # Small sleep on timeout
                             continue

                    except serial.SerialException as serial_err:
                         print(f"ERROR: Serial read error: {serial_err}. Stopping reader.")
                         self.running.clear() # Signal stop
                         # GUI should handle reconnect attempt based on running state
                         break # Exit read loop
                    except UnicodeDecodeError:
                         # print(f"WARN: Could not decode bytes: {raw_line}")
                         continue # Skip undecodable data
                    except Exception as read_err:
                        print(f"ERROR: Unexpected read error: {read_err}")
                        time.sleep(0.1) # Avoid busy-looping
                        continue

                    if line:
                        self._parse_gps_data(line)
                    # else: # No line read (timeout) - handled by continue above

                else:
                    # Serial port not open/available, wait before checking again
                    # This state should ideally be handled by the main thread trying to connect/reconnect
                    print("Read loop: Serial port not open.")
                    time.sleep(1.0) # Wait longer if port closed unexpectedly


            except Exception as loop_err:
                print(f"ERROR: Unhandled exception in read loop: {loop_err}")
                import traceback
                traceback.print_exc()
                time.sleep(1) # Avoid rapid error loops

        print("Read thread finished.")
        # Ensure log file is flushed and closed properly when thread stops
        self.stop_logging()


    # --- Control Methods ---
    def start(self) -> bool:
        """Start the GPS reading thread."""
        if self.running.is_set():
            print("Reader already running.")
            return True
        # Reset data to defaults before starting
        with self.lock:
             self.current_data = self._get_default_data()
             self.position_history = []
             if self.kalman_filter: self.initialize_kalman_filter() # Reset filter state on start

        if not self.connect():
             return False

        self.running.set()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        return True

    def stop(self):
        """Stop the GPS reading thread and disconnect."""
        if not self.running.is_set() and not (self.read_thread and self.read_thread.is_alive()):
             print("Reader already stopped.")
             # Ensure disconnect and logging stop are called even if already stopped logically
             self.disconnect()
             self.stop_logging()
             return

        print("Stopping GPS reader...")
        self.running.clear() # Signal thread to stop

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0) # Wait for thread to finish
            if self.read_thread.is_alive():
                 print("WARN: Read thread did not terminate gracefully.")

        self.disconnect()
        # stop_logging() is called automatically when read thread loop finishes
        # self.stop_logging() # Ensure it's called again here just in case loop exited abruptly
        self.read_thread = None
        print("GPS reader stopped.")

    def get_current_data(self) -> Dict[str, Any]:
        """Return a copy of the latest GPS data (thread-safe)."""
        with self.lock:
            return self.current_data.copy()

    def set_smoothing(self, method: str, history_size: Optional[int] = None):
        """Set the position smoothing method."""
        with self.lock:
            if method in SMOOTHING_METHODS:
                # Clear history/reset filter when changing method
                self.position_history = []
                if self.kalman_filter and method != SMOOTHING_KALMAN:
                     # Reset filter if switching away from it, so it starts fresh if switched back
                     self.initialize_kalman_filter()

                self.smoothing_method = method
                print(f"Smoothing method set to: {method}")

                if method == SMOOTHING_AVG and history_size is not None:
                    self.max_history_size = max(1, history_size)
                    print(f"Moving average history size set to: {self.max_history_size}")
                elif method == SMOOTHING_KALMAN:
                    if NUMPY_AVAILABLE:
                        self.initialize_kalman_filter() # Reset filter state when selected
                    else:
                        print("WARN: Cannot use Kalman filter, NumPy not installed.")
                        self.smoothing_method = SMOOTHING_NONE # Fallback
            else:
                print(f"WARN: Invalid smoothing method '{method}' requested.")

# --- GUI Application Class (Largely unchanged, relies on improved reader data) ---
class GPSApp:
    """Tkinter GUI for displaying GPS data."""

    def __init__(self, root: tk.Tk):
        """Initialize the application."""
        self.root = root
        self.root.title("Enhanced GPS Data Viewer")
        self.root.minsize(750, 600)

        self.gps_reader: Optional[GPSReader] = None
        self.gui_update_after_id: Optional[str] = None

        # --- Satellite Window ---
        self.sat_window: Optional[tk.Toplevel] = None
        self.sat_tree: Optional[ttk.Treeview] = None
        # Add references for summary labels in sat window
        self.summary_gnss: Optional[ttk.Label] = None
        self.summary_sats: Optional[ttk.Label] = None
        self.summary_dop: Optional[ttk.Label] = None
        self.summary_accuracy: Optional[ttk.Label] = None


        # --- UI Variables ---
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD_RATE))
        self.smoothing_var = tk.StringVar(value=SMOOTHING_NONE)
        self.history_var = tk.StringVar(value=str(MAX_POSITION_HISTORY))
        self.status_var = tk.StringVar(value="Not connected")

        self._create_styles()
        self._create_widgets()
        self.refresh_ports() # Populate ports on startup
        self._update_ui_states() # Initial UI state

    def _create_styles(self):
        """Configure ttk styles."""
        style = ttk.Style()
        try:
             # Try themed styles first
             if platform.system() == "Windows": style.theme_use('vista')
             elif platform.system() == "Darwin": style.theme_use('aqua')
             else : style.theme_use('clam')
        except tk.TclError:
             style.theme_use('default') # Fallback

        style.configure("TLabel", padding=2)
        style.configure("TButton", padding=5)
        style.configure("Toolbutton.TButton", padding=2) # Smaller padding for refresh
        style.configure("TCombobox", padding=2)
        style.configure("TLabelframe.Label", padding=(5, 2))
        style.configure("Treeview.Heading", font=('Helvetica', 10, 'bold'))
        # Add tag for satellites used in fix
        style.configure("Used.Treeview", foreground="green") # Used in fix
        style.configure("NotUsed.Treeview", foreground="gray") # Not used


    def _create_widgets(self):
        """Create all GUI elements."""
        # --- Main Panes ---
        main_pane = tk.PanedWindow(self.root, orient=tk.VERTICAL, sashrelief=tk.RAISED)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Top Frame (Config + Data)
        top_frame = ttk.Frame(main_pane, padding=5)
        main_pane.add(top_frame, stretch="always")

        # Bottom Frame (NMEA Log)
        bottom_frame = ttk.Frame(main_pane, padding=5)
        main_pane.add(bottom_frame, stretch="never")

        # Configure top frame grid
        top_frame.columnconfigure(1, weight=1) # Allow data labels to expand

        # --- Connection Frame ---
        conn_frame = ttk.LabelFrame(top_frame, text="Connection")
        conn_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        conn_frame.columnconfigure(6, weight=1) # Allow buttons to space out

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.refresh_btn = ttk.Button(conn_frame, text="↺", width=3, command=self.refresh_ports, style="Toolbutton.TButton")
        self.refresh_btn.grid(row=0, column=2, padx=(0, 5), pady=5)

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, values=BAUD_RATES, width=8, state="readonly")
        self.baud_combo.grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        self.log_btn = ttk.Button(conn_frame, text="Start Logging", command=self.toggle_logging)
        self.log_btn.grid(row=0, column=6, padx=5, pady=5)

        # --- Accuracy Settings Frame ---
        acc_frame = ttk.LabelFrame(top_frame, text="Processing")
        acc_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        ttk.Label(acc_frame, text="Smoothing:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        smoothing_values = [SMOOTHING_NONE, SMOOTHING_AVG]
        if NUMPY_AVAILABLE: smoothing_values.append(SMOOTHING_KALMAN)
        self.smoothing_combo = ttk.Combobox(acc_frame, textvariable=self.smoothing_var, values=smoothing_values, width=15, state="readonly")
        self.smoothing_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.smoothing_combo.bind("<<ComboboxSelected>>", self.apply_accuracy_settings)

        self.history_label = ttk.Label(acc_frame, text="Avg History:")
        self.history_label.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.history_spin = ttk.Spinbox(acc_frame, from_=2, to=50, width=5, textvariable=self.history_var, command=self.apply_accuracy_settings)
        self.history_spin.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)

        # --- Data Display Frame ---
        data_frame = ttk.LabelFrame(top_frame, text="GPS Data")
        data_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        data_frame.columnconfigure(1, weight=1) # Let data values expand
        top_frame.rowconfigure(2, weight=1) # Let data frame expand vertically

        # GPS data labels
        self.data_labels: Dict[str, ttk.Label] = {}
        # Match field names to keys in GPSReader.current_data
        data_fields = [
            ("lat", "Latitude:", 0), ("lon", "Longitude:", 1),
            ("altitude", "Altitude:", 2), ("geoid_height", "Geoid Height:", 3),
            ("speed_kph", "Speed:", 4),
            ("fix_quality", "Fix Quality:", 5), ("fix_type", "Fix Type:", 6),
            ("satellites_view_used", "Sats View/Used:", 7), # Combined label placeholder
            ("accuracy_estimate", "Est. Accuracy:", 8),
            ("hdop", "HDOP:", 9), ("pdop", "PDOP:", 10), ("vdop", "VDOP:", 11),
            ("gnss_systems_used", "GNSS Systems:", 12),
            ("fix_timestamp", "Device Time:", 13), ("host_timestamp", "Host Time:", 14),
        ]

        row_offset = 0
        for field, label_text, row in data_fields:
            ttk.Label(data_frame, text=label_text).grid(row=row_offset + row, column=0, padx=5, pady=2, sticky=tk.W)
            # Use a slightly sunken relief to make fields clearer
            self.data_labels[field] = ttk.Label(data_frame, text="--", anchor=tk.W, relief=tk.SUNKEN, padding=2, width=25) # Min width
            self.data_labels[field].grid(row=row_offset + row, column=1, padx=5, pady=2, sticky="ew")

        # --- Action Buttons Frame ---
        action_frame = ttk.Frame(top_frame, padding=5)
        action_frame.grid(row=2, column=1, padx=5, pady=5, sticky="ne") # Align to top-right of data area

        self.map_btn = ttk.Button(action_frame, text="View on Map", command=self.open_maps)
        self.map_btn.pack(pady=3, fill=tk.X)

        self.sat_btn = ttk.Button(action_frame, text="Satellite Info", command=self.toggle_satellite_window)
        self.sat_btn.pack(pady=3, fill=tk.X)

        # --- NMEA Log Display ---
        nmea_frame = ttk.LabelFrame(bottom_frame, text="Raw NMEA Sentences")
        nmea_frame.pack(fill=tk.BOTH, expand=True)

        nmea_scrollbar = ttk.Scrollbar(nmea_frame, orient=tk.VERTICAL)
        self.nmea_text = tk.Text(nmea_frame, height=6, width=80, wrap=tk.NONE,
                                 yscrollcommand=nmea_scrollbar.set, state=tk.DISABLED,
                                 font=("Courier", 9), borderwidth=1, relief=tk.SOLID)
        nmea_scrollbar.config(command=self.nmea_text.yview)

        nmea_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.nmea_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # --- Status Bar ---
        self.status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W, padding=2)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    # --- UI Update and Control Methods ---

    def _update_ui_states(self, connected: bool = False, logging: bool = False, has_fix: bool = False):
        """Enable/disable widgets based on connection and data state."""
        conn_state = tk.NORMAL if not connected else tk.DISABLED
        log_state = tk.NORMAL if connected else tk.DISABLED
        map_fix_state = tk.NORMAL if connected and has_fix else tk.DISABLED
        sat_state = tk.NORMAL if connected else tk.DISABLED # Enable Sat window even without fix

        self.port_combo.config(state=conn_state)
        self.baud_combo.config(state=conn_state)
        self.refresh_btn.config(state=conn_state)
        self.connect_btn.config(text="Disconnect" if connected else "Connect")
        # Keep connect button enabled unless actively trying to connect/disconnect
        # self.connect_btn.config(state=tk.NORMAL)

        self.log_btn.config(state=log_state)
        self.log_btn.config(text="Stop Logging" if logging else "Start Logging")

        self.map_btn.config(state=map_fix_state)
        self.sat_btn.config(state=sat_state)

        self.smoothing_combo.config(state="readonly") # Always selectable if available
        self.history_spin.config(state="readonly" if self.smoothing_var.get() == SMOOTHING_AVG else tk.DISABLED)
        self.history_label.config(state=tk.NORMAL if self.smoothing_var.get() == SMOOTHING_AVG else tk.DISABLED)


    def _get_available_ports(self) -> List[Tuple[str, str]]:
        """Get a list of available serial ports with descriptions."""
        ports = serial.tools.list_ports.comports()
        # Filter out ports without descriptions on some platforms if needed
        # ports = [p for p in ports if p.description != 'n/a']
        port_list = []
        for port in sorted(ports, key=lambda p: p.device):
            desc = f"{port.device}: {port.description}"
            port_list.append((port.device, desc))
        return port_list

    def refresh_ports(self):
        """Refresh the list of available serial ports in the Combobox."""
        available_ports = self._get_available_ports()
        port_descriptions = [desc for _, desc in available_ports]
        self.port_combo['values'] = port_descriptions

        current_selection = self.port_var.get()
        current_device = current_selection.split(':')[0].strip() if ':' in current_selection else None

        if not available_ports:
            self.port_var.set("")
            self.status_var.set("No serial ports found.")
        elif current_device and current_device in [dev for dev, _ in available_ports]:
             # Keep current selection if still available and valid
             current_desc = next((desc for dev, desc in available_ports if dev == current_device), None)
             if current_desc: self.port_var.set(current_desc)
             else: # Current device exists but description changed? Select first.
                 self.port_var.set(available_ports[0][1])
        elif available_ports:
             # Default to the first port in the sorted list
             self.port_var.set(available_ports[0][1])

        status_msg = f"Found {len(available_ports)} ports." if available_ports else "No serial ports found."
        if len(available_ports) == 1: status_msg = f"Found 1 port: {available_ports[0][0]}"
        self.status_var.set(status_msg)


    def toggle_connection(self):
        """Connect to or disconnect from the GPS reader."""
        if self.gps_reader and self.gps_reader.running.is_set():
            # --- Disconnect ---
            self.status_var.set("Disconnecting...")
            self.connect_btn.config(state=tk.DISABLED) # Disable while disconnecting
            self.root.update_idletasks()
            if self.gui_update_after_id:
                self.root.after_cancel(self.gui_update_after_id)
                self.gui_update_after_id = None
            self.gps_reader.stop() # This now handles logging stop too
            self.gps_reader = None
            self._clear_data_labels()
            self.nmea_text.config(state=tk.NORMAL)
            self.nmea_text.delete(1.0, tk.END)
            self.nmea_text.config(state=tk.DISABLED)
            self.status_var.set("Disconnected")
            self.connect_btn.config(state=tk.NORMAL) # Re-enable
            self._update_ui_states(connected=False)
            if self.sat_window:
                 self.close_satellite_window()

        else:
            # --- Connect ---
            port_desc = self.port_var.get()
            if not port_desc:
                messagebox.showerror("Connection Error", "Please select a serial port.")
                return
            port = port_desc.split(':')[0].strip()

            try:
                baud = int(self.baud_var.get())
            except ValueError:
                messagebox.showerror("Connection Error", "Invalid baud rate selected.")
                return

            self.status_var.set(f"Connecting to {port}...")
            self.connect_btn.config(state=tk.DISABLED) # Disable while connecting
            self._update_ui_states(connected=False) # Disable other controls
            self.root.update_idletasks()

            self.gps_reader = GPSReader(port, baud)
            # Apply initial smoothing setting from UI before starting reader
            self.apply_accuracy_settings(initial=True)

            if self.gps_reader.start():
                self.status_var.set(f"Connected to {port} at {baud} baud")
                self.connect_btn.config(state=tk.NORMAL) # Re-enable
                self._update_ui_states(connected=True)
                self.gui_update_after_id = self.root.after(UPDATE_INTERVAL_MS, self.update_gui)
            else:
                self.status_var.set(f"Failed to connect to {port}")
                messagebox.showerror("Connection Failed", f"Could not connect to {port}.\nCheck port, permissions, and if device is in use.")
                self.gps_reader = None
                self.connect_btn.config(state=tk.NORMAL) # Re-enable
                self._update_ui_states(connected=False)


    def toggle_logging(self):
        """Start or stop logging GPS data to a CSV file."""
        if not self.gps_reader:
            messagebox.showwarning("Logging", "Connect to GPS device first.")
            return

        if self.gps_reader.log_enabled:
            self.gps_reader.stop_logging()
            self.status_var.set("Logging stopped.")
            self._update_ui_states(connected=True, logging=False, has_fix=self._has_fix())
        else:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"gps_log_{timestamp}.csv"
            filename = filedialog.asksaveasfilename(
                title="Save GPS Log As",
                initialfile=default_filename,
                defaultextension=".csv",
                filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
            )
            if filename:
                if self.gps_reader.start_logging(filename):
                    self.status_var.set(f"Logging to {os.path.basename(filename)}")
                    self._update_ui_states(connected=True, logging=True, has_fix=self._has_fix())
                else:
                    messagebox.showerror("Logging Error", "Could not start logging.\nCheck file permissions or path.")
                    self.status_var.set("Logging failed to start.")
                    self._update_ui_states(connected=True, logging=False, has_fix=self._has_fix())
            else:
                self.status_var.set("Logging cancelled.")


    def apply_accuracy_settings(self, event=None, initial=False):
        """Apply selected smoothing/accuracy settings."""
        method = self.smoothing_var.get()
        history_size = None

        # Validate history size only if moving average is selected
        if method == SMOOTHING_AVG:
            try:
                history_size = int(self.history_var.get())
                if history_size < 1: raise ValueError("History size must be positive")
            except ValueError:
                 if not initial: # Don't show error on initial setup
                     messagebox.showerror("Settings Error", "Invalid history size. Please enter a positive integer (e.g., 10).")
                 # Optionally revert UI or set a default?
                 self.history_var.set(str(MAX_POSITION_HISTORY)) # Revert to default
                 history_size = MAX_POSITION_HISTORY # Use default

        # Apply to reader if it exists
        if self.gps_reader:
             self.gps_reader.set_smoothing(method, history_size)
             status_msg = f"Smoothing: {method}"
             if method == SMOOTHING_AVG:
                 status_msg += f", History: {self.gps_reader.max_history_size}"
             self.status_var.set(status_msg)

        # Update UI state (e.g., enable/disable history spinbox)
        # Need to know connection/logging state too
        is_connected = self.gps_reader is not None and self.gps_reader.running.is_set()
        is_logging = self.gps_reader.log_enabled if self.gps_reader else False
        has_fix = self._has_fix()
        self._update_ui_states(connected=is_connected, logging=is_logging, has_fix=has_fix)


    def _clear_data_labels(self):
        """Reset all data display labels to '--'."""
        for field in self.data_labels:
             # Handle the combined field specifically
             if field == "satellites_view_used":
                 self.data_labels[field].config(text="-- / --")
             else:
                 self.data_labels[field].config(text="--")


    def _has_fix(self) -> bool:
        """Check if the current GPS data indicates a valid fix."""
        if not self.gps_reader: return False
        # Use a thread-safe copy of the data
        data = self.gps_reader.get_current_data()
        # Consider quality > 0 and non-zero lat/lon as a basic fix indicator
        # Also check fix_type (2=2D, 3=3D are minimal fixes)
        quality = data.get('fix_quality', 0)
        fix_type = data.get('fix_type', 0)
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        # RMC status 'V' should have already set quality to 0
        return quality > 0 and fix_type >= 2 and lat != 0.0 and lon != 0.0

    def update_gui(self):
        """Periodically fetch data from GPSReader and update GUI elements."""
        if not self.gps_reader or not self.gps_reader.running.is_set():
             if self.gui_update_after_id:
                 self.root.after_cancel(self.gui_update_after_id)
                 self.gui_update_after_id = None
             # Optionally reset status bar if reader stops unexpectedly
             # self.status_var.set("Reader stopped.")
             # self._update_ui_states(connected=False)
             return

        data = self.gps_reader.get_current_data()
        has_fix = self._has_fix() # Check fix status based on latest data

        # --- Update Data Labels ---
        for field, label_widget in self.data_labels.items():
            value = data.get(field)
            text = "--" # Default text

            try:
                # Handle each field specifically for proper formatting
                if field == "lat": text = degrees_to_dms(value)
                elif field == "lon": text = degrees_to_dms(value)
                elif field in ["altitude", "geoid_height"]:
                    text = f"{float(value):.2f} m" if value is not None else "--"
                elif field == "speed_kph":
                    text = f"{float(value):.2f} km/h" if value is not None else "--"
                elif field == "fix_quality":
                    text = FIX_QUALITY_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "fix_type":
                    text = FIX_TYPE_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "satellites_view_used": # Handle combined field
                    sats_view = data.get('satellites_in_view', 0)
                    sats_used_count = len(data.get('satellites_used', []))
                    text = f"{sats_view} / {sats_used_count}"
                elif field in ["hdop", "pdop", "vdop"]:
                     text = f"{float(value):.2f}" if value is not None and value > 0 else "--"
                elif field == "accuracy_estimate":
                     text = f"~ {float(value):.2f} m" if value is not None and value > 0 else "--"
                elif field == "gnss_systems_used":
                     # Value is expected to be a set
                     text = ", ".join(sorted(list(value))) if value else "None"
                elif field == "fix_timestamp":
                    text = value.strftime("%H:%M:%S.%f")[:-3] if isinstance(value, (datetime.datetime, datetime.time)) else "--"
                elif field == "host_timestamp":
                    text = value.strftime("%Y-%m-%d %H:%M:%S") if isinstance(value, datetime.datetime) else "--"
                # else: # No default formatting needed for other fields currently

                label_widget.config(text=text)
            except (ValueError, TypeError, AttributeError) as fmt_err:
                 # print(f"WARN: Error formatting field '{field}' with value '{value}': {fmt_err}")
                 label_widget.config(text="Error") # Indicate formatting issue in GUI

        # --- Update NMEA Text Area ---
        raw_sentence = data.get('raw_sentence', '')
        if raw_sentence and raw_sentence != getattr(self, '_last_nmea_displayed', ''): # Avoid re-inserting same sentence
            self.nmea_text.config(state=tk.NORMAL)
            self.nmea_text.insert(tk.END, raw_sentence + '\n')
            self.nmea_text.see(tk.END) # Auto-scroll
            # Limit buffer size
            max_lines = 200
            num_lines = int(self.nmea_text.index('end-1c').split('.')[0])
            if num_lines > max_lines:
                self.nmea_text.delete('1.0', f'{num_lines - max_lines}.0')
            self.nmea_text.config(state=tk.DISABLED)
            self._last_nmea_displayed = raw_sentence # Store last displayed sentence


        # --- Update Status Bar ---
        if has_fix:
            self.status_bar.config(foreground="black") # Use default color for fix
            fix_desc = FIX_QUALITY_MAP.get(data.get('fix_quality', 0), "Fix")
            sats_view = data.get('satellites_in_view', 0)
            sats_used_count = len(data.get('satellites_used', []))
            hdop = data.get('hdop', 0.0)
            self.status_var.set(f"Status: {fix_desc} | Sats: {sats_view}/{sats_used_count} | HDOP: {hdop:.2f}")
        else:
            # Indicate lack of fix more clearly
            self.status_bar.config(foreground="red")
            # Check if quality is 0 but type indicates some activity
            fix_type = data.get('fix_type', 0)
            status_text = "Status: No Fix"
            if fix_type == 1: status_text = "Status: No Fix (Searching...)"
            elif data.get('fix_quality', 0) > 0 and fix_type < 2: status_text = "Status: Acquiring 2D/3D Fix..." # e.g., Quality=1, Type=1
            self.status_var.set(status_text)


        # Update button states based on fix
        self._update_ui_states(connected=True, logging=self.gps_reader.log_enabled, has_fix=has_fix)

        # --- Update Satellite Window (if open) ---
        if self.sat_window and self.sat_window.winfo_exists():
            self.update_satellite_window() # This now reads data internally

        # --- Schedule next update ---
        self.gui_update_after_id = self.root.after(UPDATE_INTERVAL_MS, self.update_gui)


    def open_maps(self):
        """Open the current coordinates in the default web browser using Google Maps."""
        if not self.gps_reader: return
        data = self.gps_reader.get_current_data()
        lat = data.get('lat', 0.0) # Use the processed/smoothed lat/lon
        lon = data.get('lon', 0.0)

        if lat != 0.0 and lon != 0.0:
            maps_url = f"https://www.google.com/maps?q={lat:.7f},{lon:.7f}&z=16" # More precision in URL
            try:
                webbrowser.open(maps_url, new=2) # Try to open in a new tab
                self.status_var.set(f"Opening map for {lat:.5f}, {lon:.5f}")
            except Exception as e:
                 messagebox.showerror("Map Error", f"Could not open web browser: {e}")
                 self.status_var.set("Failed to open map.")
        else:
            self.status_var.set("No valid coordinates to map.")
            messagebox.showwarning("Map Error", "No valid GPS coordinates available to display on map.")


    # --- Satellite Window Methods ---

    def toggle_satellite_window(self):
        """Open or bring the satellite info window to the front."""
        if self.sat_window and self.sat_window.winfo_exists():
            try:
                self.sat_window.lift()
                self.sat_window.focus_force() # Try to grab focus
            except tk.TclError: # Handle cases where window might be closing
                 self.create_satellite_window()
        else:
            self.create_satellite_window()

    def create_satellite_window(self):
        """Create the Toplevel window for satellite information."""
        if not self.gps_reader:
            messagebox.showinfo("Info", "Connect to GPS device first.")
            return
        if self.sat_window and self.sat_window.winfo_exists():
             return # Avoid creating multiple windows

        self.sat_window = tk.Toplevel(self.root)
        self.sat_window.title("Satellite Information")
        self.sat_window.geometry("650x450")
        self.sat_window.minsize(500, 300)
        self.sat_window.protocol("WM_DELETE_WINDOW", self.close_satellite_window)

        # Frame for Treeview and Scrollbar
        tree_frame = ttk.Frame(self.sat_window, padding=5)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Treeview widget
        columns = ("prn", "gnss", "elev", "azim", "snr", "used")
        self.sat_tree = ttk.Treeview(tree_frame, columns=columns, show="headings")

        # Define headings and column properties
        self.sat_tree.heading("prn", text="PRN", command=lambda: self.sort_sat_tree("prn", False))
        self.sat_tree.column("prn", width=50, anchor=tk.CENTER, stretch=False)
        self.sat_tree.heading("gnss", text="System", command=lambda: self.sort_sat_tree("gnss", False))
        self.sat_tree.column("gnss", width=70, anchor=tk.W, stretch=False)
        self.sat_tree.heading("elev", text="Elevation (°)", command=lambda: self.sort_sat_tree("elev", True))
        self.sat_tree.column("elev", width=80, anchor=tk.CENTER, stretch=False)
        self.sat_tree.heading("azim", text="Azimuth (°)", command=lambda: self.sort_sat_tree("azim", True))
        self.sat_tree.column("azim", width=80, anchor=tk.CENTER, stretch=False)
        self.sat_tree.heading("snr", text="SNR (dB)", command=lambda: self.sort_sat_tree("snr", True))
        self.sat_tree.column("snr", width=60, anchor=tk.CENTER, stretch=False)
        self.sat_tree.heading("used", text="Used in Fix", command=lambda: self.sort_sat_tree("used", False))
        self.sat_tree.column("used", width=80, anchor=tk.CENTER, stretch=False)

        # Scrollbar
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.sat_tree.yview)
        self.sat_tree.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.sat_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Frame for Summary Info below tree
        summary_frame = ttk.LabelFrame(self.sat_window, text="Summary", padding=5)
        summary_frame.pack(fill=tk.X, padx=5, pady=5)
        summary_frame.columnconfigure(1, weight=1)
        summary_frame.columnconfigure(3, weight=1)

        # Add summary labels
        ttk.Label(summary_frame, text="Active GNSS:").grid(row=0, column=0, sticky=tk.W, padx=5)
        self.summary_gnss = ttk.Label(summary_frame, text="--", anchor=tk.W)
        self.summary_gnss.grid(row=0, column=1, sticky=tk.EW, padx=5)

        ttk.Label(summary_frame, text="Sats View/Used:").grid(row=1, column=0, sticky=tk.W, padx=5)
        self.summary_sats = ttk.Label(summary_frame, text="-- / --", anchor=tk.W)
        self.summary_sats.grid(row=1, column=1, sticky=tk.EW, padx=5)

        ttk.Label(summary_frame, text="HDOP / PDOP / VDOP:").grid(row=0, column=2, sticky=tk.W, padx=5)
        self.summary_dop = ttk.Label(summary_frame, text="-- / -- / --", anchor=tk.W)
        self.summary_dop.grid(row=0, column=3, sticky=tk.EW, padx=5)

        ttk.Label(summary_frame, text="Est. Accuracy:").grid(row=1, column=2, sticky=tk.W, padx=5)
        self.summary_accuracy = ttk.Label(summary_frame, text="--", anchor=tk.W)
        self.summary_accuracy.grid(row=1, column=3, sticky=tk.EW, padx=5)

        # Initial population
        self.update_satellite_window()

    def sort_sat_tree(self, col: str, reverse: bool):
        """Sort the satellite treeview by a column."""
        if not self.sat_tree: return
        # Get data from treeview column
        data = [(self.sat_tree.set(item, col), item) for item in self.sat_tree.get_children('')]

        # Convert to numeric for sorting where appropriate, handle '--' or errors
        def safe_num_convert(x):
            try: return int(x)
            except (ValueError, TypeError): return -1 # Sort errors/non-numeric low

        if col in ["prn", "elev", "azim", "snr"]:
             data.sort(key=lambda t: safe_num_convert(t[0]), reverse=reverse)
        else: # Sort alphabetically for gnss, used
             data.sort(key=lambda t: str(t[0]).lower(), reverse=reverse)


        for index, (val, item) in enumerate(data):
            self.sat_tree.move(item, '', index)

        # Toggle sort direction for next click
        self.sat_tree.heading(col, command=lambda: self.sort_sat_tree(col, not reverse))


    def update_satellite_window(self):
        """Update the satellite treeview and summary labels with current data."""
        # Check if window and treeview exist and are valid
        if not hasattr(self, 'sat_window') or not self.sat_window or \
           not self.sat_window.winfo_exists() or \
           not hasattr(self, 'sat_tree') or not self.sat_tree:
            return
        if not self.gps_reader:
             return

        data = self.gps_reader.get_current_data()
        sat_data_list = data.get('satellite_data', []) # List of dicts
        sats_used_set = set(data.get('satellites_used', [])) # Set of PRNs

        # --- Update Treeview ---
        # Store current selection and scroll position
        selected_items = self.sat_tree.selection()
        scroll_pos = self.sat_tree.yview()

        # Keep track of items currently in the treeview by PRN
        tree_items = {self.sat_tree.set(item, "prn"): item for item in self.sat_tree.get_children('')}
        updated_prns = set()

        for sat_info in sat_data_list:
            prn = sat_info.get('prn')
            if prn is None: continue # Skip if PRN is missing

            prn_str = str(prn)
            updated_prns.add(prn_str) # Mark this PRN as present in the data

            elev = sat_info.get('elevation', '--')
            azim = sat_info.get('azimuth', '--')
            snr = sat_info.get('snr', '--')
            used = prn in sats_used_set
            used_text = "Yes" if used else "No"
            tag = 'Used' if used else 'NotUsed'

            # Determine GNSS System
            gnss = "Unknown"
            if isinstance(prn, int):
                if 1 <= prn <= 32: gnss = "GPS"
                elif 33 <= prn <= 64: gnss = "SBAS"
                elif 65 <= prn <= 96: gnss = "GLONASS"
                elif 120 <= prn <= 140: gnss = "SBAS" # More SBAS
                elif 193 <= prn <= 197: gnss = "QZSS"
                elif 201 <= prn <= 235: gnss = "Galileo"
                elif 301 <= prn <= 336: gnss = "BeiDou"

            values = (prn_str, gnss, str(elev), str(azim), str(snr), used_text)

            if prn_str in tree_items:
                # --- Update existing item ---
                item_id = tree_items[prn_str]
                try:
                     # Update values if they changed
                     current_values = self.sat_tree.item(item_id, 'values')
                     if current_values != values:
                         self.sat_tree.item(item_id, values=values, tags=(tag,))
                     # Update tag if it changed
                     current_tags = self.sat_tree.item(item_id, 'tags')
                     if tag not in current_tags:
                          self.sat_tree.item(item_id, tags=(tag,)) # Overwrite tags
                except tk.TclError: pass # Item might have been deleted concurrently?
            else:
                # --- Insert new item ---
                try:
                    self.sat_tree.insert("", tk.END, values=values, tags=(tag,), iid=prn_str) # Use PRN as item ID
                except tk.TclError as insert_err:
                     print(f"WARN: Error inserting sat {prn_str} into tree: {insert_err}") # e.g., duplicate IID?


        # --- Remove items from tree that are no longer in sat_data_list ---
        prns_in_tree = set(tree_items.keys())
        prns_to_remove = prns_in_tree - updated_prns
        for prn_to_remove in prns_to_remove:
            try:
                item_id = tree_items[prn_to_remove]
                if self.sat_tree.exists(item_id):
                     self.sat_tree.delete(item_id)
            except tk.TclError: pass # Item might already be gone


        # --- Restore selection and scroll position ---
        try:
            if selected_items:
                # Filter items that still exist
                valid_selection = [item for item in selected_items if self.sat_tree.exists(item)]
                if valid_selection:
                    self.sat_tree.selection_set(valid_selection)
                    # Optionally focus on the first selected item
                    # self.sat_tree.focus(valid_selection[0])
            self.sat_tree.yview_moveto(scroll_pos[0])
        except tk.TclError: pass # Handle errors if items/view changed drastically


        # --- Update Summary Labels ---
        # Check if labels exist before configuring
        if hasattr(self, 'summary_gnss') and self.summary_gnss:
             gnss_systems = data.get('gnss_systems_used', set())
             self.summary_gnss.config(text=", ".join(sorted(list(gnss_systems))) if gnss_systems else "None")

        if hasattr(self, 'summary_sats') and self.summary_sats:
             sats_view = data.get('satellites_in_view', 0)
             sats_used_count = len(sats_used_set)
             self.summary_sats.config(text=f"{sats_view} / {sats_used_count}")

        if hasattr(self, 'summary_dop') and self.summary_dop:
             hdop = data.get('hdop', 0.0)
             pdop = data.get('pdop', 0.0)
             vdop = data.get('vdop', 0.0)
             self.summary_dop.config(text=f"{hdop:.2f} / {pdop:.2f} / {vdop:.2f}" if hdop > 0 else "-- / -- / --")

        if hasattr(self, 'summary_accuracy') and self.summary_accuracy:
             accuracy = data.get('accuracy_estimate', 0.0)
             self.summary_accuracy.config(text=f"~ {accuracy:.2f} m" if accuracy > 0 else "--")

        # Update window title with timestamp
        now = datetime.datetime.now().strftime('%H:%M:%S')
        if self.sat_window and self.sat_window.winfo_exists():
            self.sat_window.title(f"Satellite Information (Updated: {now})")


    def close_satellite_window(self):
        """Callback when the satellite window is closed by the user."""
        if self.sat_window:
            print("Closing satellite window.")
            self.sat_window.destroy()
        self.sat_window = None
        self.sat_tree = None
        # Clear summary label references too
        self.summary_gnss = None
        self.summary_sats = None
        self.summary_dop = None
        self.summary_accuracy = None


    # --- Application Exit ---
    def on_close(self):
        """Perform cleanup actions before closing the application."""
        print("Closing application...")
        if self.gui_update_after_id:
            try: self.root.after_cancel(self.gui_update_after_id)
            except: pass
        if self.gps_reader:
            self.gps_reader.stop() # Ensure reader is stopped cleanly
        # Close satellite window if open
        if self.sat_window:
             try: self.close_satellite_window()
             except: pass
        self.root.quit()
        # Delay destroy slightly to allow quit to process? Sometimes helps.
        self.root.after(50, self.root.destroy)
        print("Application closed.")


# --- Main Execution ---
def main():
    """Main function to create and run the Tkinter application."""
    root = tk.Tk()
    app = GPSApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    if platform.system() == "Windows":
         try:
             from ctypes import windll
             windll.shcore.SetProcessDpiAwareness(1)
         except Exception: pass
    main()