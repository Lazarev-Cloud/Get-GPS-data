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
from typing import Optional, List, Dict, Any, Tuple, Union

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
UPDATE_INTERVAL_MS = 500  # Faster GUI update
MAX_POSITION_HISTORY = 10
CSV_LOG_HEADERS = [
    'Timestamp', 'FixTimestamp', 'Latitude', 'Longitude', 'Altitude',
    'Speed_KPH', 'SatellitesInView', 'FixQuality', 'HDOP', 'PDOP', 'VDOP',
    'FixType', 'AccuracyEstimate_m', 'GeoidHeight', 'GNSSSystemsUsed',
    'RawNMEA'
]
FIX_TYPE_MAP = {
    0: "No Fix", 1: "GPS (SPS)", 2: "DGPS", 3: "PPS",
    4: "RTK Fixed", 5: "RTK Float", 6: "Estimated (Dead Reckoning)",
    7: "Manual", 8: "Simulation"
}
FIX_QUALITY_MAP = {
    0: 'Invalid', 1: 'GPS fix (SPS)', 2: 'DGPS fix', 3: 'PPS fix',
    4: 'Real Time Kinematic', 5: 'Float RTK', 6: 'estimated (dead reckoning)',
    7: 'Manual input mode', 8: 'Simulation mode'
}
SMOOTHING_NONE = "None"
SMOOTHING_AVG = "Moving Average"
SMOOTHING_KALMAN = "Kalman Filter"
SMOOTHING_METHODS = [SMOOTHING_NONE, SMOOTHING_AVG, SMOOTHING_KALMAN]


# --- Helper Functions ---
def degrees_to_dms(degrees: float) -> str:
    """Convert decimal degrees to Degrees Minutes Seconds string."""
    if not isinstance(degrees, (float, int)) or degrees == 0.0:
        return "--"
    is_negative = degrees < 0
    degrees = abs(degrees)
    d = int(degrees)
    m = (degrees - d) * 60
    return f"{'-' if is_negative else ''}{d}° {m:.4f}'"

def knots_to_kph(knots: Optional[float]) -> float:
    """Convert speed from knots to km/h."""
    return float(knots) * 1.852 if knots is not None else 0.0


# --- GPS Reader Class ---
class GPSReader:
    """Handles serial communication, NMEA parsing, and data processing."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD_RATE):
        """Initialize GPS reader."""
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.running = threading.Event() # Use Event for clearer start/stop signalling
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock() # Lock for accessing shared data

        # --- Data Storage ---
        self.current_data: Dict[str, Any] = self._get_default_data()
        self.position_history: List[Tuple[float, float, float]] = []
        self.max_history_size = MAX_POSITION_HISTORY
        self.smoothing_method = SMOOTHING_NONE

        # --- Logging ---
        self.log_file: Optional[object] = None # Use 'object' for file handle type hint flexibility
        self.csv_writer: Optional[csv.writer] = None
        self.log_enabled = False
        self.log_filename: Optional[str] = None

        # --- Kalman Filter ---
        self.kalman_filter: Optional[Dict[str, Any]] = None
        if NUMPY_AVAILABLE:
            self.initialize_kalman_filter()

    def _get_default_data(self) -> Dict[str, Any]:
        """Return a dictionary with default values for GPS data."""
        return {
            'timestamp': None, 'fix_timestamp': None,
            'lat': 0.0, 'lon': 0.0, 'altitude': 0.0, 'raw_lat': 0.0, 'raw_lon': 0.0, 'raw_alt': 0.0,
            'speed_kph': 0.0, 'satellites_in_view': 0, 'satellites_used': [],
            'fix_quality': 0, 'hdop': 0.0, 'pdop': 0.0, 'vdop': 0.0,
            'geoid_height': 0.0, 'fix_type': 0, 'satellite_data': [],
            'gnss_systems_used': set(), 'accuracy_estimate': 0.0, 'raw_sentence': '',
        }

    # --- Kalman Filter Methods ---
    def initialize_kalman_filter(self):
        """Initialize Kalman filter state if NumPy is available."""
        if not NUMPY_AVAILABLE:
            self.kalman_filter = None
            return

        # Simplified Kalman Filter for Position (Lat, Lon, Alt)
        # Assumes constant velocity model is less useful without velocity measurements
        # State: [lat, lon, alt]
        # Measurement: [lat, lon, alt]
        self.kalman_filter = {
            'x': np.zeros(3),       # State vector [lat, lon, alt]
            'P': np.eye(3) * 500,   # Initial state covariance (uncertainty)
            'F': np.eye(3),         # State transition matrix (assume static for position only)
            'H': np.eye(3),         # Measurement matrix
            'Q': np.eye(3) * 0.05,  # Process noise (model uncertainty)
            'R': np.eye(3) * 10.0   # Measurement noise (sensor uncertainty) - adjust based on GPS quality
        }
        print("Kalman filter initialized.")

    def _apply_kalman_filter(self, lat: float, lon: float, alt: float) -> Tuple[float, float, float]:
        """Apply Kalman filter step."""
        if not self.kalman_filter or not NUMPY_AVAILABLE:
            return lat, lon, alt
        if lat == 0 or lon == 0: # Don't filter invalid positions
             # Optionally reset filter if needed after prolonged invalid data
            # self.initialize_kalman_filter()
            return lat, lon, alt

        try:
            kf = self.kalman_filter
            x = kf['x']
            P = kf['P']
            F = kf['F']
            Q = kf['Q']
            H = kf['H']
            R = kf['R']
            z = np.array([lat, lon, alt]) # Measurement

            # --- Predict ---
            x_pred = F @ x  # In this simple model F=I, so x_pred = x
            P_pred = F @ P @ F.T + Q

            # --- Update ---
            y = z - H @ x_pred             # Measurement residual
            S = H @ P_pred @ H.T + R     # Residual covariance
            K = P_pred @ H.T @ np.linalg.inv(S) # Kalman gain

            x_new = x_pred + K @ y
            P_new = (np.eye(len(x)) - K @ H) @ P_pred

            # Save updated state
            kf['x'] = x_new
            kf['P'] = P_new

            return float(x_new[0]), float(x_new[1]), float(x_new[2])

        except np.linalg.LinAlgError:
            print("Kalman filter: Singular matrix error. Resetting filter.")
            self.initialize_kalman_filter() # Reset filter on numerical instability
            return lat, lon, alt
        except Exception as e:
            print(f"Kalman filter error: {e}")
            return lat, lon, alt # Return raw on error

    # --- Moving Average Methods ---
    def _add_to_position_history(self, lat: float, lon: float, alt: float):
        """Add position to history buffer."""
        if lat == 0 or lon == 0: return
        self.position_history.append((lat, lon, alt))
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)

    def _get_averaged_position(self) -> Tuple[float, float, float]:
        """Calculate simple moving average of recent positions."""
        if not self.position_history:
            return 0.0, 0.0, 0.0

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
            # Consider adding a small delay or check if ready before configuring
            time.sleep(0.5)
            self._configure_gps_module() # Attempt configuration after connection
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
        # !! These are specific to u-blox 6/7/8 series !!
        # !! May not work or cause issues on other GPS modules !!
        commands = {
            "Enable SBAS": "B5 62 06 16 08 00 01 01 03 00 00 00 00 00 29 8B",
            "Set Rate 5Hz": "B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A", # 200ms interval
            # "Enable All GNSS (Example for NEO-M8)": "B5 62 06 3E 3C 00...", # Complex command, omitted for brevity
            "Enable GNGGA": "B5 62 06 01 08 00 F0 00 01 01 01 01 01 01 04 4B",
            "Enable GNGLL": "B5 62 06 01 08 00 F0 01 00 01 00 00 00 00 00 46", # Disable GLL by default?
            "Enable GNGSA": "B5 62 06 01 08 00 F0 02 01 01 01 01 01 01 06 55",
            "Enable GPGSV": "B5 62 06 01 08 00 F0 03 01 01 01 01 01 01 07 5A", # GPS
            "Enable GLGSV": "B5 62 06 01 08 00 F0 06 01 01 01 01 01 01 0A 6B", # GLONASS (if supported)
            "Enable GNRMC": "B5 62 06 01 08 00 F0 04 01 01 01 01 01 01 08 5F",
            "Enable GNVTG": "B5 62 06 01 08 00 F0 05 01 01 01 01 01 01 09 64",
        }

        print("Attempting to send configuration commands to GPS module...")
        try:
            for name, hex_cmd in commands.items():
                try:
                    cmd_bytes = bytearray.fromhex(hex_cmd)
                    self.ser.write(cmd_bytes)
                    print(f"  Sent: {name}")
                    time.sleep(0.1) # Small delay between commands
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
        self.stop_logging() # Ensure any previous log is closed
        try:
            # Use 'with' statement for automatic file closing on error/exit
            # Open file immediately to catch errors early
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

    # --- Data Parsing ---
    def _parse_gps_data(self, line: str):
        """Parse NMEA sentence and update internal data dictionary."""
        if not line.startswith('$'):
            # print(f"DEBUG: Skipping non-NMEA line: {line}")
            return False # Not an NMEA sentence

        new_data = self._get_default_data() # Start with fresh defaults for this update cycle
        # Keep some persistent data like satellite list across updates until overwritten
        new_data['satellite_data'] = self.current_data['satellite_data'] # Persist satellite list between GSA/GSV
        new_data['gnss_systems_used'] = self.current_data['gnss_systems_used'] # Persist systems

        try:
            msg = pynmea2.parse(line)
            new_data['raw_sentence'] = line # Store raw sentence that was successfully parsed

            # --- Process different NMEA types ---
            if hasattr(msg, 'timestamp'):
                # Use device timestamp if available, otherwise keep host timestamp
                new_data['fix_timestamp'] = msg.timestamp

            if isinstance(msg, pynmea2.types.talker.GGA):
                new_data['raw_lat'] = float(msg.latitude) if msg.latitude else 0.0
                new_data['raw_lon'] = float(msg.longitude) if msg.longitude else 0.0
                new_data['raw_alt'] = float(msg.altitude) if msg.altitude is not None else 0.0
                new_data['fix_quality'] = int(msg.gps_qual) if msg.gps_qual else 0
                new_data['satellites_in_view'] = int(msg.num_sats) if msg.num_sats else 0 # Usually satellites used in fix for GGA
                new_data['hdop'] = float(msg.horizontal_dil) if msg.horizontal_dil else 0.0
                new_data['geoid_height'] = float(msg.geo_sep) if msg.geo_sep else 0.0

            elif isinstance(msg, pynmea2.types.talker.RMC):
                # RMC often provides the primary fix status and speed/course
                new_data['raw_lat'] = float(msg.latitude) if msg.latitude else self.current_data['raw_lat'] # Keep last known if RMC doesn't have it
                new_data['raw_lon'] = float(msg.longitude) if msg.longitude else self.current_data['raw_lon']
                new_data['speed_kph'] = knots_to_kph(msg.spd_over_grnd)
                # RMC status ('A' = Active/valid, 'V' = Void/invalid) can override fix quality
                if msg.status == 'V':
                    new_data['fix_quality'] = 0 # Force to invalid if RMC says so
                elif msg.status == 'A' and self.current_data['fix_quality'] == 0:
                     new_data['fix_quality'] = 1 # Assume basic GPS fix if RMC is Active and no other quality info yet

            elif isinstance(msg, pynmea2.types.talker.GSA):
                # Provides fix type (2D/3D) and DOP values, lists satellites used in fix
                new_data['fix_type'] = int(msg.mode_fix_type) if msg.mode_fix_type else 0
                new_data['pdop'] = float(msg.pdop) if msg.pdop else 0.0
                new_data['hdop'] = float(msg.hdop) if msg.hdop else self.current_data['hdop'] # Prefer GSA's HDOP if available
                new_data['vdop'] = float(msg.vdop) if msg.vdop else 0.0
                new_data['satellites_used'] = [int(s) for s in msg.sv_id if s] # List of PRNs used

                # Infer GNSS systems from satellite IDs (simple approach)
                current_systems = set()
                for prn in new_data['satellites_used']:
                   # Basic PRN ranges (may vary slightly or need updating for new constellations)
                    if 1 <= prn <= 32: current_systems.add("GPS")
                    elif 33 <= prn <= 64: current_systems.add("WAAS/SBAS") # Or other SBAS based on NMEA Talker ID (GP vs GA vs GB etc)
                    elif 65 <= prn <= 96: current_systems.add("GLONASS")
                    # Add ranges for Galileo (Europe), BeiDou (China), QZSS (Japan), etc. if needed
                    # elif 193 <= prn <= 197: current_systems.add("QZSS") # Example
                    # elif 201 <= prn <= 235: current_systems.add("Galileo") # Example
                    # elif 301 <= prn <= 336: current_systems.add("BeiDou") # Example
                new_data['gnss_systems_used'] = current_systems


            elif isinstance(msg, pynmea2.types.talker.GSV):
                # Provides detailed info about satellites in view (signal strength, elevation, azimuth)
                # GSV messages can be multi-part
                num_sv_in_view = int(msg.num_sv_in_view) if msg.num_sv_in_view else 0
                new_data['satellites_in_view'] = max(num_sv_in_view, self.current_data['satellites_in_view']) # Keep max count seen recently
                msg_num = int(msg.msg_num) if msg.msg_num else 1
                num_msgs = int(msg.num_messages) if msg.num_messages else 1

                # Clear previous satellite data only when receiving the first message of a sequence
                if msg_num == 1:
                   new_data['satellite_data'] = []

                sats = []
                for i in range(1, 5):  # GSV has fields for up to 4 satellites
                    prn = getattr(msg, f'sv_prn_num_{i}', None)
                    if prn:
                        sats.append({
                            'prn': int(prn),
                            'elevation': int(getattr(msg, f'elevation_{i}', 0) or 0),
                            'azimuth': int(getattr(msg, f'azimuth_{i}', 0) or 0),
                            'snr': int(getattr(msg, f'snr_{i}', 0) or 0)
                        })
                # Append new sats to potentially existing list from previous messages in sequence
                new_data['satellite_data'].extend(sats)
                # Keep only unique PRNs, preferring latest info if duplicates arrive (unlikely in single sequence)
                seen_prns = set()
                unique_sats = []
                for sat in reversed(new_data['satellite_data']): # Process newest first
                    if sat['prn'] not in seen_prns:
                        unique_sats.append(sat)
                        seen_prns.add(sat['prn'])
                new_data['satellite_data'] = list(reversed(unique_sats)) # Restore original order


            elif isinstance(msg, pynmea2.types.talker.VTG):
                # Provides Course and Speed Over Ground
                 new_data['speed_kph'] = float(msg.spd_over_grnd_kmph) if msg.spd_over_grnd_kmph is not None else self.current_data['speed_kph']


            # --- Post-processing and Smoothing ---
            # Determine final Lat/Lon/Alt based on smoothing method
            proc_lat, proc_lon, proc_alt = new_data['raw_lat'], new_data['raw_lon'], new_data['raw_alt']

            if self.smoothing_method == SMOOTHING_AVG:
                self._add_to_position_history(proc_lat, proc_lon, proc_alt)
                proc_lat, proc_lon, proc_alt = self._get_averaged_position()
            elif self.smoothing_method == SMOOTHING_KALMAN:
                proc_lat, proc_lon, proc_alt = self._apply_kalman_filter(proc_lat, proc_lon, proc_alt)

            new_data['lat'] = proc_lat
            new_data['lon'] = proc_lon
            new_data['altitude'] = proc_alt

            # --- Estimate Accuracy (Heuristic) ---
            hdop = new_data['hdop']
            sats_used = len(new_data['satellites_used'])
            if hdop > 0 and sats_used >= 4:
                # Very rough estimate: Base precision * HDOP, adjusted by satellite count
                base_precision = 2.5 # meters (typical good conditions)
                accuracy = hdop * base_precision
                if sats_used < 5: accuracy *= 1.5 # Penalize low satellite count
                elif sats_used > 8: accuracy *= 0.8 # Reward high satellite count
                new_data['accuracy_estimate'] = round(accuracy, 2)
            else:
                new_data['accuracy_estimate'] = 0.0 # Cannot estimate reliably


            # --- Update Shared Data (Thread Safe) ---
            with self.lock:
                # Merge new data into current data, overwriting only fields present in new_data
                # This ensures fields from different sentences (like altitude from GGA, speed from RMC) persist
                for key, value in new_data.items():
                     # Only update if the new value is considered valid/useful
                     # Avoid overwriting a good value with a default/zero from a sentence that doesn't contain it
                     if key in ['lat', 'lon', 'altitude'] and value == 0.0 and self.current_data.get(key, 0.0) != 0.0:
                         continue # Don't overwrite valid coords with 0.0
                     if key in ['hdop', 'pdop', 'vdop'] and value == 0.0 and self.current_data.get(key, 0.0) != 0.0:
                         continue # Don't overwrite valid DOPs with 0.0
                     if key == 'fix_quality' and value == 0 and self.current_data.get(key, 0) != 0:
                         continue # Don't overwrite a valid fix quality with 0 unless RMC explicitly says 'V' (handled earlier)

                     # Allow satellite data and GNSS systems to be overwritten/updated
                     if key not in ['satellite_data', 'gnss_systems_used']:
                          if value is None or value == '' or value == []: # Treat empty values carefully
                              # Keep old value if new one is empty, unless it's a specific field we always want updated
                              if key not in ['raw_sentence', 'fix_timestamp']: # Always update raw sentence and timestamp
                                  continue
                     self.current_data[key] = value

                # Always update timestamp from host
                self.current_data['timestamp'] = datetime.datetime.now()

                # --- Log data if enabled ---
                if self.log_enabled and self.csv_writer:
                    try:
                        log_row = [
                            self.current_data['timestamp'].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                            self.current_data['fix_timestamp'].strftime("%H:%M:%S.%f")[:-3] if self.current_data['fix_timestamp'] else '',
                            f"{self.current_data['lat']:.7f}",
                            f"{self.current_data['lon']:.7f}",
                            f"{self.current_data['altitude']:.2f}",
                            f"{self.current_data['speed_kph']:.2f}",
                            self.current_data['satellites_in_view'],
                            FIX_QUALITY_MAP.get(self.current_data['fix_quality'], self.current_data['fix_quality']),
                            f"{self.current_data['hdop']:.2f}",
                            f"{self.current_data['pdop']:.2f}",
                            f"{self.current_data['vdop']:.2f}",
                            FIX_TYPE_MAP.get(self.current_data['fix_type'], self.current_data['fix_type']),
                            f"{self.current_data['accuracy_estimate']:.2f}",
                            f"{self.current_data['geoid_height']:.2f}",
                            ",".join(sorted(list(self.current_data['gnss_systems_used']))),
                            self.current_data['raw_sentence'] # Log the specific sentence
                        ]
                        self.csv_writer.writerow(log_row)
                        # Consider flushing less often for performance?
                        # self.log_file.flush()
                    except Exception as log_err:
                        print(f"ERROR writing to log file: {log_err}")
                        # Consider disabling logging after repeated errors?
                        # self.stop_logging()

            return True # Successfully parsed and processed

        except pynmea2.ParseError as e:
            # print(f"WARN: Could not parse NMEA sentence: {line} - {e}")
            return False
        except ValueError as e:
            print(f"WARN: Value error parsing NMEA sentence: {line} - {e}")
            return False
        except AttributeError as e:
             print(f"WARN: Attribute error parsing NMEA sentence (likely missing field): {line} - {e}")
             return False
        except Exception as e:
            print(f"ERROR: Unexpected error parsing NMEA data: {line} - {e}")
            # import traceback
            # traceback.print_exc() # More detailed error for debugging
            return False

    # --- Reading Thread ---
    def _read_loop(self):
        """Background thread task to continuously read from serial port."""
        print("Read thread started.")
        buffer = ""
        while self.running.is_set():
            try:
                if self.ser and self.ser.is_open:
                    # Read available bytes, decode safely, add to buffer
                    try:
                        # Adjust read size based on expected data rate
                        # Using read_all() or large read() can be efficient
                        # bytes_to_read = self.ser.in_waiting or 1
                        # raw_data = self.ser.read(bytes_to_read)
                        raw_data = self.ser.readline() # Readline is often better for NMEA
                    except serial.SerialException as serial_err:
                         print(f"ERROR: Serial read error: {serial_err}. Attempting to reconnect...")
                         self.running.clear() # Signal stop
                         self.disconnect()
                         time.sleep(2) # Wait before potential reconnect attempt by main thread
                         break # Exit read loop
                    except Exception as read_err:
                        print(f"ERROR: Unexpected read error: {read_err}")
                        time.sleep(0.1) # Avoid busy-looping on unexpected errors
                        continue

                    if raw_data:
                        try:
                            data = raw_data.decode('ascii', errors='replace').strip()
                            if data:
                                # Process complete lines
                                self._parse_gps_data(data)
                        except UnicodeDecodeError:
                            # print(f"WARN: Could not decode bytes: {raw_data}")
                            pass # Skip undecodable data
                    else:
                        # Timeout occurred or no data, sleep briefly
                        time.sleep(0.05) # Shorter sleep when waiting for data

                else:
                    # Serial port not open/available, wait before checking again
                    time.sleep(0.5)

            except Exception as loop_err:
                print(f"ERROR: Unhandled exception in read loop: {loop_err}")
                import traceback
                traceback.print_exc()
                time.sleep(1) # Avoid rapid error loops

        print("Read thread finished.")
        # Clean up log file flushing if thread stops unexpectedly
        if self.log_enabled and self.log_file:
             try: self.log_file.flush()
             except: pass


    # --- Control Methods ---
    def start(self) -> bool:
        """Start the GPS reading thread."""
        if self.running.is_set():
            print("Reader already running.")
            return True
        if not self.connect(): # Try connecting first
             return False

        self.running.set()
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
        return True

    def stop(self):
        """Stop the GPS reading thread and disconnect."""
        print("Stopping GPS reader...")
        self.running.clear() # Signal thread to stop

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0) # Wait for thread to finish
            if self.read_thread.is_alive():
                 print("WARN: Read thread did not terminate gracefully.")

        self.disconnect()
        self.stop_logging() # Ensure logging is stopped and file closed
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
                self.smoothing_method = method
                print(f"Smoothing method set to: {method}")
                if method == SMOOTHING_AVG and history_size is not None:
                    self.max_history_size = max(1, history_size)
                    self.position_history = [] # Clear history on change
                    print(f"Moving average history size set to: {self.max_history_size}")
                elif method == SMOOTHING_KALMAN:
                    if NUMPY_AVAILABLE:
                        self.initialize_kalman_filter() # Reset filter state
                    else:
                        print("WARN: Cannot use Kalman filter, NumPy not installed.")
                        self.smoothing_method = SMOOTHING_NONE # Fallback
            else:
                print(f"WARN: Invalid smoothing method '{method}' requested.")

# --- GUI Application Class ---
class GPSApp:
    """Tkinter GUI for displaying GPS data."""

    def __init__(self, root: tk.Tk):
        """Initialize the application."""
        self.root = root
        self.root.title("Enhanced GPS Data Viewer")
        # Set minimum size, allow resizing
        self.root.minsize(750, 600)
        # self.root.geometry("800x650") # Initial size

        self.gps_reader: Optional[GPSReader] = None
        self.gui_update_after_id: Optional[str] = None

        # --- Satellite Window ---
        self.sat_window: Optional[tk.Toplevel] = None
        self.sat_tree: Optional[ttk.Treeview] = None

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
        style.theme_use('clam') # Or 'alt', 'default', 'classic'
        style.configure("TLabel", padding=2)
        style.configure("TButton", padding=5)
        style.configure("TCombobox", padding=2)
        style.configure("TLabelframe.Label", padding=(5, 2))
        style.configure("Treeview.Heading", font=('Helvetica', 10, 'bold'))
        # Add tag for satellites used in fix
        style.configure("Used.Treeview", foreground="green")


    def _create_widgets(self):
        """Create all GUI elements."""
        # --- Main Panes ---
        # Use PanedWindow for adjustable sections
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

        self.refresh_btn = ttk.Button(conn_frame, text="↺", width=3, command=self.refresh_ports, style="Toolbutton")
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
        data_fields = [
            ("lat", "Latitude:", 0), ("lon", "Longitude:", 1),
            ("altitude", "Altitude:", 2), ("geoid_height", "Geoid Height:", 3),
            ("speed_kph", "Speed:", 4),
            ("fix_quality", "Fix Quality:", 5), ("fix_type", "Fix Type:", 6),
            ("satellites_in_view", "Sats View/Used:", 7), # Combined label
            ("accuracy_estimate", "Est. Accuracy:", 8),
            ("hdop", "HDOP:", 9), ("pdop", "PDOP:", 10), ("vdop", "VDOP:", 11),
            ("gnss_systems_used", "GNSS Systems:", 12),
            ("fix_timestamp", "Device Time:", 13), ("timestamp", "Host Time:", 14),
        ]

        row_offset = 0
        for field, label_text, row in data_fields:
            ttk.Label(data_frame, text=label_text).grid(row=row_offset + row, column=0, padx=5, pady=2, sticky=tk.W)
            self.data_labels[field] = ttk.Label(data_frame, text="--", anchor=tk.W, relief=tk.SUNKEN, padding=2)
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
        self.port_combo.config(state="readonly" if not connected else tk.DISABLED)
        self.baud_combo.config(state="readonly" if not connected else tk.DISABLED)
        self.refresh_btn.config(state=tk.NORMAL if not connected else tk.DISABLED)
        self.connect_btn.config(text="Disconnect" if connected else "Connect")
        self.log_btn.config(state=tk.NORMAL if connected else tk.DISABLED)
        self.log_btn.config(text="Stop Logging" if logging else "Start Logging")
        self.map_btn.config(state=tk.NORMAL if connected and has_fix else tk.DISABLED)
        self.sat_btn.config(state=tk.NORMAL if connected else tk.DISABLED) # Enable Sat window even without fix
        self.smoothing_combo.config(state="readonly") # Always selectable
        self.history_spin.config(state="readonly" if self.smoothing_var.get() == SMOOTHING_AVG else tk.DISABLED)
        self.history_label.config(state=tk.NORMAL if self.smoothing_var.get() == SMOOTHING_AVG else tk.DISABLED)


    def _get_available_ports(self) -> List[Tuple[str, str]]:
        """Get a list of available serial ports with descriptions."""
        ports = serial.tools.list_ports.comports()
        port_list = []
        for port in sorted(ports, key=lambda p: p.device):
            # Format: ("COM5", "COM5: USB-SERIAL CH340 (COM5)")
            # On Linux: ("/dev/ttyUSB0", "/dev/ttyUSB0: USB Serial")
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
            # Keep current selection if still available
            pass
        elif available_ports:
             # Default to the last port, often the most relevant USB serial
             self.port_var.set(available_ports[-1][1])


        self.status_var.set(f"Found {len(available_ports)} ports." if available_ports else "No serial ports found.")


    def toggle_connection(self):
        """Connect to or disconnect from the GPS reader."""
        if self.gps_reader and self.gps_reader.running.is_set():
            # --- Disconnect ---
            self.status_var.set("Disconnecting...")
            self.root.update_idletasks() # Show status update immediately
            if self.gui_update_after_id:
                self.root.after_cancel(self.gui_update_after_id)
                self.gui_update_after_id = None
            self.gps_reader.stop()
            self.gps_reader = None
            self._clear_data_labels()
            self.nmea_text.config(state=tk.NORMAL)
            self.nmea_text.delete(1.0, tk.END)
            self.nmea_text.config(state=tk.DISABLED)
            self.status_var.set("Disconnected")
            self._update_ui_states(connected=False)
            if self.sat_window: # Close satellite window on disconnect
                 self.close_satellite_window()

        else:
            # --- Connect ---
            port_desc = self.port_var.get()
            if not port_desc:
                messagebox.showerror("Connection Error", "Please select a serial port.")
                return
            # Extract device name (e.g., "COM5" or "/dev/ttyUSB0")
            port = port_desc.split(':')[0].strip()

            try:
                baud = int(self.baud_var.get())
            except ValueError:
                messagebox.showerror("Connection Error", "Invalid baud rate selected.")
                return

            self.status_var.set(f"Connecting to {port}...")
            self.root.update_idletasks()
            self._update_ui_states(connected=False) # Disable controls during connection attempt
            self.connect_btn.config(state=tk.DISABLED)

            # Run connection in a separate thread to avoid blocking GUI (optional but good practice)
            # For simplicity here, we connect directly but show status updates
            self.gps_reader = GPSReader(port, baud)
            # Apply initial smoothing setting from UI
            self.apply_accuracy_settings(initial=True)

            if self.gps_reader.start():
                self.status_var.set(f"Connected to {port} at {baud} baud")
                self._update_ui_states(connected=True)
                self.connect_btn.config(state=tk.NORMAL) # Re-enable button
                 # Start the GUI update loop
                self.gui_update_after_id = self.root.after(UPDATE_INTERVAL_MS, self.update_gui)
            else:
                self.status_var.set(f"Failed to connect to {port}")
                messagebox.showerror("Connection Failed", f"Could not connect to {port}.\nCheck port, permissions, and if device is in use.")
                self.gps_reader = None
                self._update_ui_states(connected=False)
                self.connect_btn.config(state=tk.NORMAL) # Re-enable button


    def toggle_logging(self):
        """Start or stop logging GPS data to a CSV file."""
        if not self.gps_reader:
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
            if filename: # User didn't cancel
                if self.gps_reader.start_logging(filename):
                    self.status_var.set(f"Logging to {os.path.basename(filename)}")
                    self._update_ui_states(connected=True, logging=True, has_fix=self._has_fix())
                else:
                    messagebox.showerror("Logging Error", "Could not start logging.\nCheck file permissions.")
                    self.status_var.set("Logging failed to start.")
                    self._update_ui_states(connected=True, logging=False, has_fix=self._has_fix())
            else:
                # User cancelled save dialog
                self.status_var.set("Logging cancelled.")


    def apply_accuracy_settings(self, event=None, initial=False):
        """Apply selected smoothing/accuracy settings."""
        if not self.gps_reader and not initial: # Allow applying initial setting before reader exists
            return

        method = self.smoothing_var.get()
        history_size = None
        if method == SMOOTHING_AVG:
            try:
                history_size = int(self.history_var.get())
                if history_size < 1: raise ValueError("History size must be positive")
            except ValueError:
                messagebox.showerror("Settings Error", "Invalid history size. Please enter a positive integer.")
                # Optionally revert UI?
                return

        if self.gps_reader: # Apply to existing reader
             self.gps_reader.set_smoothing(method, history_size)
             self.status_var.set(f"Smoothing: {method}" + (f", History: {history_size}" if history_size else ""))

        # Update UI state related to history spinbox
        self._update_ui_states(connected=(self.gps_reader is not None and self.gps_reader.running.is_set()),
                               logging=self.gps_reader.log_enabled if self.gps_reader else False,
                               has_fix=self._has_fix())


    def _clear_data_labels(self):
        """Reset all data display labels to '--'."""
        for label in self.data_labels.values():
            label.config(text="--")
        # Clear NMEA text - handled in disconnect

    def _has_fix(self) -> bool:
        """Check if the current GPS data indicates a valid fix."""
        if not self.gps_reader: return False
        data = self.gps_reader.get_current_data()
        # Consider quality > 0 and non-zero lat/lon as a basic fix indicator
        return data.get('fix_quality', 0) > 0 and data.get('lat', 0.0) != 0.0 and data.get('lon', 0.0) != 0.0

    def update_gui(self):
        """Periodically fetch data from GPSReader and update GUI elements."""
        if not self.gps_reader or not self.gps_reader.running.is_set():
             # print("DEBUG: Update GUI called but reader not running.")
             if self.gui_update_after_id:
                 self.root.after_cancel(self.gui_update_after_id)
                 self.gui_update_after_id = None
             return # Stop updates if reader stopped

        data = self.gps_reader.get_current_data()
        has_fix = self._has_fix() # Check fix status based on latest data

        # --- Update Data Labels ---
        for field, label_widget in self.data_labels.items():
            value = data.get(field)
            text = "--" # Default text

            try:
                if field == "lat": text = degrees_to_dms(value)
                elif field == "lon": text = degrees_to_dms(value)
                elif field in ["altitude", "geoid_height"]:
                    text = f"{float(value):.2f} m" if value is not None and value != 0.0 else "--"
                elif field == "speed_kph":
                    text = f"{float(value):.2f} km/h" if value is not None else "--"
                elif field == "fix_quality":
                    text = FIX_QUALITY_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "fix_type":
                    text = FIX_TYPE_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "satellites_in_view":
                    sats_view = data.get('satellites_in_view', 0)
                    sats_used_list = data.get('satellites_used', [])
                    sats_used = len(sats_used_list) if isinstance(sats_used_list, list) else 0
                    text = f"{sats_view} / {sats_used}"
                elif field in ["hdop", "pdop", "vdop"]:
                     text = f"{float(value):.2f}" if value is not None and value > 0 else "--"
                elif field == "accuracy_estimate":
                     text = f"~ {float(value):.2f} m" if value is not None and value > 0 else "--"
                elif field == "gnss_systems_used":
                     text = ", ".join(sorted(list(value))) if value else "None"
                elif field == "fix_timestamp":
                    text = value.strftime("%H:%M:%S.%f")[:-3] if value else "--"
                elif field == "timestamp":
                    text = value.strftime("%Y-%m-%d %H:%M:%S") if value else "--"
                else:
                    # Default string conversion for other fields if needed
                    text = str(value) if value is not None else "--"

                label_widget.config(text=text)
            except (ValueError, TypeError, AttributeError) as fmt_err:
                 print(f"WARN: Error formatting field '{field}' with value '{value}': {fmt_err}")
                 label_widget.config(text="Error") # Indicate formatting issue

        # --- Update NMEA Text Area ---
        raw_sentence = data.get('raw_sentence', '')
        if raw_sentence:
            self.nmea_text.config(state=tk.NORMAL)
            self.nmea_text.insert(tk.END, raw_sentence + '\n')
            self.nmea_text.see(tk.END) # Auto-scroll
            # Limit buffer size (optional)
            if int(self.nmea_text.index('end-1c').split('.')[0]) > 200: # Keep approx last 200 lines
                self.nmea_text.delete('1.0', '2.0')
            self.nmea_text.config(state=tk.DISABLED)

        # --- Update Status Bar ---
        if has_fix:
            self.status_bar.config(foreground="green")
            fix_desc = FIX_QUALITY_MAP.get(data.get('fix_quality', 0), "Fix")
            self.status_var.set(f"Status: {fix_desc} | Sats: {data.get('satellites_in_view', 0)}/{len(data.get('satellites_used',[]))} | HDOP: {data.get('hdop', 0.0):.2f}")
        else:
            self.status_bar.config(foreground="red")
            self.status_var.set("Status: No Fix / Acquiring...")

        # Update button states based on fix
        self._update_ui_states(connected=True, logging=self.gps_reader.log_enabled, has_fix=has_fix)

        # --- Update Satellite Window (if open) ---
        if self.sat_window and self.sat_window.winfo_exists() and self.sat_tree:
            self.update_satellite_window()

        # --- Schedule next update ---
        self.gui_update_after_id = self.root.after(UPDATE_INTERVAL_MS, self.update_gui)


    def open_maps(self):
        """Open the current coordinates in the default web browser using Google Maps."""
        if not self.gps_reader: return
        data = self.gps_reader.get_current_data()
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)

        if lat != 0.0 and lon != 0.0:
            # Standard Google Maps URL
            maps_url = f"https://www.google.com/maps?q={lat},{lon}&z=16" # z=16 for zoom level
            try:
                webbrowser.open(maps_url)
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
            self.sat_window.lift() # Bring to front if already open
        else:
            self.create_satellite_window() # Create if not existing or closed

    def create_satellite_window(self):
        """Create the Toplevel window for satellite information."""
        if not self.gps_reader:
            messagebox.showinfo("Info", "Connect to GPS device first.")
            return

        self.sat_window = tk.Toplevel(self.root)
        self.sat_window.title("Satellite Information")
        self.sat_window.geometry("650x450")
        self.sat_window.minsize(500, 300)
        # Handle window close event
        self.sat_window.protocol("WM_DELETE_WINDOW", self.close_satellite_window)

        # Frame for Treeview and Scrollbar
        tree_frame = ttk.Frame(self.sat_window, padding=5)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Treeview widget
        columns = ("prn", "gnss", "elev", "azim", "snr", "used")
        self.sat_tree = ttk.Treeview(tree_frame, columns=columns, show="headings")

        # Define headings and column properties
        self.sat_tree.heading("prn", text="PRN")
        self.sat_tree.column("prn", width=50, anchor=tk.CENTER)
        self.sat_tree.heading("gnss", text="System")
        self.sat_tree.column("gnss", width=70, anchor=tk.W)
        self.sat_tree.heading("elev", text="Elevation (°)")
        self.sat_tree.column("elev", width=80, anchor=tk.CENTER)
        self.sat_tree.heading("azim", text="Azimuth (°)")
        self.sat_tree.column("azim", width=80, anchor=tk.CENTER)
        self.sat_tree.heading("snr", text="SNR (dB)")
        self.sat_tree.column("snr", width=60, anchor=tk.CENTER)
        self.sat_tree.heading("used", text="Used in Fix")
        self.sat_tree.column("used", width=80, anchor=tk.CENTER)

        # Scrollbar
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.sat_tree.yview)
        self.sat_tree.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.sat_tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Configure the 'Used' tag for highlighting
        self.sat_tree.tag_configure('Used', foreground='green', font=('Helvetica', 9, 'bold'))
        self.sat_tree.tag_configure('NotUsed', foreground='gray')

        # Frame for Summary Info below tree
        summary_frame = ttk.LabelFrame(self.sat_window, text="Summary", padding=5)
        summary_frame.pack(fill=tk.X, padx=5, pady=5)
        summary_frame.columnconfigure(1, weight=1)
        summary_frame.columnconfigure(3, weight=1)

        # Add summary labels (use instance variables to update them)
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

    def update_satellite_window(self):
        """Update the satellite treeview and summary labels with current data."""
        if not self.sat_window or not self.sat_window.winfo_exists() or not self.sat_tree:
            # print("DEBUG: Satellite window or tree does not exist for update.")
            return
        if not self.gps_reader:
             print("DEBUG: No GPS reader for satellite update.")
             return

        data = self.gps_reader.get_current_data()
        sat_data = data.get('satellite_data', [])
        sats_used = data.get('satellites_used', [])

        # --- Update Treeview ---
        # Store current selection and scroll position
        selected_item = self.sat_tree.focus()
        scroll_pos = self.sat_tree.yview()

        self.sat_tree.delete(*self.sat_tree.get_children()) # Clear existing items

        # Sort satellites by PRN for consistent display
        sat_data.sort(key=lambda x: x.get('prn', 999))

        for sat in sat_data:
            prn = sat.get('prn', '--')
            elev = sat.get('elevation', '--')
            azim = sat.get('azimuth', '--')
            snr = sat.get('snr', '--')
            used = prn in sats_used if prn != '--' else False
            used_text = "Yes" if used else "No"
            tag = 'Used' if used else 'NotUsed'

            # Determine GNSS System (more robustly if possible)
            gnss = "Unknown"
            if isinstance(prn, int):
                # Example ranges (adjust as needed, consult NMEA specs or receiver docs)
                if 1 <= prn <= 32: gnss = "GPS"           # GPS L1
                elif 33 <= prn <= 64: gnss = "SBAS"       # WAAS, EGNOS, etc.
                elif 65 <= prn <= 96: gnss = "GLONASS"    # GLONASS L1 OF
                elif 120 <= prn <= 140: gnss = "SBAS"     # More SBAS numbers
                elif 193 <= prn <= 197: gnss = "QZSS"     # Japan QZSS L1
                elif 201 <= prn <= 235: gnss = "Galileo"  # Galileo E1
                elif 301 <= prn <= 336: gnss = "BeiDou"   # BeiDou B1
                # Add more ranges if needed...
            else: # Handle '--' or other non-int PRN
                prn = str(prn)

            values = (prn, gnss, elev, azim, snr, used_text)
            try:
                self.sat_tree.insert("", tk.END, values=values, tags=(tag,))
            except Exception as tree_err:
                print(f"ERROR inserting into satellite tree: {tree_err} for values {values}")


        # Restore selection and scroll position if possible
        if selected_item and self.sat_tree.exists(selected_item):
             try: # Item might disappear between updates
                 self.sat_tree.focus(selected_item)
                 self.sat_tree.selection_set(selected_item)
             except tk.TclError: pass # Item no longer exists
        self.sat_tree.yview_moveto(scroll_pos[0])


        # --- Update Summary Labels ---
        gnss_systems = data.get('gnss_systems_used', set())
        self.summary_gnss.config(text=", ".join(sorted(list(gnss_systems))) if gnss_systems else "None")

        sats_view = data.get('satellites_in_view', 0)
        self.summary_sats.config(text=f"{sats_view} / {len(sats_used)}")

        hdop = data.get('hdop', 0.0)
        pdop = data.get('pdop', 0.0)
        vdop = data.get('vdop', 0.0)
        self.summary_dop.config(text=f"{hdop:.2f} / {pdop:.2f} / {vdop:.2f}" if hdop > 0 else "-- / -- / --")

        accuracy = data.get('accuracy_estimate', 0.0)
        self.summary_accuracy.config(text=f"~ {accuracy:.2f} m" if accuracy > 0 else "--")

        # Update window title with timestamp
        now = datetime.datetime.now().strftime('%H:%M:%S')
        self.sat_window.title(f"Satellite Information (Updated: {now})")


    def close_satellite_window(self):
        """Callback when the satellite window is closed by the user."""
        if self.sat_window:
            self.sat_window.destroy()
        self.sat_window = None
        self.sat_tree = None


    # --- Application Exit ---
    def on_close(self):
        """Perform cleanup actions before closing the application."""
        print("Closing application...")
        if self.gui_update_after_id:
            self.root.after_cancel(self.gui_update_after_id)
        if self.gps_reader:
            self.gps_reader.stop()
        self.root.quit() # Stop Tkinter main loop
        self.root.destroy() # Destroy the window
        print("Application closed.")


# --- Main Execution ---
def main():
    """Main function to create and run the Tkinter application."""
    root = tk.Tk()
    app = GPSApp(root)
    # Set the close protocol handler
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    # Add platform-specific settings if needed
    if platform.system() == "Windows":
         # Optional: Set DPI awareness for sharper UI on high-res displays
         try:
             from ctypes import windll
             windll.shcore.SetProcessDpiAwareness(1)
         except Exception:
             pass # Ignore if it fails
    main()