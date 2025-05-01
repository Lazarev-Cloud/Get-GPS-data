"""GPS reader module for handling GPS device communication and data processing."""

import threading
import time
import datetime
import serial
from typing import Optional, Dict, Any, List, Tuple, Set, Union

from config.settings import DEFAULT_BAUD_RATE, MAX_POSITION_HISTORY
from config.settings import SMOOTHING_NONE, SMOOTHING_AVG, SMOOTHING_KALMAN
from config.settings import SMOOTHING_METHODS, NUMPY_AVAILABLE
from core.nmea_parser import NMEAParser
from core.filters.moving_avg import MovingAverageFilter
from logging.data_logger import DataLogger

# Import Kalman filter if available
if NUMPY_AVAILABLE:
    from core.filters.kalman import KalmanFilter

class GPSReader:
    """Handles serial communication with GPS device and data processing."""

    def __init__(self, port: str, baud: int = DEFAULT_BAUD_RATE):
        """Initialize GPS reader.
        
        Args:
            port: Serial port name (e.g. COM3, /dev/ttyUSB0)
            baud: Baud rate for serial communication
        """
        self.port = port
        self.baud = baud
        self.ser: Optional[serial.Serial] = None
        self.running = threading.Event()
        self.read_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()  # Lock for accessing shared data

        # Data storage
        self.current_data: Dict[str, Any] = self._get_default_data()
        
        # Parsers and processors
        self.nmea_parser = NMEAParser(self)
        
        # Position filters
        self.smoothing_method = SMOOTHING_NONE
        self.moving_avg_filter = MovingAverageFilter(max_history_size=MAX_POSITION_HISTORY)
        self.kalman_filter = KalmanFilter() if NUMPY_AVAILABLE else None
        
        # Logger
        self.data_logger = DataLogger()

    def _get_default_data(self) -> Dict[str, Any]:
        """Return a dictionary with initial default values for GPS data.
        
        Returns:
            Dictionary with default GPS data values
        """
        return {
            'host_timestamp': None, 'fix_timestamp': None,
            'lat': 0.0, 'lon': 0.0, 'altitude': 0.0,
            'raw_lat': 0.0, 'raw_lon': 0.0, 'raw_alt': 0.0,  # Store raw values
            'speed_kph': 0.0,
            'satellites_in_view': 0, 'satellites_used': [],  # List of PRNs
            'fix_quality': 0, 'hdop': 0.0, 'pdop': 0.0, 'vdop': 0.0,
            'geoid_height': 0.0, 'fix_type': 0,
            'satellite_data': [],  # Detailed list of dicts for satellites in view
            'gnss_systems_used': set(),  # Set of strings like {"GPS", "GLONASS"}
            'accuracy_estimate': 0.0, 'raw_sentence': '',
        }

    def connect(self) -> bool:
        """Connect to the serial port.
        
        Returns:
            True if connection successful, False otherwise
        """
        if self.ser and self.ser.is_open:
            print("Already connected.")
            return True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Connected to {self.port} at {self.baud} baud")
            time.sleep(0.5)  # Allow device to settle
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
        """Send configuration commands to u-blox modules."""
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

    def start_logging(self, filename: str) -> bool:
        """Start logging GPS data to a CSV file.
        
        Args:
            filename: Path to the log file
            
        Returns:
            True if logging started successfully, False otherwise
        """
        return self.data_logger.start_logging(filename)

    def stop_logging(self):
        """Stop logging GPS data."""
        self.data_logger.stop_logging()

    def _read_loop(self):
        """Background thread task to continuously read from serial port."""
        print("Read thread started.")
        while self.running.is_set():
            line = None  # Ensure line is reset each loop
            try:
                if self.ser and self.ser.is_open:
                    # Read one line, decode safely
                    try:
                        raw_line = self.ser.readline()
                        if raw_line:
                            line = raw_line.decode('ascii', errors='replace').strip()
                        else:
                            # Timeout occurred on readline, just loop again
                            time.sleep(0.01)  # Small sleep on timeout
                            continue

                    except serial.SerialException as serial_err:
                        print(f"ERROR: Serial read error: {serial_err}. Stopping reader.")
                        self.running.clear()  # Signal stop
                        break  # Exit read loop
                    except UnicodeDecodeError:
                        continue  # Skip undecodable data
                    except Exception as read_err:
                        print(f"ERROR: Unexpected read error: {read_err}")
                        time.sleep(0.1)  # Avoid busy-looping
                        continue

                    if line:
                        self.nmea_parser.parse_sentence(line)

                else:
                    # Serial port not open/available, wait before checking again
                    print("Read loop: Serial port not open.")
                    time.sleep(1.0)  # Wait longer if port closed unexpectedly

            except Exception as loop_err:
                print(f"ERROR: Unhandled exception in read loop: {loop_err}")
                import traceback
                traceback.print_exc()
                time.sleep(1)  # Avoid rapid error loops

        print("Read thread finished.")
        # Ensure log file is closed properly when thread stops
        self.stop_logging()

    def start(self) -> bool:
        """Start the GPS reading thread.
        
        Returns:
            True if started successfully, False otherwise
        """
        if self.running.is_set():
            print("Reader already running.")
            return True
            
        # Reset data to defaults before starting
        with self.lock:
            self.current_data = self._get_default_data()
            self.moving_avg_filter.reset()
            if self.kalman_filter:
                self.kalman_filter.reset()

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
        self.running.clear()  # Signal thread to stop

        if self.read_thread and self.read_thread.is_alive():
            self.read_thread.join(timeout=2.0)  # Wait for thread to finish
            if self.read_thread.is_alive():
                print("WARN: Read thread did not terminate gracefully.")

        self.disconnect()
        self.read_thread = None
        print("GPS reader stopped.")

    def get_current_data(self) -> Dict[str, Any]:
        """Return a copy of the latest GPS data (thread-safe).
        
        Returns:
            Dictionary with current GPS data
        """
        with self.lock:
            return self.current_data.copy()

    def set_smoothing(self, method: str, history_size: Optional[int] = None):
        """Set the position smoothing method.
        
        Args:
            method: Smoothing method name ('None', 'Moving Average', 'Kalman Filter')
            history_size: Optional history size for moving average filter
        """
        with self.lock:
            if method in SMOOTHING_METHODS:
                # Clear history/reset filter when changing method
                self.moving_avg_filter.reset()
                if self.kalman_filter:
                    self.kalman_filter.reset()

                self.smoothing_method = method
                print(f"Smoothing method set to: {method}")

                if method == SMOOTHING_AVG and history_size is not None:
                    self.moving_avg_filter.set_params(max_history_size=max(1, history_size))
                    print(f"Moving average history size set to: {history_size}")
                elif method == SMOOTHING_KALMAN:
                    if not NUMPY_AVAILABLE:
                        print("WARN: Cannot use Kalman filter, NumPy not installed.")
                        self.smoothing_method = SMOOTHING_NONE  # Fallback
            else:
                print(f"WARN: Invalid smoothing method '{method}' requested.")