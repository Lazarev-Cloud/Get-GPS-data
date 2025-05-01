import serial
import pynmea2
import time
import threading
import tkinter as tk
from tkinter import ttk
import datetime
import os
import csv
import webbrowser
import serial.tools.list_ports


class GPSReader:
    def __init__(self, port, baud=9600):
        """Initialize GPS reader with serial port and baud rate."""
        self.port = port
        self.baud = baud
        self.ser = None
        self.running = False
        self.data = {
            'timestamp': None,
            'lat': 0.0,
            'lon': 0.0,
            'altitude': 0.0,
            'speed': 0.0,
            'satellites': 0,
            'fix_quality': 0,
            'hdop': 0.0,
            'pdop': 0.0,  # Position Dilution of Precision
            'vdop': 0.0,  # Vertical Dilution of Precision
            'geoid_height': 0.0,  # Height of geoid above WGS84 ellipsoid
            'fix_type': 0,  # 0=no fix, 1=GPS fix, 2=DGPS fix, 3=PPS fix
            'satellite_data': [],  # Detailed satellite info
            'gnss_mode': '',  # GPS, GLONASS, GALILEO, etc.
            'gnss_systems': [],  # Active GNSS systems
            'accuracy': 0.0,  # Estimated horizontal accuracy in meters
            'raw_sentence': ''
        }
        self.position_history = []  # Store recent positions for averaging
        self.max_history_size = 10  # Maximum number of positions to keep
        self.log_file = None
        self.writer = None
        self.log_enabled = False
        self.kalman_filter = None
        self.initialize_kalman_filter()

    def initialize_kalman_filter(self):
        """Initialize Kalman filter for position smoothing."""
        try:
            import numpy as np

            # Initial state [lat, lon, alt, lat_velocity, lon_velocity, alt_velocity]
            self.kalman_filter = {
                'x': np.zeros(6),  # State vector
                'P': np.eye(6) * 100,  # Covariance matrix
                'F': np.array([  # State transition matrix
                    [1, 0, 0, 1, 0, 0],
                    [0, 1, 0, 0, 1, 0],
                    [0, 0, 1, 0, 0, 1],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]
                ]),
                'H': np.array([  # Measurement matrix
                    [1, 0, 0, 0, 0, 0],
                    [0, 1, 0, 0, 0, 0],
                    [0, 0, 1, 0, 0, 0]
                ]),
                'Q': np.eye(6) * 0.01,  # Process noise
                'R': np.eye(3) * 10.0  # Measurement noise
            }
        except ImportError:
            # If numpy is not available, disable Kalman filtering
            self.kalman_filter = None
            print("NumPy not available, Kalman filtering disabled")

    def apply_kalman_filter(self, lat, lon, alt):
        """Apply Kalman filter to smooth position data."""
        if self.kalman_filter is None:
            return lat, lon, alt

        try:
            import numpy as np

            # Skip if any values are invalid
            if lat == 0 or lon == 0:
                return lat, lon, alt

            # Predict step
            x = self.kalman_filter['x']
            P = self.kalman_filter['P']
            F = self.kalman_filter['F']
            Q = self.kalman_filter['Q']

            x = F @ x
            P = F @ P @ F.T + Q

            # Update step
            z = np.array([lat, lon, alt])
            H = self.kalman_filter['H']
            R = self.kalman_filter['R']

            y = z - H @ x
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)

            x = x + K @ y
            P = (np.eye(6) - K @ H) @ P

            # Save updated state
            self.kalman_filter['x'] = x
            self.kalman_filter['P'] = P

            # Return smoothed values
            return float(x[0]), float(x[1]), float(x[2])
        except Exception as e:
            print(f"Kalman filter error: {e}")
            return lat, lon, alt

    def add_to_position_history(self, lat, lon, alt):
        """Add position to history for averaging."""
        if lat == 0 or lon == 0:
            return

        self.position_history.append((lat, lon, alt))
        if len(self.position_history) > self.max_history_size:
            self.position_history.pop(0)

    def get_averaged_position(self):
        """Calculate weighted moving average of recent positions."""
        if not self.position_history:
            return 0.0, 0.0, 0.0

        # More weight to recent readings
        weights = [i + 1 for i in range(len(self.position_history))]
        total_weight = sum(weights)

        avg_lat = sum(pos[0] * w for pos, w in zip(self.position_history, weights)) / total_weight
        avg_lon = sum(pos[1] * w for pos, w in zip(self.position_history, weights)) / total_weight
        avg_alt = sum(pos[2] * w for pos, w in zip(self.position_history, weights)) / total_weight

        return avg_lat, avg_lon, avg_alt

    def connect(self):
        """Connect to the GPS module via serial port."""
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            print(f"Connected to {self.port} at {self.baud} baud")

            # Send configuration commands to GPS module to enhance accuracy
            self._configure_gps_module()

            return True
        except serial.SerialException as e:
            print(f"Error connecting to serial port: {e}")
            return False

    def _configure_gps_module(self):
        """Send configuration commands to optimize GPS module settings."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            # Wait for GPS module to initialize
            time.sleep(1)

            # UBX commands for u-blox NEO-6M/7M/8M GPS modules
            # These are binary commands to configure various module settings

            # 1. Enable SBAS (Satellite Based Augmentation System)
            # This improves accuracy by using correction data from satellite systems like WAAS, EGNOS, etc.
            sbas_enable = bytearray.fromhex("B5 62 06 16 08 00 01 01 03 00 00 00 00 00 29 8B")
            self.ser.write(sbas_enable)
            time.sleep(0.1)

            # 2. Set update rate to 5Hz (200ms) for more frequent updates
            # Default is 1Hz, but 5Hz gives better tracking for moving objects
            update_rate = bytearray.fromhex("B5 62 06 08 06 00 C8 00 01 00 01 00 DE 6A")
            self.ser.write(update_rate)
            time.sleep(0.1)

            # 3. Enable all GNSS systems (GPS, GLONASS, Galileo, BeiDou)
            # More satellite systems = better accuracy
            # Note: Not all modules support all systems, but NEO-8M supports most
            gnss_config = bytearray.fromhex(
                "B5 62 06 3E 3C 00 00 00 20 07 00 08 10 00 01 00 01 01 01 01 03 00 00 00 01 01 02 04 08 00 00 00 01 01 03 08 10 00 00 00 01 01 04 00 08 00 00 00 01 03 05 00 03 00 00 00 01 05 06 08 0E 00 01 00 01 01 5F 19")
            self.ser.write(gnss_config)
            time.sleep(0.1)

            # 4. Configure to output more NMEA sentences for better data
            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 00 01 01 01 01 01 01 04 4B")  # GGA
            self.ser.write(nmea_config)
            time.sleep(0.1)

            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 01 01 01 01 01 01 01 05 50")  # GLL
            self.ser.write(nmea_config)
            time.sleep(0.1)

            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 02 01 01 01 01 01 01 06 55")  # GSA
            self.ser.write(nmea_config)
            time.sleep(0.1)

            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 03 01 01 01 01 01 01 07 5A")  # GSV
            self.ser.write(nmea_config)
            time.sleep(0.1)

            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 04 01 01 01 01 01 01 08 5F")  # RMC
            self.ser.write(nmea_config)
            time.sleep(0.1)

            nmea_config = bytearray.fromhex("B5 62 06 01 08 00 F0 05 01 01 01 01 01 01 09 64")  # VTG
            self.ser.write(nmea_config)
            time.sleep(0.1)

            print("GPS module configured for optimal accuracy")

        except Exception as e:
            print(f"Error configuring GPS module: {e}")
            # Continue even if configuration fails

    def disconnect(self):
        """Disconnect from the GPS module."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected from GPS")

    def start_logging(self, filename=None):
        """Start logging GPS data to a CSV file."""
        if filename is None:
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"gps_log_{timestamp}.csv"

        try:
            self.log_file = open(filename, 'w', newline='')
            self.writer = csv.writer(self.log_file)
            self.writer.writerow([
                'Timestamp', 'Latitude', 'Longitude', 'Altitude',
                'Speed', 'Satellites', 'Fix Quality', 'HDOP', 'PDOP', 'VDOP',
                'Fix Type', 'Accuracy', 'Geoid Height', 'GNSS Systems'
            ])
            self.log_enabled = True
            print(f"Logging started to {filename}")
        except Exception as e:
            print(f"Error starting log: {e}")
            self.log_enabled = False

    def stop_logging(self):
        """Stop logging GPS data."""
        if self.log_file:
            self.log_file.close()
            self.log_enabled = False
            print("Logging stopped")

    def _parse_gps_data(self, data):
        """Parse NMEA sentence and extract relevant GPS data."""
        try:
            msg = pynmea2.parse(data)

            # Parse different NMEA sentence types
            if isinstance(msg, pynmea2.GGA):
                # Global Positioning System Fix Data
                self.data['timestamp'] = msg.timestamp
                if msg.latitude:
                    self.data['lat'] = msg.latitude
                if msg.longitude:
                    self.data['lon'] = msg.longitude
                if msg.altitude:
                    self.data['altitude'] = msg.altitude
                self.data['fix_quality'] = msg.gps_qual
                if msg.num_sats:
                    self.data['satellites'] = msg.num_sats
                if msg.horizontal_dil:
                    self.data['hdop'] = msg.horizontal_dil
                if hasattr(msg, 'geo_sep') and msg.geo_sep:
                    self.data['geoid_height'] = msg.geo_sep
                self.data['raw_sentence'] = data

                # Apply filtering for smoothing
                lat, lon, alt = self.apply_kalman_filter(
                    float(msg.latitude) if msg.latitude else 0.0,
                    float(msg.longitude) if msg.longitude else 0.0,
                    float(msg.altitude) if msg.altitude else 0.0
                )

                # Add to position history
                self.add_to_position_history(lat, lon, alt)

                # Use filtered data
                self.data['lat'] = lat
                self.data['lon'] = lon
                self.data['altitude'] = alt

            elif isinstance(msg, pynmea2.RMC):
                # Recommended Minimum Specific GPS/Transit Data
                self.data['timestamp'] = msg.timestamp
                if msg.latitude:
                    self.data['lat'] = msg.latitude
                if msg.longitude:
                    self.data['lon'] = msg.longitude
                if msg.spd_over_grnd:
                    self.data['speed'] = msg.spd_over_grnd * 1.852  # Convert knots to km/h

            elif isinstance(msg, pynmea2.GSV):
                # GPS Satellites in View
                if msg.num_sv_in_view:
                    try:
                        self.data['satellites'] = int(msg.num_sv_in_view)
                    except (ValueError, TypeError):
                        # If conversion fails, keep existing value
                        pass

                # Extract detailed satellite info
                satellites = []
                for i in range(1, 5):  # Each GSV can contain up to 4 satellites
                    try:
                        prn = getattr(msg, f'sv_prn_{i}', None)
                        if prn:
                            elevation = getattr(msg, f'elevation_{i}', None)
                            azimuth = getattr(msg, f'azimuth_{i}', None)
                            snr = getattr(msg, f'snr_{i}', None)
                            satellites.append({
                                'prn': prn,
                                'elevation': elevation,
                                'azimuth': azimuth,
                                'snr': snr
                            })
                    except Exception as e:
                        print(f"Warning: Error processing satellite {i} data: {e}")

                # Append to existing satellite data
                self.data['satellite_data'].extend(satellites)
                # Limit to most recent 32 satellites to avoid unlimited growth
                self.data['satellite_data'] = self.data['satellite_data'][-32:]

            elif isinstance(msg, pynmea2.GSA):
                # GPS DOP and Active Satellites
                try:
                    if hasattr(msg, 'mode_fix_type') and msg.mode_fix_type:
                        try:
                            self.data['fix_type'] = int(msg.mode_fix_type)
                        except (ValueError, TypeError):
                            # If conversion fails, keep existing value
                            pass

                    if hasattr(msg, 'pdop') and msg.pdop:
                        try:
                            self.data['pdop'] = float(msg.pdop)
                        except (ValueError, TypeError):
                            pass

                    if hasattr(msg, 'hdop') and msg.hdop:
                        try:
                            self.data['hdop'] = float(msg.hdop)
                        except (ValueError, TypeError):
                            pass

                    if hasattr(msg, 'vdop') and msg.vdop:
                        try:
                            self.data['vdop'] = float(msg.vdop)
                        except (ValueError, TypeError):
                            pass

                    # Determine active GNSS system
                    gnss_id = getattr(msg, 'sv_id01', '')
                    if gnss_id:
                        # Convert to string and strip whitespace
                        gnss_id = str(gnss_id).strip()
                        first_char = gnss_id[0] if len(gnss_id) > 0 else ''

                        # Determine GNSS system based on ID
                        if first_char.isdigit() and first_char in "01456789":
                            gnss = "GPS"
                        elif first_char == '3':
                            gnss = "SBAS"
                        elif first_char == '4':
                            gnss = "EGNOS"
                        elif first_char == '8':
                            gnss = "GLONASS"
                        elif first_char == '2':
                            gnss = "GALILEO"
                        elif first_char == '5':
                            gnss = "QZSS"
                        elif first_char == '3':
                            gnss = "BEIDOU"
                        else:
                            gnss = "UNKNOWN"

                        if gnss not in self.data['gnss_systems']:
                            self.data['gnss_systems'].append(gnss)
                except Exception as e:
                    print(f"Warning: Error processing GSA data: {e}")

            elif isinstance(msg, pynmea2.GLL):
                # Geographic Position - Latitude/Longitude
                if msg.latitude:
                    self.data['lat'] = msg.latitude
                if msg.longitude:
                    self.data['lon'] = msg.longitude

            elif isinstance(msg, pynmea2.VTG):
                # Track Made Good and Ground Speed
                if hasattr(msg, 'spd_over_grnd_kmph'):
                    self.data['speed'] = float(msg.spd_over_grnd_kmph) if msg.spd_over_grnd_kmph else 0.0

            # Calculate position accuracy based on HDOP and satellites
            try:
                hdop_value = float(self.data['hdop']) if isinstance(self.data['hdop'], (int, float, str)) else 0
                satellites_value = int(self.data['satellites']) if isinstance(self.data['satellites'],
                                                                              (int, float, str)) else 0

                if hdop_value > 0 and satellites_value > 0:
                    # Rough estimate: HDOP × a base precision factor
                    base_precision = 2.5  # meters, typical GPS accuracy under good conditions
                    self.data['accuracy'] = hdop_value * base_precision

                    # Adjust based on number of satellites
                    if satellites_value < 4:
                        self.data['accuracy'] *= 2.0  # Poor accuracy with few satellites
                    elif satellites_value > 8:
                        self.data['accuracy'] *= 0.7  # Better accuracy with many satellites
            except (ValueError, TypeError) as e:
                # If conversion fails, just skip accuracy calculation for this update
                print(f"Warning: Could not calculate accuracy: {e}")

            return True
        except pynmea2.ParseError:
            # Not all data from GPS will be valid NMEA sentences
            return False
        except Exception as e:
            print(f"Error parsing GPS data: {e}")
            return False

    def _read_thread(self):
        """Background thread to continuously read GPS data."""
        while self.running:
            try:
                if self.ser and self.ser.is_open:
                    line = self.ser.readline().decode('ascii', errors='replace').strip()
                    if line:
                        if self._parse_gps_data(line) and self.log_enabled:
                            # Log data if we successfully parsed a GPS sentence
                            self.writer.writerow([
                                datetime.datetime.now(),
                                self.data['lat'],
                                self.data['lon'],
                                self.data['altitude'],
                                self.data['speed'],
                                self.data['satellites'],
                                self.data['fix_quality'],
                                self.data['hdop'],
                                self.data['pdop'],
                                self.data['vdop'],
                                self.data['fix_type'],
                                self.data['accuracy'],
                                self.data['geoid_height'],
                                ','.join(self.data['gnss_systems'])
                            ])

                            # Flush to ensure data is written immediately
                            if self.log_file:
                                self.log_file.flush()
            except Exception as e:
                print(f"Error reading GPS data: {e}")
                time.sleep(1)

    def start(self):
        """Start reading GPS data in a background thread."""
        if not self.ser:
            if not self.connect():
                return False

        self.running = True
        self.thread = threading.Thread(target=self._read_thread)
        self.thread.daemon = True
        self.thread.start()
        return True

    def stop(self):
        """Stop reading GPS data."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        self.disconnect()
        if self.log_enabled:
            self.stop_logging()


class GPSApp:
    def __init__(self, root):
        """Initialize the GPS application with GUI elements."""
        self.root = root
        self.root.title("Enhanced GPS Data Display")
        self.root.geometry("800x650")

        self.gps = None
        self.update_interval = 1000  # Update GUI every 1000ms

        self._create_widgets()

    def _create_widgets(self):
        """Create the GUI elements."""
        # Frame for connection controls
        conn_frame = ttk.LabelFrame(self.root, text="Connection")
        conn_frame.pack(padx=10, pady=10, fill=tk.X)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)

        # Get available ports for dropdown
        available_ports = self._get_available_ports()
        self.port_combo = ttk.Combobox(conn_frame, values=available_ports, width=15)
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        # Set default selection if available
        if available_ports:
            self.port_combo.current(len(available_ports) - 1)  # Select last port, often the most recently added

        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.baud_combo = ttk.Combobox(conn_frame, values=["4800", "9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        self.baud_combo.current(1)  # Select 9600 by default

        # Add refresh button for port list
        self.refresh_btn = ttk.Button(conn_frame, text="⟳", width=2, command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=4, padx=2, pady=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        self.log_btn = ttk.Button(conn_frame, text="Start Logging", command=self.toggle_logging)
        self.log_btn.grid(row=0, column=6, padx=5, pady=5)
        self.log_btn.state(['disabled'])

        # Add accuracy settings frame
        accuracy_frame = ttk.LabelFrame(self.root, text="Accuracy Settings")
        accuracy_frame.pack(padx=10, pady=5, fill=tk.X)

        # Position smoothing option
        ttk.Label(accuracy_frame, text="Position Smoothing:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.smoothing_var = tk.StringVar(value="Kalman")
        smoothing_options = ttk.Combobox(accuracy_frame, textvariable=self.smoothing_var,
                                         values=["None", "Moving Average", "Kalman"])
        smoothing_options.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)

        # History size for averaging
        ttk.Label(accuracy_frame, text="History Size:").grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.history_var = tk.StringVar(value="10")
        history_spin = ttk.Spinbox(accuracy_frame, from_=1, to=30, width=5, textvariable=self.history_var)
        history_spin.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)

        # Apply settings button
        self.apply_btn = ttk.Button(accuracy_frame, text="Apply Settings", command=self.apply_accuracy_settings)
        self.apply_btn.grid(row=0, column=4, padx=5, pady=5)
        self.apply_btn.state(['disabled'])

        # Frame for GPS data display
        data_frame = ttk.LabelFrame(self.root, text="GPS Data")
        data_frame.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)

        # Add maps button
        self.map_btn = ttk.Button(data_frame, text="View on Map", command=self.open_maps)
        self.map_btn.grid(row=0, column=2, padx=5, pady=5, rowspan=2)
        self.map_btn.state(['disabled'])  # Disable until we have valid coordinates

        # Button to view satellite info
        self.sat_btn = ttk.Button(data_frame, text="Satellite Info", command=self.show_satellite_info)
        self.sat_btn.grid(row=2, column=2, padx=5, pady=5, rowspan=2)
        self.sat_btn.state(['disabled'])

        # GPS data labels
        self.data_labels = {}
        data_fields = [
            ("lat", "Latitude:", 0),
            ("lon", "Longitude:", 1),
            ("altitude", "Altitude (m):", 2),
            ("speed", "Speed (km/h):", 3),
            ("satellites", "Satellites:", 4),
            ("fix_quality", "Fix Quality:", 5),
            ("hdop", "HDOP:", 6),
            ("pdop", "PDOP:", 7),
            ("vdop", "VDOP:", 8),
            ("accuracy", "Accuracy (m):", 9),
            ("fix_type", "Fix Type:", 10),
            ("gnss_mode", "GNSS Mode:", 11),
            ("timestamp", "Time:", 12),
        ]

        for field, label, row in data_fields:
            ttk.Label(data_frame, text=label).grid(row=row, column=0, padx=5, pady=5, sticky=tk.W)
            self.data_labels[field] = ttk.Label(data_frame, text="--")
            self.data_labels[field].grid(row=row, column=1, padx=5, pady=5, sticky=tk.W)

        # Raw NMEA data display
        ttk.Label(data_frame, text="Raw NMEA:").grid(row=13, column=0, padx=5, pady=5, sticky=tk.W)
        self.nmea_display = tk.Text(data_frame, height=5, width=60, wrap=tk.WORD)
        self.nmea_display.grid(row=14, column=0, columnspan=3, padx=5, pady=5, sticky=tk.W + tk.E)

        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Not connected")
        self.status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

    def _get_available_ports(self):
        """Get a list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        port_list = []
        for port in ports:
            # Format as "COM5: USB-SERIAL CH341"
            port_list.append(f"{port.device}: {port.description}")
        return port_list

    def refresh_ports(self):
        """Refresh the list of available ports."""
        available_ports = self._get_available_ports()
        self.port_combo['values'] = available_ports
        if available_ports:
            self.port_combo.current(len(available_ports) - 1)
        self.status_var.set("Port list refreshed")

    def toggle_connection(self):
        """Connect to or disconnect from the GPS."""
        if self.gps and self.gps.running:
            # Disconnect
            self.gps.stop()
            self.gps = None
            self.connect_btn.configure(text="Connect")
            self.log_btn.state(['disabled'])
            self.status_var.set("Disconnected")
            self.after_id = None
        else:
            # Connect - extract port name from the combo box selection
            port_with_desc = self.port_combo.get()
            port = port_with_desc.split(':')[0].strip() if ':' in port_with_desc else port_with_desc

            baud = int(self.baud_combo.get())

            self.gps = GPSReader(port, baud)
            if self.gps.start():
                self.connect_btn.configure(text="Disconnect")
                self.log_btn.state(['!disabled'])
                self.status_var.set(f"Connected to {port}")
                self.after_id = self.root.after(self.update_interval, self.update_data)
            else:
                self.gps = None
                self.status_var.set(f"Failed to connect to {port}")

    def toggle_logging(self):
        """Start or stop logging GPS data."""
        if not self.gps:
            return

        if self.gps.log_enabled:
            self.gps.stop_logging()
            self.log_btn.configure(text="Start Logging")
            self.status_var.set("Logging stopped")
        else:
            self.gps.start_logging()
            self.log_btn.configure(text="Stop Logging")
            self.status_var.set("Logging started")

    def apply_accuracy_settings(self):
        """Apply selected accuracy enhancement settings."""
        if not self.gps:
            return

        try:
            # Get settings from UI
            smoothing_method = self.smoothing_var.get()
            history_size = int(self.history_var.get())

            # Update GPS reader settings
            self.gps.max_history_size = history_size

            # Re-initialize Kalman filter if needed
            if smoothing_method == "Kalman":
                self.gps.initialize_kalman_filter()

            self.status_var.set(f"Applied accuracy settings: {smoothing_method}, History: {history_size}")
        except Exception as e:
            self.status_var.set(f"Error applying settings: {e}")

    def show_satellite_info(self):
        """Display detailed satellite information in a new window."""
        if not self.gps or not hasattr(self.gps, 'data'):
            return

        # Create new window
        sat_window = tk.Toplevel(self.root)
        sat_window.title("Satellite Information")
        sat_window.geometry("600x400")

        # Create a frame for the satellite info
        frame = ttk.Frame(sat_window, padding=10)
        frame.pack(fill=tk.BOTH, expand=True)

        # Create a treeview to display satellite data
        columns = ("PRN", "Elevation", "Azimuth", "SNR", "Used")
        tree = ttk.Treeview(frame, columns=columns, show="headings")

        # Define column headings
        for col in columns:
            tree.heading(col, text=col)
            tree.column(col, width=100, anchor=tk.CENTER)

        # Add scrollbar
        scrollbar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=tree.yview)
        tree.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Populate with satellite data
        sat_data = self.gps.data.get('satellite_data', [])
        for i, sat in enumerate(sat_data):
            try:
                prn = sat.get('prn', '--')
                elevation = sat.get('elevation', '--')
                azimuth = sat.get('azimuth', '--')
                snr = sat.get('snr', '--')

                # Safely determine if satellite is used based on SNR
                used = "No"
                if snr and snr != '--':
                    try:
                        if int(snr) > 30:
                            used = "Yes"
                    except (ValueError, TypeError):
                        pass

                tree.insert("", tk.END, values=(prn, elevation, azimuth, snr, used))
            except Exception as e:
                print(f"Error adding satellite to tree: {e}")

        # Add summary at the bottom
        summary_frame = ttk.LabelFrame(sat_window, text="Summary")
        summary_frame.pack(padx=10, pady=10, fill=tk.X)

        # GNSS systems in use
        gnss_systems = self.gps.data.get('gnss_systems', [])
        ttk.Label(summary_frame, text="Active GNSS Systems:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        ttk.Label(summary_frame, text=", ".join(gnss_systems) if gnss_systems else "None").grid(row=0, column=1, padx=5,
                                                                                                pady=5, sticky=tk.W)

        # DOP values
        ttk.Label(summary_frame, text="HDOP/PDOP/VDOP:").grid(row=1, column=0, padx=5, pady=5, sticky=tk.W)
        dop_values = f"{self.gps.data.get('hdop', '--')}/{self.gps.data.get('pdop', '--')}/{self.gps.data.get('vdop', '--')}"
        ttk.Label(summary_frame, text=dop_values).grid(row=1, column=1, padx=5, pady=5, sticky=tk.W)

        # Estimated accuracy
        ttk.Label(summary_frame, text="Estimated Accuracy:").grid(row=2, column=0, padx=5, pady=5, sticky=tk.W)
        accuracy = self.gps.data.get('accuracy', 0.0)
        try:
            accuracy_text = f"{float(accuracy):.2f} meters" if accuracy else "Unknown"
        except (ValueError, TypeError):
            accuracy_text = "Unknown"
        ttk.Label(summary_frame, text=accuracy_text).grid(row=2, column=1, padx=5, pady=5, sticky=tk.W)

        # Refresh button
        refresh_btn = ttk.Button(sat_window, text="Refresh Data",
                                 command=lambda: self.update_satellite_window(sat_window, tree))
        refresh_btn.pack(pady=10)

    def update_satellite_window(self, window, tree):
        """Update the satellite information window with fresh data."""
        if not self.gps or not hasattr(self.gps, 'data'):
            return

        # Clear existing data
        for item in tree.get_children():
            tree.delete(item)

        # Populate with updated satellite data
        sat_data = self.gps.data.get('satellite_data', [])
        for i, sat in enumerate(sat_data):
            try:
                prn = sat.get('prn', '--')
                elevation = sat.get('elevation', '--')
                azimuth = sat.get('azimuth', '--')
                snr = sat.get('snr', '--')

                # Safely determine if satellite is used based on SNR
                used = "No"
                if snr and snr != '--':
                    try:
                        if int(snr) > 30:
                            used = "Yes"
                    except (ValueError, TypeError):
                        pass

                tree.insert("", tk.END, values=(prn, elevation, azimuth, snr, used))
            except Exception as e:
                print(f"Error updating satellite in tree: {e}")

        # Update window title with timestamp
        window.title(f"Satellite Information - Updated: {datetime.datetime.now().strftime('%H:%M:%S')}")

    def update_data(self):
        """Update the GUI with the latest GPS data."""
        has_valid_coords = False

        if self.gps and self.gps.running:
            for field, label in self.data_labels.items():
                value = self.gps.data.get(field, "--")
                if field == "lat" or field == "lon":
                    # Format lat/lon to degrees, minutes, seconds
                    if value != 0.0 and value != "--":
                        degrees = int(value)
                        minutes = (value - degrees) * 60
                        label.configure(text=f"{degrees}° {minutes:.4f}'")
                        # Enable map button if we have both coordinates
                        if self.gps.data.get('lat', 0) != 0 and self.gps.data.get('lon', 0) != 0:
                            has_valid_coords = True
                    else:
                        label.configure(text="--")
                elif field in ["speed", "altitude", "hdop", "pdop", "vdop", "accuracy"]:
                    if value != 0.0 and value != "--":
                        # Make sure value is a float before formatting
                        try:
                            # Convert to float if it's not already
                            float_value = float(value) if not isinstance(value, float) else value
                            label.configure(text=f"{float_value:.2f}")
                        except (ValueError, TypeError):
                            # If conversion fails, just show the value as is
                            label.configure(text=str(value))
                    else:
                        label.configure(text="--")
                elif field == "fix_type":
                    # Interpret fix type
                    fix_types = {
                        0: "No Fix",
                        1: "GPS Fix",
                        2: "DGPS Fix",
                        3: "PPS Fix",
                        4: "RTK Fix",
                        5: "Float RTK"
                    }
                    fix_text = fix_types.get(value, str(value))
                    label.configure(text=fix_text)
                elif field == "gnss_mode":
                    # Show active GNSS systems
                    gnss_systems = self.gps.data.get('gnss_systems', [])
                    label.configure(text=", ".join(gnss_systems) if gnss_systems else "GPS")
                else:
                    label.configure(text=str(value))

            # Update button states
            if has_valid_coords:
                self.map_btn.state(['!disabled'])
                self.sat_btn.state(['!disabled'])
                self.apply_btn.state(['!disabled'])
            else:
                self.map_btn.state(['disabled'])
                self.sat_btn.state(['disabled'])

            # Update raw NMEA display
            raw = self.gps.data.get('raw_sentence', '')
            if raw:
                self.nmea_display.delete(1.0, tk.END)
                self.nmea_display.insert(tk.END, raw)

            # Schedule the next update
            self.after_id = self.root.after(self.update_interval, self.update_data)

    def open_maps(self):
        """Open the current GPS coordinates in Google Maps."""
        lat = self.gps.data.get('lat', 0)
        lon = self.gps.data.get('lon', 0)

        if lat != 0 and lon != 0:
            # Format for Google Maps URL
            maps_url = f"https://www.google.com/maps?q={lat},{lon}&z=17"  # z=17 for closer zoom
            webbrowser.open(maps_url)
            self.status_var.set(f"Opening map at {lat}, {lon}")
        else:
            self.status_var.set("No valid coordinates available")

    def on_close(self):
        """Clean up resources when closing the application."""
        if self.gps:
            self.gps.stop()
        self.root.destroy()


def main():
    """Main entry point for the application."""
    # Start the GUI application
    root = tk.Tk()
    app = GPSApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


if __name__ == "__main__":
    main()