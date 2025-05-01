"""NMEA sentence parser module."""

import datetime
import pynmea2
from typing import Any, Dict, Set, Optional, List, Union

class NMEAParser:
    """Handles parsing of NMEA sentences and updating GPS data."""
    
    def __init__(self, gps_reader):
        """Initialize parser with reference to GPS reader.
        
        Args:
            gps_reader: Reference to the GPSReader instance
        """
        self.gps_reader = gps_reader
        
    def parse_sentence(self, line: str) -> bool:
        """Parse a NMEA sentence and update the GPS reader's data.
        
        Args:
            line: NMEA sentence string
            
        Returns:
            True if parsing successful, False otherwise
        """
        if not line.startswith('$') or len(line) < 6:
            return False

        try:
            msg = pynmea2.parse(line)
            
            # Update data in the GPS reader based on the NMEA message type
            with self.gps_reader.lock:
                # Common updates for all messages
                self.gps_reader.current_data['raw_sentence'] = line
                self.gps_reader.current_data['host_timestamp'] = datetime.datetime.now()
                
                # Update timestamp if available
                self._update_timestamp(msg)
                
                # Process different NMEA message types
                if isinstance(msg, pynmea2.types.talker.GGA):
                    self._process_gga(msg)
                elif isinstance(msg, pynmea2.types.talker.RMC):
                    self._process_rmc(msg)
                elif isinstance(msg, pynmea2.types.talker.GSA):
                    self._process_gsa(msg)
                elif isinstance(msg, pynmea2.types.talker.GSV):
                    self._process_gsv(msg)
                elif isinstance(msg, pynmea2.types.talker.VTG):
                    self._process_vtg(msg)
                
                # Apply smoothing and estimate accuracy
                self._post_process_data()
                
                # Log data if enabled
                if self.gps_reader.data_logger.log_enabled:
                    self.gps_reader.data_logger.log_entry(self.gps_reader.current_data)
                    
            return True
            
        except pynmea2.ParseError:
            # Silently ignore parse errors - common with partial/corrupt sentences
            return False
        except ValueError as e:
            # Value conversion errors
            print(f"WARN: Value error processing NMEA data: {line} - {e}")
            return False
        except AttributeError as e:
            # Missing attribute errors
            print(f"WARN: Attribute error processing NMEA data: {line} - {e}")
            return False
        except Exception as e:
            print(f"ERROR: Unexpected error processing NMEA data: {line} - {e}")
            return False
            
    def _update_timestamp(self, msg):
        """Update the fix timestamp from NMEA message.
        
        Args:
            msg: pynmea2 message object
        """
        if hasattr(msg, 'timestamp') and msg.timestamp:
            try:
                # Combine current date with message time if date isn't included
                current_date = datetime.date.today()
                self.gps_reader.current_data['fix_timestamp'] = datetime.datetime.combine(
                    current_date, msg.timestamp)
            except TypeError:
                # Handle cases where msg.timestamp might not be just time
                if isinstance(msg.timestamp, datetime.time):
                    # Fallback if combine fails but it's a time object
                    try:
                        self.gps_reader.current_data['fix_timestamp'] = datetime.datetime.now().replace(
                            hour=msg.timestamp.hour,
                            minute=msg.timestamp.minute,
                            second=msg.timestamp.second,
                            microsecond=msg.timestamp.microsecond)
                    except Exception:
                        pass  # Ignore if time replacement fails
                elif isinstance(msg.timestamp, datetime.datetime):
                    self.gps_reader.current_data['fix_timestamp'] = msg.timestamp
        
    def _process_gga(self, msg):
        """Process GGA message (Global Positioning System Fix Data).
        
        Args:
            msg: pynmea2 GGA message object
        """
        # Position
        if msg.latitude:
            self.gps_reader.current_data['raw_lat'] = msg.latitude
        if msg.longitude:
            self.gps_reader.current_data['raw_lon'] = msg.longitude
        if msg.altitude is not None:  # Allow 0 altitude
            self.gps_reader.current_data['raw_alt'] = msg.altitude
        
        # Fix quality
        if msg.gps_qual is not None:
            self.gps_reader.current_data['fix_quality'] = int(msg.gps_qual)
        
        # Satellites in use
        if msg.num_sats:
            try:
                # num_sats in GGA usually means satellites *used* for fix
                num_sats_int = int(msg.num_sats)
                # Update satellites_used list length notionally if GSA hasn't provided details yet
                if not self.gps_reader.current_data['satellites_used'] or len(self.gps_reader.current_data['satellites_used']) != num_sats_int:
                    # Placeholder if GSA is missing/late
                    self.gps_reader.current_data['satellites_used'] = list(range(1, num_sats_int + 1))  # Not real PRNs!
            except (ValueError, TypeError):
                pass
        
        # HDOP and geoid separation
        if msg.horizontal_dil:
            self.gps_reader.current_data['hdop'] = float(msg.horizontal_dil)
        if msg.geo_sep is not None:
            self.gps_reader.current_data['geoid_height'] = float(msg.geo_sep)
        
    def _process_rmc(self, msg):
        """Process RMC message (Recommended Minimum Navigation Information).
        
        Args:
            msg: pynmea2 RMC message object
        """
        # Position (if available)
        if msg.latitude:
            self.gps_reader.current_data['raw_lat'] = msg.latitude
        if msg.longitude:
            self.gps_reader.current_data['raw_lon'] = msg.longitude
        
        # Speed
        if msg.spd_over_grnd is not None:
            from utils.geo_utils import knots_to_kph
            self.gps_reader.current_data['speed_kph'] = knots_to_kph(msg.spd_over_grnd)
        
        # RMC status ('A' = Active/valid, 'V' = Void/invalid) is crucial
        if msg.status == 'V':
            self.gps_reader.current_data['fix_quality'] = 0  # Force to invalid
            self.gps_reader.current_data['fix_type'] = 0  # No fix
            self.gps_reader.current_data['satellites_used'] = []  # Clear used sats on void status
            # Clear position history for filters
            if self.gps_reader.moving_avg_filter:
                self.gps_reader.moving_avg_filter.reset()
        elif msg.status == 'A' and self.gps_reader.current_data['fix_quality'] == 0:
            self.gps_reader.current_data['fix_quality'] = 1  # Assume basic GPS fix if RMC is Active and no other quality info yet
        
    def _process_gsa(self, msg):
        """Process GSA message (GPS DOP and active satellites).
        
        Args:
            msg: pynmea2 GSA message object
        """
        # Fix type and DOPs
        if msg.mode_fix_type:
            self.gps_reader.current_data['fix_type'] = int(msg.mode_fix_type)
        if msg.pdop:
            self.gps_reader.current_data['pdop'] = float(msg.pdop)
        if msg.hdop:
            self.gps_reader.current_data['hdop'] = float(msg.hdop)  # Prefer GSA's HDOP
        if msg.vdop:
            self.gps_reader.current_data['vdop'] = float(msg.vdop)

        # Satellites used
        # msg.sv_ids is a tuple containing the PRNs as strings
        valid_sv_ids: List[int] = []
        if hasattr(msg, 'sv_ids'):
            valid_sv_ids = [int(svid) for svid in msg.sv_ids if svid]  # Convert valid IDs to int
        self.gps_reader.current_data['satellites_used'] = valid_sv_ids

        # Infer GNSS systems from the PRNs used
        current_systems: Set[str] = set()
        for prn in valid_sv_ids:
            if 1 <= prn <= 32:
                current_systems.add("GPS")
            elif 33 <= prn <= 64:
                current_systems.add("SBAS")  # WAAS/EGNOS etc.
            elif 65 <= prn <= 96:
                current_systems.add("GLONASS")
            # Add ranges for Galileo, BeiDou, QZSS etc.
            elif 201 <= prn <= 235:
                current_systems.add("Galileo")
            elif 301 <= prn <= 336:
                current_systems.add("BeiDou")
        self.gps_reader.current_data['gnss_systems_used'] = current_systems
        
    def _process_gsv(self, msg):
        """Process GSV message (GPS Satellites in view).
        
        Args:
            msg: pynmea2 GSV message object
        """
        try:
            num_sv_in_view = int(msg.num_sv_in_view) if msg.num_sv_in_view else 0
            # Update total count only if it increases (GSV messages might interleave)
            self.gps_reader.current_data['satellites_in_view'] = max(
                num_sv_in_view,
                self.gps_reader.current_data['satellites_in_view']
            )

            msg_num = int(msg.msg_num) if msg.msg_num else 1
            # Clear previous satellite details *only* when receiving the first message of a sequence
            if msg_num == 1:
                self.gps_reader.current_data['satellite_data'] = []

            sats_in_this_msg = []
            for i in range(1, 5):  # GSV has fields for up to 4 satellites
                prn_str = getattr(msg, f'sv_prn_num_{i}', None)
                if prn_str:  # Check if the PRN field exists and is not empty
                    try:
                        sats_in_this_msg.append({
                            'prn': int(prn_str),
                            'elevation': int(getattr(msg, f'elevation_{i}', 0) or 0),
                            'azimuth': int(getattr(msg, f'azimuth_{i}', 0) or 0),
                            'snr': int(getattr(msg, f'snr_{i}', 0) or 0)  # Treat empty SNR as 0
                        })
                    except (ValueError, TypeError) as sat_parse_err:
                        print(f"WARN: Could not parse satellite detail in GSV: {sat_parse_err} - PRN:'{prn_str}'")

            # Append new sats, then de-duplicate based on PRN, keeping the latest entry
            existing_prns = {sat['prn']: sat for sat in self.gps_reader.current_data['satellite_data']}
            for sat in sats_in_this_msg:
                existing_prns[sat['prn']] = sat  # Add or overwrite with latest info
            self.gps_reader.current_data['satellite_data'] = list(existing_prns.values())

        except (ValueError, TypeError, AttributeError) as gsv_err:
            print(f"WARN: Error processing GSV message: {gsv_err}")
        
    def _process_vtg(self, msg):
        """Process VTG message (Track Made Good and Ground Speed).
        
        Args:
            msg: pynmea2 VTG message object
        """
        # Speed over ground
        if msg.spd_over_grnd_kmph is not None:
            self.gps_reader.current_data['speed_kph'] = float(msg.spd_over_grnd_kmph)
        
    def _post_process_data(self):
        """Apply smoothing and calculate derived fields."""
        # Get the raw position
        proc_lat = self.gps_reader.current_data['raw_lat']
        proc_lon = self.gps_reader.current_data['raw_lon']
        proc_alt = self.gps_reader.current_data['raw_alt']
        
        # Apply the selected smoothing method
        if self.gps_reader.smoothing_method == "Moving Average":
            # Add to history and get averaged position
            self.gps_reader.moving_avg_filter.add_position(proc_lat, proc_lon, proc_alt)
            proc_lat, proc_lon, proc_alt = self.gps_reader.moving_avg_filter.get_filtered_position()
        elif self.gps_reader.smoothing_method == "Kalman Filter" and self.gps_reader.kalman_filter:
            # Apply Kalman filter
            self.gps_reader.kalman_filter.add_position(proc_lat, proc_lon, proc_alt)
            proc_lat, proc_lon, proc_alt = self.gps_reader.kalman_filter.get_filtered_position()
        
        # Update smoothed position
        self.gps_reader.current_data['lat'] = proc_lat
        self.gps_reader.current_data['lon'] = proc_lon
        self.gps_reader.current_data['altitude'] = proc_alt
        
        # Estimate accuracy
        self._estimate_accuracy()
        
    def _estimate_accuracy(self):
        """Estimate position accuracy based on HDOP and satellites."""
        hdop = self.gps_reader.current_data['hdop']
        sats_used_count = len(self.gps_reader.current_data.get('satellites_used', []))
        
        if hdop > 0 and sats_used_count >= 4:
            base_precision = 2.5  # meters (typical good conditions)
            accuracy = hdop * base_precision
            if sats_used_count < 5:
                accuracy *= 1.5
            elif sats_used_count > 8:
                accuracy *= 0.8
            self.gps_reader.current_data['accuracy_estimate'] = round(accuracy, 2)
        elif self.gps_reader.current_data['fix_quality'] == 0:
            self.gps_reader.current_data['accuracy_estimate'] = 0.0  # No accuracy if no fix