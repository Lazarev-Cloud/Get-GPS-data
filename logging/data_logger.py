"""CSV data logging module."""

import csv
import datetime
from typing import Optional, Dict, Any

from config.settings import CSV_LOG_HEADERS, FIX_QUALITY_MAP, FIX_TYPE_MAP

class DataLogger:
    """Handles logging of GPS data to CSV files."""
    
    def __init__(self):
        """Initialize the data logger."""
        self.log_file: Optional[object] = None
        self.csv_writer: Optional[csv.writer] = None
        self.log_enabled = False
        self.log_filename: Optional[str] = None
        
    def start_logging(self, filename: str) -> bool:
        """Start logging GPS data to a CSV file.
        
        Args:
            filename: Path to the log file
            
        Returns:
            True if logging started successfully, False otherwise
        """
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
                
    def log_entry(self, data: Dict[str, Any]):
        """Log a GPS data entry to the CSV file.
        
        Args:
            data: GPS data dictionary
        """
        if not self.log_enabled or not self.csv_writer:
            return
            
        try:
            log_row = [
                data['host_timestamp'].strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] if data['host_timestamp'] else '',
                data['fix_timestamp'].strftime("%H:%M:%S.%f")[:-3] if data['fix_timestamp'] else '',
                f"{data['lat']:.7f}" if data['lat'] else '0.0',
                f"{data['lon']:.7f}" if data['lon'] else '0.0',
                f"{data['altitude']:.2f}" if data['altitude'] is not None else '0.0',
                f"{data['speed_kph']:.2f}" if data['speed_kph'] is not None else '0.0',
                data['satellites_in_view'],
                len(data.get('satellites_used', [])),
                FIX_QUALITY_MAP.get(data['fix_quality'], data['fix_quality']),
                f"{data['hdop']:.2f}" if data['hdop'] else '0.0',
                f"{data['pdop']:.2f}" if data['pdop'] else '0.0',
                f"{data['vdop']:.2f}" if data['vdop'] else '0.0',
                FIX_TYPE_MAP.get(data['fix_type'], data['fix_type']),
                f"{data['accuracy_estimate']:.2f}" if data['accuracy_estimate'] else '0.0',
                f"{data['geoid_height']:.2f}" if data['geoid_height'] is not None else '0.0',
                ",".join(sorted(list(data['gnss_systems_used']))) if data['gnss_systems_used'] else 'None',
                data['raw_sentence']  # Log the raw NMEA sentence
            ]
            self.csv_writer.writerow(log_row)
        except Exception as log_err:
            print(f"ERROR writing to log file: {log_err}")