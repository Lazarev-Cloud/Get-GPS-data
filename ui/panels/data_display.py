"""Data display panel for GPS Data Viewer."""

import tkinter as tk
from tkinter import ttk, messagebox
import webbrowser
from typing import Dict, Any

from utils.geo_utils import degrees_to_dms
from config.settings import FIX_QUALITY_MAP, FIX_TYPE_MAP

class DataDisplayPanel:
    """Panel for displaying GPS data and actions."""
    
    def __init__(self, parent, app):
        """Initialize the data display panel.
        
        Args:
            parent: Parent tkinter container
            app: Main application instance
        """
        self.parent = parent
        self.app = app
        
        # Create UI
        self._create_widgets()
        
    def _create_widgets(self):
        """Create the data display panel widgets."""
        # Data Frame
        data_frame = ttk.LabelFrame(self.parent, text="GPS Data")
        data_frame.grid(row=2, column=0, padx=5, pady=5, sticky="nsew")
        data_frame.columnconfigure(1, weight=1)  # Let data values expand
        self.parent.rowconfigure(2, weight=1)  # Let data frame expand vertically

        # GPS data labels
        self.data_labels: Dict[str, ttk.Label] = {}
        # Match field names to keys in GPSReader.current_data
        data_fields = [
            ("lat", "Latitude:", 0), ("lon", "Longitude:", 1),
            ("altitude", "Altitude:", 2), ("geoid_height", "Geoid Height:", 3),
            ("speed_kph", "Speed:", 4),
            ("fix_quality", "Fix Quality:", 5), ("fix_type", "Fix Type:", 6),
            ("satellites_view_used", "Sats View/Used:", 7),  # Combined label placeholder
            ("accuracy_estimate", "Est. Accuracy:", 8),
            ("hdop", "HDOP:", 9), ("pdop", "PDOP:", 10), ("vdop", "VDOP:", 11),
            ("gnss_systems_used", "GNSS Systems:", 12),
            ("fix_timestamp", "Device Time:", 13), ("host_timestamp", "Host Time:", 14),
        ]

        row_offset = 0
        for field, label_text, row in data_fields:
            ttk.Label(data_frame, text=label_text).grid(row=row_offset + row, column=0, padx=5, pady=2, sticky=tk.W)
            # Use a slightly sunken relief to make fields clearer
            self.data_labels[field] = ttk.Label(data_frame, text="--", anchor=tk.W, 
                                               relief=tk.SUNKEN, padding=2, width=25)  # Min width
            self.data_labels[field].grid(row=row_offset + row, column=1, padx=5, pady=2, sticky="ew")

        # --- Action Buttons Frame ---
        action_frame = ttk.Frame(self.parent, padding=5)
        action_frame.grid(row=2, column=1, padx=5, pady=5, sticky="ne")  # Align to top-right of data area

        self.map_btn = ttk.Button(action_frame, text="View on Map", command=self._open_maps)
        self.map_btn.pack(pady=3, fill=tk.X)

        self.sat_btn = ttk.Button(action_frame, text="Satellite Info", command=self._show_satellite_window)
        self.sat_btn.pack(pady=3, fill=tk.X)
        
    def clear(self):
        """Clear all data fields."""
        for field in self.data_labels:
            if field == "satellites_view_used":
                self.data_labels[field].config(text="-- / --")
            else:
                self.data_labels[field].config(text="--")
        
    def update_data(self, data: Dict[str, Any]):
        """Update the display with current GPS data.
        
        Args:
            data: GPS data dictionary
        """
        for field, label_widget in self.data_labels.items():
            value = data.get(field)
            text = "--"  # Default text

            try:
                # Handle each field specifically for proper formatting
                if field == "lat":
                    text = degrees_to_dms(value)
                elif field == "lon":
                    text = degrees_to_dms(value)
                elif field in ["altitude", "geoid_height"]:
                    text = f"{float(value):.2f} m" if value is not None else "--"
                elif field == "speed_kph":
                    text = f"{float(value):.2f} km/h" if value is not None else "--"
                elif field == "fix_quality":
                    text = FIX_QUALITY_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "fix_type":
                    text = FIX_TYPE_MAP.get(value, str(value)) if value is not None else "--"
                elif field == "satellites_view_used":  # Handle combined field
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
                    import datetime
                    text = value.strftime("%H:%M:%S.%f")[:-3] if isinstance(value, (datetime.datetime, datetime.time)) else "--"
                elif field == "host_timestamp":
                    import datetime
                    text = value.strftime("%Y-%m-%d %H:%M:%S") if isinstance(value, datetime.datetime) else "--"

                label_widget.config(text=text)
            except (ValueError, TypeError, AttributeError):
                label_widget.config(text="Error")  # Indicate formatting issue in GUI
                
    def update_state(self, connected: bool, has_fix: bool):
        """Update the UI state based on connection and fix status.
        
        Args:
            connected: True if connected to GPS device
            has_fix: True if GPS has a valid fix
        """
        sat_state = tk.NORMAL if connected else tk.DISABLED
        map_fix_state = tk.NORMAL if connected and has_fix else tk.DISABLED
        
        self.map_btn.config(state=map_fix_state)
        self.sat_btn.config(state=sat_state)
        
    def _open_maps(self):
        """Open the current coordinates in the default web browser using Google Maps."""
        if not self.app.gps_reader:
            return
            
        data = self.app.gps_reader.get_current_data()
        lat = data.get('lat', 0.0)  # Use the processed/smoothed lat/lon
        lon = data.get('lon', 0.0)

        if lat != 0.0 and lon != 0.0:
            maps_url = f"https://www.google.com/maps?q={lat:.7f},{lon:.7f}&z=16"  # More precision in URL
            try:
                webbrowser.open(maps_url, new=2)  # Try to open in a new tab
                self.app.status_var.set(f"Opening map for {lat:.5f}, {lon:.5f}")
            except Exception as e:
                messagebox.showerror("Map Error", f"Could not open web browser: {e}")
                self.app.status_var.set("Failed to open map.")
        else:
            self.app.status_var.set("No valid coordinates to map.")
            messagebox.showwarning("Map Error", "No valid GPS coordinates available to display on map.")
            
    def _show_satellite_window(self):
        """Show the satellite information window."""
        self.app.show_satellite_window()