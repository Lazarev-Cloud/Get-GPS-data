"""Main application window for GPS Data Viewer."""

import tkinter as tk
from tkinter import ttk
import platform
from typing import Optional, Dict, Any

from config.settings import UPDATE_INTERVAL_MS
from ui.panels.connection_panel import ConnectionPanel
from ui.panels.data_display import DataDisplayPanel
from ui.panels.nmea_display import NMEADisplayPanel
from ui.panels.settings_panel import SettingsPanel
from ui.windows.satellite_view import SatelliteWindow

class GPSApp:
    """Main application class for GPS Data Viewer."""

    def __init__(self, root: tk.Tk):
        """Initialize the application.
        
        Args:
            root: Tkinter root window
        """
        self.root = root
        self.root.title("GPS Data Viewer")
        self.root.minsize(750, 600)

        # GPS reader reference
        self.gps_reader = None
        self.gui_update_after_id = None
        
        # Update interval
        self.update_interval = UPDATE_INTERVAL_MS

        # Create styles
        self._create_styles()
        
        # Create main UI structure
        self._create_main_layout()
        
        # Status bar
        self.status_var = tk.StringVar(value="Not connected")
        self.status_bar = ttk.Label(self.root, textvariable=self.status_var, 
                                  relief=tk.SUNKEN, anchor=tk.W, padding=2)
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)
        
        # Initialize UI components
        self.connection_panel = ConnectionPanel(self.top_frame, self)
        self.settings_panel = SettingsPanel(self.top_frame, self)
        self.data_display = DataDisplayPanel(self.top_frame, self)
        self.nmea_display = NMEADisplayPanel(self.bottom_frame, self)
        
        # Satellite window (created on demand)
        self.satellite_window = None
        
        # Initialize UI state
        self.update_ui_states()

    def _create_styles(self):
        """Configure ttk styles."""
        style = ttk.Style()
        try:
            # Try themed styles first
            if platform.system() == "Windows":
                style.theme_use('vista')
            elif platform.system() == "Darwin":
                style.theme_use('aqua')
            else:
                style.theme_use('clam')
        except tk.TclError:
            style.theme_use('default')  # Fallback

        style.configure("TLabel", padding=2)
        style.configure("TButton", padding=5)
        style.configure("Toolbutton.TButton", padding=2)  # Smaller padding for refresh
        style.configure("TCombobox", padding=2)
        style.configure("TLabelframe.Label", padding=(5, 2))
        style.configure("Treeview.Heading", font=('Helvetica', 10, 'bold'))
        # Add tag for satellites used in fix
        style.configure("Used.Treeview", foreground="green")  # Used in fix
        style.configure("NotUsed.Treeview", foreground="gray")  # Not used

    def _create_main_layout(self):
        """Create the main application layout."""
        # Main panes
        main_pane = tk.PanedWindow(self.root, orient=tk.VERTICAL, sashrelief=tk.RAISED)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Top and bottom frames
        self.top_frame = ttk.Frame(main_pane, padding=5)
        self.bottom_frame = ttk.Frame(main_pane, padding=5)
        
        main_pane.add(self.top_frame, stretch="always")
        main_pane.add(self.bottom_frame, stretch="never")
        
        # Configure top frame grid
        self.top_frame.columnconfigure(1, weight=1)  # Allow data labels to expand

    def update_ui_states(self, connected: bool = False, logging: bool = False, has_fix: bool = False):
        """Update UI states based on connection and data status.
        
        Args:
            connected: True if connected to GPS device
            logging: True if logging is active
            has_fix: True if GPS has a valid fix
        """
        # Update connection panel
        self.connection_panel.update_state(connected, logging)
        
        # Update settings panel
        if hasattr(self, 'settings_panel'):
            # Update settings panel UI based on selected method
            if connected and self.gps_reader:
                self.settings_panel._update_ui_state(self.gps_reader.smoothing_method)
        
        # Update data display
        self.data_display.update_state(connected, has_fix)
        
        # Update NMEA display
        self.nmea_display.update_state(connected)
        
        # Update satellite window if open
        if self.satellite_window and self.satellite_window.is_open():
            self.satellite_window.update_state(connected)
            
        # Update status bar color based on fix
        if connected and has_fix:
            self.status_bar.config(foreground="black")
        elif connected and not has_fix:
            self.status_bar.config(foreground="red")
        else:
            self.status_bar.config(foreground="black")
def update_gui(self):
        """Periodically update the GUI with latest GPS data."""
        if not self.gps_reader or not self.gps_reader.running.is_set():
            if self.gui_update_after_id:
                self.root.after_cancel(self.gui_update_after_id)
                self.gui_update_after_id = None
            return

        # Get current data
        data = self.gps_reader.get_current_data()
        has_fix = self._has_fix(data)

        # Update UI components
        self.data_display.update_data(data)
        self.nmea_display.update_data(data.get('raw_sentence', ''))
        
        # Update satellite window if open
        if self.satellite_window and self.satellite_window.is_open():
            self.satellite_window.update_data(data)

        # Update status bar
        self._update_status_bar(data, has_fix)
        
        # Update UI states
        self.update_ui_states(
            connected=True, 
            logging=self.gps_reader.data_logger.log_enabled, 
            has_fix=has_fix
        )

        # Schedule next update
        self.gui_update_after_id = self.root.after(self.update_interval, self.update_gui)

    def _has_fix(self, data: Dict[str, Any] = None) -> bool:
        """Check if the current GPS data indicates a valid fix.
        
        Args:
            data: GPS data dictionary (optional, will get from reader if None)
            
        Returns:
            True if GPS has a valid fix
        """
        if not self.gps_reader:
            return False
            
        # If no data provided, get it from reader
        if data is None:
            data = self.gps_reader.get_current_data()
            
        # Consider quality > 0 and non-zero lat/lon as a basic fix indicator
        # Also check fix_type (2=2D, 3=3D are minimal fixes)
        quality = data.get('fix_quality', 0)
        fix_type = data.get('fix_type', 0)
        lat = data.get('lat', 0.0)
        lon = data.get('lon', 0.0)
        
        # RMC status 'V' should have already set quality to 0
        return quality > 0 and fix_type >= 2 and lat != 0.0 and lon != 0.0

    def _update_status_bar(self, data: Dict[str, Any], has_fix: bool):
        """Update the status bar with current GPS status.
        
        Args:
            data: GPS data dictionary
            has_fix: True if GPS has a valid fix
        """
        if has_fix:
            from config.settings import FIX_QUALITY_MAP
            # Use default color for fix
            self.status_bar.config(foreground="black")
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
            if fix_type == 1:
                status_text = "Status: No Fix (Searching...)"
            elif data.get('fix_quality', 0) > 0 and fix_type < 2:
                status_text = "Status: Acquiring 2D/3D Fix..."  # e.g., Quality=1, Type=1
            self.status_var.set(status_text)

    def show_satellite_window(self):
        """Show the satellite information window."""
        if not self.gps_reader:
            from tkinter import messagebox
            messagebox.showinfo("Info", "Connect to GPS device first.")
            return
            
        if not self.satellite_window:
            self.satellite_window = SatelliteWindow(self.root, self)
        else:
            self.satellite_window.show()

    def on_close(self):
        """Perform cleanup actions before closing the application."""
        print("Closing application...")
        if self.gui_update_after_id:
            try: 
                self.root.after_cancel(self.gui_update_after_id)
            except:
                pass
                
        if self.gps_reader:
            self.gps_reader.stop()
            
        if self.satellite_window:
            try:
                self.satellite_window.close()
            except:
                pass
                
        self.root.quit()
        self.root.after(50, self.root.destroy)
        print("Application closed.")