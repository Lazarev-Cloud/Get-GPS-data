"""Connection panel for GPS Data Viewer."""

import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import os
import datetime

from config.settings import BAUD_RATES
from utils.serial_utils import get_available_ports

class ConnectionPanel:
    """Panel for connection controls and logging."""
    
    def __init__(self, parent, app):
        """Initialize the connection panel.
        
        Args:
            parent: Parent tkinter container
            app: Main application instance
        """
        self.parent = parent
        self.app = app
        
        # UI Variables
        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value="9600")
        
        # Create UI
        self._create_widgets()
        self.refresh_ports()  # Initial port list
        
    def _create_widgets(self):
        """Create the connection panel widgets."""
        # Connection Frame
        conn_frame = ttk.LabelFrame(self.parent, text="Connection")
        conn_frame.grid(row=0, column=0, columnspan=2, padx=5, pady=5, sticky="ew")
        conn_frame.columnconfigure(6, weight=1)  # Allow buttons to space out

        # Port selector
        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.port_combo = ttk.Combobox(conn_frame, textvariable=self.port_var, width=30, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        # Refresh button
        self.refresh_btn = ttk.Button(conn_frame, text="↺", width=3, 
                                      command=self.refresh_ports, style="Toolbutton.TButton")
        self.refresh_btn.grid(row=0, column=2, padx=(0, 5), pady=5)

        # Baud rate selector
        ttk.Label(conn_frame, text="Baud:").grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        self.baud_combo = ttk.Combobox(conn_frame, textvariable=self.baud_var, 
                                      values=BAUD_RATES, width=8, state="readonly")
        self.baud_combo.grid(row=0, column=4, padx=5, pady=5, sticky=tk.W)

        # Connect button
        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self._toggle_connection)
        self.connect_btn.grid(row=0, column=5, padx=5, pady=5)

        # Log button
        self.log_btn = ttk.Button(conn_frame, text="Start Logging", command=self._toggle_logging)
        self.log_btn.grid(row=0, column=6, padx=5, pady=5)
        
    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        available_ports = get_available_ports()
        port_descriptions = [desc for _, desc in available_ports]
        self.port_combo['values'] = port_descriptions

        current_selection = self.port_var.get()
        current_device = current_selection.split(':')[0].strip() if ':' in current_selection else None

        if not available_ports:
            self.port_var.set("")
            self.app.status_var.set("No serial ports found.")
        elif current_device and current_device in [dev for dev, _ in available_ports]:
            # Keep current selection if still available and valid
            current_desc = next((desc for dev, desc in available_ports if dev == current_device), None)
            if current_desc:
                self.port_var.set(current_desc)
            else:
                # Current device exists but description changed? Select first.
                self.port_var.set(available_ports[0][1])
        elif available_ports:
            # Default to the first port in the sorted list
            self.port_var.set(available_ports[0][1])

        status_msg = f"Found {len(available_ports)} ports." if available_ports else "No serial ports found."
        if len(available_ports) == 1:
            status_msg = f"Found 1 port: {available_ports[0][0]}"
        self.app.status_var.set(status_msg)
        
    def _toggle_connection(self):
        """Connect to or disconnect from the GPS device."""
        if self.app.gps_reader and self.app.gps_reader.running.is_set():
            # --- Disconnect ---
            self._disconnect()
        else:
            # --- Connect ---
            self._connect()
            
    def _connect(self):
        """Connect to the GPS device."""
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

        self.app.status_var.set(f"Connecting to {port}...")
        self.connect_btn.config(state=tk.DISABLED)  # Disable while connecting
        self.app.root.update_idletasks()

        # Create and start GPS reader
        from core.gps_reader import GPSReader
        self.app.gps_reader = GPSReader(port, baud)
        
        # Apply smoothing from settings panel (if exists)
        if hasattr(self.app, 'settings_panel'):
            self.app.settings_panel.apply_settings(initial=True)

        if self.app.gps_reader.start():
            self.app.status_var.set(f"Connected to {port} at {baud} baud")
            self.connect_btn.config(state=tk.NORMAL)  # Re-enable
            self.app.update_ui_states(connected=True)
            self.app.gui_update_after_id = self.app.root.after(
                self.app.update_interval, self.app.update_gui)
        else:
            self.app.status_var.set(f"Failed to connect to {port}")
            messagebox.showerror("Connection Failed", 
                                f"Could not connect to {port}.\nCheck port, permissions, and if device is in use.")
            self.app.gps_reader = None
            self.connect_btn.config(state=tk.NORMAL)  # Re-enable
            self.app.update_ui_states(connected=False)
            
    def _disconnect(self):
        """Disconnect from the GPS device."""
        self.app.status_var.set("Disconnecting...")
        self.connect_btn.config(state=tk.DISABLED)  # Disable while disconnecting
        self.app.root.update_idletasks()
        
        # Cancel update timer
        if self.app.gui_update_after_id:
            self.app.root.after_cancel(self.app.gui_update_after_id)
            self.app.gui_update_after_id = None
            
        # Stop GPS reader
        if self.app.gps_reader:
            self.app.gps_reader.stop()
            self.app.gps_reader = None
            
        # Clear displays
        if hasattr(self.app, 'data_display'):
            self.app.data_display.clear()
        if hasattr(self.app, 'nmea_display'):
            self.app.nmea_display.clear()
            
        # Close satellite window if open
        if hasattr(self.app, 'satellite_window') and self.app.satellite_window:
            self.app.satellite_window.close()
            
        self.app.status_var.set("Disconnected")
        self.connect_btn.config(state=tk.NORMAL)  # Re-enable
        self.app.update_ui_states(connected=False)
            
    def _toggle_logging(self):
        """Start or stop logging GPS data."""
        if not self.app.gps_reader:
            messagebox.showwarning("Logging", "Connect to GPS device first.")
            return

        if self.app.gps_reader.data_logger.log_enabled:
            # Stop logging
            self.app.gps_reader.stop_logging()
            self.app.status_var.set("Logging stopped.")
            self.app.update_ui_states(
                connected=True, 
                logging=False, 
                has_fix=self.app._has_fix())
        else:
            # Start logging with file dialog
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            default_filename = f"gps_log_{timestamp}.csv"
            filename = filedialog.asksaveasfilename(
                title="Save GPS Log As",
                initialfile=default_filename,
                defaultextension=".csv",
                filetypes=[("CSV Files", "*.csv"), ("All Files", "*.*")]
            )
            if filename:
                if self.app.gps_reader.start_logging(filename):
                    self.app.status_var.set(f"Logging to {os.path.basename(filename)}")
                    self.app.update_ui_states(
                        connected=True, 
                        logging=True, 
                        has_fix=self.app._has_fix())
                else:
                    messagebox.showerror("Logging Error", 
                                        "Could not start logging.\nCheck file permissions or path.")
                    self.app.status_var.set("Logging failed to start.")
                    self.app.update_ui_states(
                        connected=True, 
                        logging=False, 
                        has_fix=self.app._has_fix())
            else:
                self.app.status_var.set("Logging cancelled.")
                
    def update_state(self, connected: bool, logging: bool):
        """Update the panel UI state based on connection and logging status.
        
        Args:
            connected: True if connected to GPS device
            logging: True if logging is active
        """
        conn_state = tk.NORMAL if not connected else tk.DISABLED
        log_state = tk.NORMAL if connected else tk.DISABLED

        self.port_combo.config(state=conn_state)
        self.baud_combo.config(state=conn_state)
        self.refresh_btn.config(state=conn_state)
        self.connect_btn.config(text="Disconnect" if connected else "Connect")
        self.log_btn.config(state=log_state)
        self.log_btn.config(text="Stop Logging" if logging else "Start Logging")