"""Satellite information window."""

import tkinter as tk
from tkinter import ttk
import datetime
from typing import Dict, Any, Optional, List

class SatelliteWindow:
    """Window for displaying detailed satellite information."""
    
    def __init__(self, parent, app):
        """Initialize the satellite window.
        
        Args:
            parent: Parent tkinter window
            app: Main application instance
        """
        self.parent = parent
        self.app = app
        self.window: Optional[tk.Toplevel] = None
        self.tree: Optional[ttk.Treeview] = None
        self.summary_gnss: Optional[ttk.Label] = None
        self.summary_sats: Optional[ttk.Label] = None
        self.summary_dop: Optional[ttk.Label] = None
        self.summary_accuracy: Optional[ttk.Label] = None
        
        self._create_window()
        
    def _create_window(self):
        """Create the satellite information window."""
        if self.window:
            return
            
        self.window = tk.Toplevel(self.parent)
        self.window.title("Satellite Information")
        self.window.geometry("650x450")
        self.window.minsize(500, 300)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        
        # Frame for Treeview and Scrollbar
        tree_frame = ttk.Frame(self.window, padding=5)
        tree_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Treeview widget
        columns = ("prn", "gnss", "elev", "azim", "snr", "used")
        self.tree = ttk.Treeview(tree_frame, columns=columns, show="headings")

        # Define headings and column properties
        self.tree.heading("prn", text="PRN", command=lambda: self._sort_tree("prn", False))
        self.tree.column("prn", width=50, anchor=tk.CENTER, stretch=False)
        self.tree.heading("gnss", text="System", command=lambda: self._sort_tree("gnss", False))
        self.tree.column("gnss", width=70, anchor=tk.W, stretch=False)
        self.tree.heading("elev", text="Elevation (°)", command=lambda: self._sort_tree("elev", True))
        self.tree.column("elev", width=80, anchor=tk.CENTER, stretch=False)
        self.tree.heading("azim", text="Azimuth (°)", command=lambda: self._sort_tree("azim", True))
        self.tree.column("azim", width=80, anchor=tk.CENTER, stretch=False)
        self.tree.heading("snr", text="SNR (dB)", command=lambda: self._sort_tree("snr", True))
        self.tree.column("snr", width=60, anchor=tk.CENTER, stretch=False)
        self.tree.heading("used", text="Used in Fix", command=lambda: self._sort_tree("used", False))
        self.tree.column("used", width=80, anchor=tk.CENTER, stretch=False)

        # Scrollbar
        scrollbar = ttk.Scrollbar(tree_frame, orient=tk.VERTICAL, command=self.tree.yview)
        self.tree.configure(yscrollcommand=scrollbar.set)

        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Frame for Summary Info below tree
        summary_frame = ttk.LabelFrame(self.window, text="Summary", padding=5)
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
        
    def _sort_tree(self, col: str, reverse: bool):
        """Sort the satellite treeview by a column.
        
        Args:
            col: Column name to sort by
            reverse: True for reverse sort
        """
        if not self.tree:
            return
            
        # Get data from treeview column
        data = [(self.tree.set(item, col), item) for item in self.tree.get_children('')]

        # Convert to numeric for sorting where appropriate, handle '--' or errors
        def safe_num_convert(x):
            try:
                return int(x)
            except (ValueError, TypeError):
                return -1  # Sort errors/non-numeric low

        if col in ["prn", "elev", "azim", "snr"]:
            data.sort(key=lambda t: safe_num_convert(t[0]), reverse=reverse)
        else:  # Sort alphabetically for gnss, used
            data.sort(key=lambda t: str(t[0]).lower(), reverse=reverse)

        for index, (val, item) in enumerate(data):
            self.tree.move(item, '', index)

        # Toggle sort direction for next click
        self.tree.heading(col, command=lambda: self._sort_tree(col, not reverse))
        
    def update_data(self, data: Dict[str, Any]):
        """Update the window with current GPS data.
        
        Args:
            data: GPS data dictionary
        """
        if not self.is_open():
            return
            
        sat_data_list = data.get('satellite_data', [])  # List of dicts
        sats_used_set = set(data.get('satellites_used', []))  # Set of PRNs

        # --- Update Treeview ---
        # Store current selection and scroll position
        selected_items = self.tree.selection()
        scroll_pos = self.tree.yview()

        # Keep track of items currently in the treeview by PRN
        tree_items = {self.tree.set(item, "prn"): item for item in self.tree.get_children('')}
        updated_prns = set()

        for sat_info in sat_data_list:
            prn = sat_info.get('prn')
            if prn is None:
                continue  # Skip if PRN is missing

            prn_str = str(prn)
            updated_prns.add(prn_str)  # Mark this PRN as present in the data

            elev = sat_info.get('elevation', '--')
            azim = sat_info.get('azimuth', '--')
            snr = sat_info.get('snr', '--')
            used = prn in sats_used_set
            used_text = "Yes" if used else "No"
            tag = 'Used' if used else 'NotUsed'

            # Determine GNSS System
            gnss = "Unknown"
            if isinstance(prn, int):
                if 1 <= prn <= 32:
                    gnss = "GPS"
                elif 33 <= prn <= 64:
                    gnss = "SBAS"
                elif 65 <= prn <= 96:
                    gnss = "GLONASS"
                elif 120 <= prn <= 140:
                    gnss = "SBAS"  # More SBAS
                elif 193 <= prn <= 197:
                    gnss = "QZSS"
                elif 201 <= prn <= 235:
                    gnss = "Galileo"
                elif 301 <= prn <= 336:
                    gnss = "BeiDou"

            values = (prn_str, gnss, str(elev), str(azim), str(snr), used_text)

            if prn_str in tree_items:
                # --- Update existing item ---
                item_id = tree_items[prn_str]
                try:
                    # Update values if they changed
                    current_values = self.tree.item(item_id, 'values')
                    if current_values != values:
                        self.tree.item(item_id, values=values, tags=(tag,))
                    # Update tag if it changed
                    current_tags = self.tree.item(item_id, 'tags')
                    if tag not in current_tags:
                        self.tree.item(item_id, tags=(tag,))  # Overwrite tags
                except tk.TclError:
                    pass  # Item might have been deleted concurrently?
            else:
                # --- Insert new item ---
                try:
                    self.tree.insert("", tk.END, values=values, tags=(tag,), iid=prn_str)
                except tk.TclError as insert_err:
                    print(f"WARN: Error inserting sat {prn_str} into tree: {insert_err}")

        # --- Remove items from tree that are no longer in sat_data_list ---
        prns_in_tree = set(tree_items.keys())
        prns_to_remove = prns_in_tree - updated_prns
        for prn_to_remove in prns_to_remove:
            try:
                item_id = tree_items[prn_to_remove]
                if self.tree.exists(item_id):
                    self.tree.delete(item_id)
            except tk.TclError:
                pass  # Item might already be gone

        # --- Restore selection and scroll position ---
        try:
            if selected_items:
                # Filter items that still exist
                valid_selection = [item for item in selected_items if self.tree.exists(item)]
                if valid_selection:
                    self.tree.selection_set(valid_selection)
            self.tree.yview_moveto(scroll_pos[0])
        except tk.TclError:
            pass  # Handle errors if items/view changed drastically

        # --- Update Summary Labels ---
        # GNSS systems
        gnss_systems = data.get('gnss_systems_used', set())
        self.summary_gnss.config(text=", ".join(sorted(list(gnss_systems))) if gnss_systems else "None")

        # Satellites
        sats_view = data.get('satellites_in_view', 0)
        sats_used_count = len(sats_used_set)
        self.summary_sats.config(text=f"{sats_view} / {sats_used_count}")

        # DOP values
        hdop = data.get('hdop', 0.0)
        pdop = data.get('pdop', 0.0)
        vdop = data.get('vdop', 0.0)
        self.summary_dop.config(text=f"{hdop:.2f} / {pdop:.2f} / {vdop:.2f}" if hdop > 0 else "-- / -- / --")

        # Accuracy estimate
        accuracy = data.get('accuracy_estimate', 0.0)
        self.summary_accuracy.config(text=f"~ {accuracy:.2f} m" if accuracy > 0 else "--")

        # Update window title with timestamp
        now = datetime.datetime.now().strftime('%H:%M:%S')
        self.window.title(f"Satellite Information (Updated: {now})")
        
    def update_state(self, connected: bool):
        """Update the window state based on connection status.
        
        Args:
            connected: True if connected to GPS device
        """
        # If not connected, close the window
        if not connected and self.is_open():
            self.close()
        
    def is_open(self) -> bool:
        """Check if the window is open.
        
        Returns:
            True if the window is open
        """
        return self.window is not None and self.window.winfo_exists()
        
    def show(self):
        """Show or bring the window to front."""
        if self.is_open():
            self.window.lift()
            self.window.focus_force()  # Try to grab focus
        else:
            self._create_window()
            
    def close(self):
        """Close the window."""
        if self.is_open():
            print("Closing satellite window.")
            self.window.destroy()
            
        self.window = None
        self.tree = None
        self.summary_gnss = None
        self.summary_sats = None
        self.summary_dop = None
        self.summary_accuracy = None