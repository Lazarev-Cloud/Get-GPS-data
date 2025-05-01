"""NMEA sentence display panel."""

import tkinter as tk
from tkinter import ttk

class NMEADisplayPanel:
    """Panel for displaying raw NMEA sentences."""
    
    def __init__(self, parent, app):
        """Initialize the NMEA display panel.
        
        Args:
            parent: Parent tkinter container
            app: Main application instance
        """
        self.parent = parent
        self.app = app
        self._last_nmea_displayed = ""
        
        # Create UI
        self._create_widgets()
        
    def _create_widgets(self):
        """Create the NMEA display panel widgets."""
        # Frame
        nmea_frame = ttk.LabelFrame(self.parent, text="Raw NMEA Sentences")
        nmea_frame.pack(fill=tk.BOTH, expand=True)

        # Scrollbar and text widget
        nmea_scrollbar = ttk.Scrollbar(nmea_frame, orient=tk.VERTICAL)
        self.nmea_text = tk.Text(nmea_frame, height=6, width=80, wrap=tk.NONE,
                               yscrollcommand=nmea_scrollbar.set, state=tk.DISABLED,
                               font=("Courier", 9), borderwidth=1, relief=tk.SOLID)
        nmea_scrollbar.config(command=self.nmea_text.yview)

        nmea_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.nmea_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
    def clear(self):
        """Clear the NMEA display."""
        self.nmea_text.config(state=tk.NORMAL)
        self.nmea_text.delete(1.0, tk.END)
        self.nmea_text.config(state=tk.DISABLED)
        self._last_nmea_displayed = ""
        
    def update_data(self, sentence: str):
        """Update the display with a new NMEA sentence.
        
        Args:
            sentence: Raw NMEA sentence string
        """
        if sentence and sentence != self._last_nmea_displayed:
            self.nmea_text.config(state=tk.NORMAL)
            self.nmea_text.insert(tk.END, sentence + '\n')
            self.nmea_text.see(tk.END)  # Auto-scroll
            
            # Limit buffer size
            max_lines = 200
            num_lines = int(self.nmea_text.index('end-1c').split('.')[0])
            if num_lines > max_lines:
                self.nmea_text.delete('1.0', f'{num_lines - max_lines}.0')
                
            self.nmea_text.config(state=tk.DISABLED)
            self._last_nmea_displayed = sentence
            
    def update_state(self, connected: bool):
        """Update the UI state based on connection status.
        
        Args:
            connected: True if connected to GPS device
        """
        # No state changes needed for this panel
        pass