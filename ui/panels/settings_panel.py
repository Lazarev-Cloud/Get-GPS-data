"""Settings panel for GPS Data Viewer."""

import tkinter as tk
from tkinter import ttk, messagebox

from config.settings import SMOOTHING_METHODS, MAX_POSITION_HISTORY

class SettingsPanel:
    """Panel for GPS data processing settings."""
    
    def __init__(self, parent, app):
        """Initialize the settings panel.
        
        Args:
            parent: Parent tkinter container
            app: Main application instance
        """
        self.parent = parent
        self.app = app
        
        # UI Variables
        self.smoothing_var = tk.StringVar(value=SMOOTHING_METHODS[0])
        self.history_var = tk.StringVar(value=str(MAX_POSITION_HISTORY))
        
        # Create UI
        self._create_widgets()
        
    def _create_widgets(self):
        """Create the settings panel widgets."""
        # Frame
        settings_frame = ttk.LabelFrame(self.parent, text="Processing")
        settings_frame.grid(row=1, column=0, columnspan=2, padx=5, pady=5, sticky="ew")

        # Smoothing method
        ttk.Label(settings_frame, text="Smoothing:").grid(row=0, column=0, padx=5, pady=5, sticky=tk.W)
        self.smoothing_combo = ttk.Combobox(settings_frame, textvariable=self.smoothing_var, 
                                          values=SMOOTHING_METHODS, width=15, state="readonly")
        self.smoothing_combo.grid(row=0, column=1, padx=5, pady=5, sticky=tk.W)
        self.smoothing_combo.bind("<<ComboboxSelected>>", self.apply_settings)

        # History size
        self.history_label = ttk.Label(settings_frame, text="Avg History:")
        self.history_label.grid(row=0, column=2, padx=5, pady=5, sticky=tk.W)
        self.history_spin = ttk.Spinbox(settings_frame, from_=2, to=50, width=5, 
                                       textvariable=self.history_var, command=self.apply_settings)
        self.history_spin.grid(row=0, column=3, padx=5, pady=5, sticky=tk.W)
        
    def apply_settings(self, event=None, initial=False):
        """Apply the current settings to the GPS reader.
        
        Args:
            event: Event that triggered this call (optional)
            initial: True if this is the initial application of settings
        """
        method = self.smoothing_var.get()
        history_size = None

        # Validate history size only if moving average is selected
        if method == "Moving Average":
            try:
                history_size = int(self.history_var.get())
                if history_size < 1:
                    raise ValueError("History size must be positive")
            except ValueError:
                if not initial:  # Don't show error on initial setup
                    messagebox.showerror("Settings Error", 
                                        "Invalid history size. Please enter a positive integer (e.g., 10).")
                # Revert to default
                self.history_var.set(str(MAX_POSITION_HISTORY))
                history_size = MAX_POSITION_HISTORY

        # Apply to reader if it exists
        if self.app.gps_reader:
            self.app.gps_reader.set_smoothing(method, history_size)
            status_msg = f"Smoothing: {method}"
            if method == "Moving Average":
                status_msg += f", History: {history_size}"
            self.app.status_var.set(status_msg)

        # Update UI state
        self._update_ui_state(method)
        
    def _update_ui_state(self, method: str):
        """Update UI components based on selected smoothing method.
        
        Args:
            method: Selected smoothing method
        """
        history_state = "readonly" if method == "Moving Average" else tk.DISABLED
        self.history_spin.config(state=history_state)
        history_label_state = tk.NORMAL if method == "Moving Average" else tk.DISABLED
        self.history_label.config(state=history_label_state)