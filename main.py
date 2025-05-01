#!/usr/bin/env python3
"""GPS Data Viewer - Main entry point for the application."""

import tkinter as tk
import platform
from ui.app import GPSApp

def main():
    """Main function to create and run the application."""
    # Windows DPI awareness
    if platform.system() == "Windows":
        try:
            from ctypes import windll
            windll.shcore.SetProcessDpiAwareness(1)
        except Exception:
            pass
            
    # Create and run the application
    root = tk.Tk()
    app = GPSApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()

if __name__ == "__main__":
    main()