# Get GPS Data

[![GitHub license](https://img.shields.io/github/license/Lazarev-Cloud/Get-GPS-data)](./LICENSE)
[![GitHub issues](https://img.shields.io/github/issues/Lazarev-Cloud/Get-GPS-data)](https://github.com/Lazarev-Cloud/Get-GPS-data/issues)

## Overview

The **Get GPS Data** project provides a Python-based solution for interfacing with a USB-to-serial adapter module and a GY-GPS6MV2 GPS module equipped with a u-blox NEO-6M receiver. This repository aims to simplify the integration of GPS modules into your applications by offering tools to retrieve, process, and analyze GPS data.

---

## Features

- **Real-Time GPS Data Retrieval:** Fetch real-time data from the u-blox NEO-6M GPS receiver.
- **Serial Communication:** Communicate effectively with the GPS module via a USB-to-serial adapter.
- **Data Parsing:** Extract latitude, longitude, altitude, and other geolocation data.
- **Position Filtering:** Apply Kalman filter or moving average smoothing to improve position accuracy.
- **Satellite Monitoring:** Visualize satellite information and signal quality.
- **Data Logging:** Record GPS data to CSV for analysis and tracking.
- **Error Handling:** Robust error handling for serial communication interruptions.
- **Extensible:** Modular architecture ready to be integrated into larger projects.

---

## Getting Started

Follow these instructions to set up the project and start retrieving GPS data from your module.

### Prerequisites

Ensure you have the following installed on your system:

- Python (>=3.7)
- A USB-to-serial adapter module
- A GY-GPS6MV2 GPS module with a u-blox NEO-6M receiver
- Required Python libraries (see below)

### Installation

1. Clone this repository:

    ```sh
    git clone https://github.com/Lazarev-Cloud/Get-GPS-data.git
    cd Get-GPS-data
    ```

2. Install the required Python dependencies:

    ```sh
    pip install -r requirements.txt
    ```

3. Connect your GPS module to your computer via the USB-to-serial adapter.

---

## Usage

To start the GPS Data Viewer application, run:

```sh
python main.py
```

The application provides an intuitive GUI with the following features:
- Connection settings for serial port and baud rate
- Real-time display of GPS data (position, altitude, speed, etc.)
- Position smoothing options (None, Moving Average, Kalman Filter)
- Satellite information view with signal strength visualization
- Raw NMEA sentence display
- Data logging to CSV files

---

## Project Structure

The project uses a modular architecture for better maintainability:

```
get-gps-data/
├── main.py                  # Entry point
├── config/                  # Configuration settings
├── core/                    # Core GPS functionality
│   ├── gps_reader.py        # GPS data acquisition
│   ├── nmea_parser.py       # NMEA parsing
│   └── filters/             # Position filtering algorithms
├── utils/                   # Utility functions
├── logging/                 # Data logging functionality
└── ui/                      # User interface components
    ├── app.py               # Main application
    ├── panels/              # UI panels
    └── windows/             # Additional windows
```

---

## Troubleshooting

### Common Issues

1. **Serial Port Not Found:**
   - Ensure your GPS module is properly connected to the USB port.
   - Check the serial port name (e.g., `/dev/ttyUSB0` on Linux or `COM3` on Windows) and update if necessary.

2. **No GPS Data Received:**
   - Verify that the GPS module has a clear view of the sky to acquire satellite signals.
   - Check for proper power supply to the GPS module.

---

## Contributing

Contributions are welcome! If you'd like to improve this project, please follow these steps:

1. Fork the repository.
2. Create a new branch: `git checkout -b feature/your-feature-name`.
3. Commit your changes: `git commit -m 'Add some feature'`.
4. Push to the branch: `git push origin feature/your-feature-name`.
5. Submit a pull request.

---

## License

This project is licensed under the MIT License. See the [LICENSE](./LICENSE) file for details.

---

## Contact

If you have any questions or suggestions, feel free to contact me via GitHub [@lazarevtill](https://github.com/lazarevtill).
