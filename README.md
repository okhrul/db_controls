# DBox GUI.

GUI communicates with an Arduino to control slit and Faraday cup movements, and with picoammeter to read current.

During a scan, the slit is first moved to the nearest boundary. The speed is then reduced to a set value, and the system begins continuously reading the slit position from the Arduino and current from the Keithley picoammeter every 200 ms. This data is live-plotted in the GUI and can be saved in csv.


## Requirements
- Python 3.9 (PyQt tools don't seem to work with newer versions)
- Drivers for the Keithley picoammeter
  
Install dependencies with:
```sh
pip install pyqt5 pyqtgraph pyserial pyvisa numpy scipy
```

## Usage

1. Connect Arduino and Keithley to PC
2. Run the db_gui2.py
3. Connect Devices:
   - Use the "Connection" menu to connect to the Arduino (COM port is different on different PCs, baudrate is set to 38400).
   - Connect to the Keithley via the same menu if you want to measure profiles.

Now it's ready to be used.

## Notes
- The system status is always displayed as **Unknown** *(to be fixed)*.
- The FC can only be moved **IN** and **OUT**, since there is no position reading.
- The slit can be moved:
  - To an **absolute** position (in mm)
  - Or **relatively** from the current position  
    *Reference values: `57.5 mm` = horizontal slit in, `100.5 mm` = vertical slit in*
## Troubleshooting
  - The GUI tends to crash or freeze from time to time. Just restart it (this is to be fixed)
  - If the Keithley connection fails, restart the picoammeter.


