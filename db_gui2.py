import sys
import time
import threading
from queue import Queue
from typing import Optional, List

import serial
import serial.tools.list_ports
import pyvisa
import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot
from PyQt5.QtWidgets import (
    QDialog,
    QVBoxLayout,
    QComboBox,
    QPushButton,
    QLabel,
    QHBoxLayout,
    QAction,
    QMainWindow,
    QApplication,
    QFileDialog,
)
from PyQt5 import uic

NORMAL_SPEED = 800  # Hz
DEFAULT_SCAN_SPEED = 150  # Hz
REACHED_POSITION_TIMEOUT = 60  # s
UPDATE_INTERVAL = 0.2  # s - has to match arduino


class SerialThread(QThread):
    """
    A thread class for handling serial port operations.
    """

    received_data = pyqtSignal(str)
    error_occurred = pyqtSignal(str)

    def __init__(self, parent):
        super().__init__(parent)
        self.ser = None
        self._is_running = False
        self._write_queue = Queue()
        self._lock = threading.Lock()

    def setup_port(self, port_name: str, baud_rate: int):
        """Initialize the serial port connection.

        Args:
            port_name: Name of the serial port (e.g., 'COM3')
            baud_rate: Baud rate for communication

        Returns:
            bool: True if connection was successful, False otherwise
        """
        try:
            with self._lock:
                self.ser = serial.Serial(port_name, baud_rate, timeout=1)
                self._is_running = True
            return True
        except serial.SerialException as e:
            self.error_occurred.emit(f"Error opening serial port: {e}")
            return False

    def run(self):
        """Main thread loop for handling serial communication."""
        while self._is_running and self.ser and self.ser.is_open:
            with self._lock:
                # Handle writing
                while not self._write_queue.empty():
                    data = self._write_queue.get()
                    try:
                        self.ser.write(data)
                        self.ser.flush()
                    except serial.SerialException as e:
                        self.error_occurred.emit(f"Write error: {e}")
                        break
                # Handle reading
                try:
                    if self.ser.in_waiting:
                        data = self.ser.readline().decode(errors="ignore").strip()
                        # self.ser.reset_input_buffer()
                        if data:
                            self.received_data.emit(data)
                except serial.SerialException as e:
                    self.error_occurred.emit(f"Read error: {e}")
                    break
            time.sleep(0.01)  # Small delay to prevent busy looping
        if self.ser and self.ser.is_open:  # Not actually sure what and why...
            self.ser.close()

    def stop(self) -> None:
        """Safely stop the thread and close the serial port."""
        with self._lock:
            self._is_running = False
            if self.ser and self.ser.is_open:
                self.ser.close()
        self.quit()
        self.wait(1000)  # Wait up to 1 second for thread to finish

    def write_data(self, data: str) -> bool:
        """Write queue data to the serial port.

        Args:
            data: String data to send

        Returns:
            bool: True if data was queued successfully, False otherwise
        """
        with self._lock:
            if not self.ser or not self.ser.is_open:
                self.error_occurred.emit("Serial port is not open")
                return False
            try:
                self._write_queue.put(data.encode())
                return True
            except Exception as e:
                self.error_occurred.emit(f"Error queuing data: {e}")
                return False


class ConnectionDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Connect to Arduino")
        self.setMinimumWidth(300)

        layout = QVBoxLayout()

        # Port selection
        layout.addWidget(QLabel("Select COM Port:"))
        self.port_combo = QComboBox()
        self._populate_serial_ports()
        layout.addWidget(self.port_combo)

        # Baudrate selection
        layout.addWidget(QLabel("Select Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200"])
        self.baud_combo.setCurrentText("38400")
        layout.addWidget(self.baud_combo)

        # Connect button
        btn_layout = QHBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.accept)
        btn_layout.addWidget(self.connect_btn)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def _populate_serial_ports(self) -> None:
        """Populate the COM port dropdown with available ports."""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.port_combo.addItem("No ports found", None)
            return
        for port in sorted(ports, key=lambda p: p.device):
            self.port_combo.addItem(port.description, port.device)

    def get_selection(self):
        """Get the user's port and baud rate selection.

        Returns:
            tuple: (port_name, baud_rate) where port_name may be None if no port selected
        """
        return self.port_combo.currentData(), self.baud_combo.currentText()


class KeithleyController:
    """Controller for Keithley picoammeter communication."""

    def __init__(self):
        self._resource = None
        self._rm = pyvisa.ResourceManager()

    def connect(self, address: str = "GPIB0::14::INSTR") -> bool:
        """Initialize connection to Keithley picoammeter.

        Args:
            address: VISA resource address

        Returns:
            bool: True if connection was successful
        """
        try:
            if self._resource:
                self.disconnect()

            self._resource = self._rm.open_resource(address)
            self._resource.timeout = 1000
            self._resource.write("*RST")
            self._resource.write("SYST:ZCH OFF")  # Turn off zero check
            self._resource.write(":SENS:FUNC 'CURR'")
            self._resource.write("CURR:NPLC 1")  # Faster reading
            return True
        except Exception as e:
            print(f"Failed to setup Keithley: {str(e)}")
            return False

    def disconnect(self) -> None:
        """Close the connection to the Keithley."""
        if self._resource:
            self._resource.close()
            self._resource = None

    def read_current(self) -> Optional[float]:
        """Read current measurement from Keithley.

        Returns:
            float: Current reading in amperes, or None if read failed
        """
        if not self._resource:
            return None

        try:
            return float(self._resource.query(":READ?").split(",")[0][:-1])
        except Exception as e:
            print(f"Keithley read error: {str(e)}")
            return None


class DB_Main_Window(QMainWindow):
    update_plot_signal = pyqtSignal(list, list)

    def __init__(self, testing=False):
        self.testing = testing
        super().__init__()
        uic.loadUi("db_gui2.ui", self)
        self.SL_pos_field.setToolTip("0 mm - OUT, 123 mm - IN")
        self._init_menu_bar()
        self._init_components()
        self._setup_functions()
        self._setup_plot()
        self._init_scan_state()

    def _setup_plot(self) -> None:
        """Configure the initial plot settings."""
        self.plotWidget.setBackground("w")
        self.plotWidget.clear()
        self.plotWidget.setLabel("left", "Current (A)")
        self.plotWidget.setLabel("bottom", "Slit position (mm)")

    def _init_components(self) -> None:
        """Initialize thread and ampermeter"""
        self.serial_thread = SerialThread(self)
        self.serial_thread.received_data.connect(self._handle_received_data)
        self.serial_thread.error_occurred.connect(self._log_message)
        self.update_plot_signal.connect(self._update_scan_plot)

        self.keithley = KeithleyController()

    def _init_menu_bar(self) -> None:
        """Initialize the menu bar."""
        menubar = self.menuBar()
        conn_menu = menubar.addMenu("Connection")

        # Arduino connection actions
        connect_action = QAction("Connect to Arduino", self)
        connect_action.triggered.connect(self._show_connection_dialog)
        conn_menu.addAction(connect_action)

        self.disconnect_action = QAction("Disconnect", self)
        self.disconnect_action.triggered.connect(self._disconnect_serial)
        self.disconnect_action.setEnabled(False)
        conn_menu.addAction(self.disconnect_action)

        conn_menu.addSeparator()

        # Keithley connection actions
        connect_keithley_action = QAction("Connect to Keithley", self)
        connect_keithley_action.triggered.connect(self._connect_keithley)
        conn_menu.addAction(connect_keithley_action)

    def _setup_functions(self) -> None:
        """Setup function connections for UI elements."""
        # Control buttons
        self.FC_In_Btn.clicked.connect(lambda: self._send_serial_command("FC_IN\n"))
        self.FC_Out_Btn.clicked.connect(lambda: self._send_serial_command("FC_OUT\n"))
        self.FC_Stop_Btn.clicked.connect(lambda: self._send_serial_command("FC_STOP\n"))
        self.SL_In_Btn.clicked.connect(lambda: self._send_serial_command("SL_IN\n"))
        self.SL_Out_Btn.clicked.connect(lambda: self._send_serial_command("SL_OUT\n"))
        self.SL_Stop_Btn.clicked.connect(lambda: self._send_serial_command("SL_STOP\n"))

        # Movement fields
        self.mvto_field.returnPressed.connect(self._send_move_to)
        self.mvby_field.returnPressed.connect(self._send_move_by)

        # Scan buttons
        self.hStartScanBtn.clicked.connect(
            lambda: self._run_scan(
                self.hStartPosField, self.hEndPosField, self.slitScanSpeedFiled
            )
        )
        self.hStopScanBtn.clicked.connect(self._cleanup_scan)
        self.vStartScanBtn.clicked.connect(
            lambda: self._run_scan(
                self.vStartPosField, self.vEndPosField, self.slitScanSpeedFiled
            )
        )
        self.vStopScanBtn.clicked.connect(self._cleanup_scan)

        # Data saving
        self.saveBtn.clicked.connect(self._save_scan_data)

    def _init_scan_state(self) -> None:
        """Initialize variables related to scanning."""
        self.scan_data = []
        self.is_scanning = False
        self._scan_interrupt = False
        self._scan_position = None  # Shared position for scan routine
        self._scan_sl_status = None  # Track SL status
        self._pos_reached_event = threading.Event()
        self._ack_event = threading.Event()

    # Serial connection methods
    def _show_connection_dialog(self) -> None:
        """Show the serial port connection dialog."""
        dialog = ConnectionDialog(self)
        if dialog.exec_():
            port, baud = dialog.get_selection()
            self._connect_serial(port, baud)

    def _connect_serial(self, port: str, baud: str) -> None:
        """Connect to the specified serial port.

        Args:
            port: Serial port name
            baud: Baud rate as string
        """
        if not port:
            self._log_message("No port selected.")
            return

        try:
            baud_rate = int(baud)
        except ValueError:
            self._log_message(f"Invalid baud rate: {baud}")
            return

        if self.serial_thread.setup_port(port, baud_rate):
            self.serial_thread.start()
            self._log_message(f"Connected to {port}.")
            self.disconnect_action.setEnabled(True)
        else:
            self._log_message(f"Failed to connect to {port}.")
            self.disconnect_action.setEnabled(False)

    def _disconnect_serial(self) -> None:
        """Disconnect from the serial port."""
        self.serial_thread.stop()
        self._log_message("Disconnected from serial port.")
        self.disconnect_action.setEnabled(False)

    def _connect_keithley(self) -> None:
        """Connect to the Keithley picoammeter."""
        if self.keithley.connect():
            self._log_message("Keithley connected successfully.")
        else:
            self._log_message("Failed to connect to Keithley.")

    # =========== Communication with arduino ================
    def _send_serial_command(self, command: str) -> None:
        """Send a command to the Arduino via serial.

        Args:
            command: The command string to send
        """
        if not self.serial_thread.isRunning():
            self._log_message("Error: Not connected to COM port!")
            return

        if self.serial_thread.write_data(command):
            self._log_message(f"Sent: {command.strip()}")
        else:
            self._log_message("Failed to send command.")

    def _send_move_to(self) -> None:
        """Handle move-to command from the UI."""
        if not self.serial_thread.isRunning():
            self._log_message("Error: Not connected to COM port!")
            return
        try:
            pos = float(self.mvto_field.text())
            self._send_serial_command(f"SL_TO#{pos}")
        except ValueError:
            self._log_message("Position must be a number!")

    def _send_move_by(self) -> None:
        """Handle move-by command from the UI."""
        if not self.serial_thread.isRunning():
            self._log_message("Error: Not connected to COM port!")
            return
        try:
            val = float(self.mvby_field.text())
            if val > 0:
                self._send_serial_command(f"SL_IN_BY#{val}")
            elif val < 0:
                self._send_serial_command(f"SL_OUT_BY#{abs(val)}")
        except ValueError:
            self._log_message("Distance must be a number!")

    # ================== Displaying data ================
    def _handle_received_data(self, data: str) -> None:
        """Process data received from the Arduino.

        Args:
            data: The received data string
        """
        if data.startswith("&FC"):
            self._process_status_data(data)
        elif "ACK" in data:
            self._ack_event.set()
        else:
            self._log_message(data)

    def _process_status_data(self, data: str) -> None:
        """Process status update data from Arduino.

        Args:
            data: The status data string
        """
        try:
            parts = data.split("&")
            fc_status = parts[1].split(":")[1]
            sl_status = parts[2].split(":")[1]

            fc_ls_in, fc_ls_out, fc_state, _ = fc_status.split(";")
            sl_ls_in, sl_ls_out, sl_state, pos = sl_status.split(";")

            self._scan_position = float(pos)
            self._scan_sl_status = sl_state

            # Update position reached event if waiting
            if hasattr(self, "_target_position"):
                if (
                    abs(self._scan_position - self._target_position) < 0.5
                    and self._scan_sl_status == "0"
                ):
                    if hasattr(self, "_pos_reached_event"):
                        self._pos_reached_event.set()

            # Update status indicators
            self._update_status_indicators(
                fc_ls_in, fc_ls_out, sl_ls_in, sl_ls_out, pos
            )
        except Exception as e:
            self._log_message(f"Error parsing status data: {e}")

    def _update_status_indicators(
        self, fc_ls_in: str, fc_ls_out: str, sl_ls_in: str, sl_ls_out: str, pos: str
    ) -> None:
        """Update the status indicator LEDs and position display.

        Args:
            fc_ls_in: FC limit switch in status
            fc_ls_out: FC limit switch out status
            sl_ls_in: SL limit switch in status
            sl_ls_out: SL limit switch out status
            pos: Current position
        """

        self.FC_LS_IN_status.setStyleSheet(
            "background-color: red; border-radius: 10px;"
            if fc_ls_in == "1"
            else "background-color: grey; border-radius: 10px;"
        )
        self.FC_LS_OUT_status.setStyleSheet(
            "background-color: red; border-radius: 10px;"
            if fc_ls_out == "1"
            else "background-color: grey; border-radius: 10px;"
        )
        self.SL_LS_IN_status.setStyleSheet(
            "background-color: red; border-radius: 10px;"
            if sl_ls_in == "1"
            else "background-color: grey; border-radius: 10px;"
        )
        self.SL_LS_OUT_status.setStyleSheet(
            "background-color: red; border-radius: 10px;"
            if sl_ls_out == "1"
            else "background-color: grey; border-radius: 10px;"
        )
        self.SL_pos_field.setText(f"{pos} mm")

    def _log_message(self, message: str) -> None:
        """Add a message to the log display.

        Args:
            message: The message to log
        """
        self.logs_field.append(message)

    # ================= Scan routine ================
    def _run_scan(self, start_field=None, end_field=None, speed_field=None) -> None:
        """Run a scan between the specified positions.

        Args:
            start_field: The QLineEdit containing the start position
            end_field: The QLineEdit containing the end position
        """
        if not self.serial_thread.isRunning():
            self._log_message("Error: Not connected to COM port!")
            return

        if not self.keithley._resource:
            self._log_message("No connection to Keithley. Cannot start the scan")
            return

        if self.is_scanning:
            self._log_message("Scan already in progress.")
            return

        try:
            pos1 = float(start_field.text())
            pos2 = float(end_field.text())
        except ValueError:
            self._log_message("Invalid scan positions.")
            return

        try:
            scan_speed = int(speed_field.text())
        except ValueError:
            self._log_message(f"Invalid speed. Setting {DEFAULT_SCAN_SPEED}Hz")
            scan_speed = DEFAULT_SCAN_SPEED

        self.scan_data = []
        self.is_scanning = True
        self._scan_interrupt = False
        # TODO: use qthread
        threading.Thread(
            target=self._scan_routine, args=(pos1, pos2, scan_speed), daemon=True
        ).start()

    def _cleanup_scan(self) -> None:
        """Clean up after a scan completes or is interrupted."""
        if not self.is_scanning:
            self._log_message("Scan wasn't running")
            return

        self.is_scanning = False
        self._scan_interrupt = True
        self._pos_reached_event.set()  # Unblock any wait for position
        self._ack_event.set()  # Unblock any wait for ACK

        self._send_serial_command("SL_STOP\n")
        self._send_serial_command("SL_FR#800\n")
        self._log_message("Scan stopped")

    def _scan_routine(self, start: float, end: float, scan_speed: int) -> None:
        """The actual scan routine that runs in a separate thread.

        Args:
            start: Start position of the scan
            end: End position of the scan
        """
        try:
            # 1) Move to nearest boundary if not already there
            current_pos = self._scan_position
            if current_pos is None:
                self._log_message("Current position unknown. Cannot start scan.")
                return

            dist_to_start = abs(current_pos - start)
            dist_to_end = abs(current_pos - end)
            nearest = start if dist_to_start < dist_to_end else end
            farthest = end if nearest == start else start

            self._send_serial_command(f"SL_FR#{NORMAL_SPEED}\n")
            time.sleep(0.2)

            self._pos_reached_event.clear()
            self._target_position = nearest

            if abs(current_pos - nearest) > 0.5:
                self._send_serial_command(f"SL_TO#{nearest}\n")
                self._log_message(f"Moving to nearest scan boundary: {nearest} mm")
            else:
                self._log_message(f"Already near scan boundary: {nearest} mm")
                self._pos_reached_event.set()

            # Wait until position is reached (with timeout)
            if not self._pos_reached_event.wait(timeout=REACHED_POSITION_TIMEOUT):
                self._log_message("Timeout waiting for position.")
                return
            #! self._pos_reached_event.clear()

            # 2) Reduce speed for scan
            self._send_serial_command(f"SL_FR#{scan_speed}\n")

            # 3) Start scan: move to farthest boundary
            direction = "SL_IN_BY" if farthest > nearest else "SL_OUT_BY"
            distance = abs(farthest - nearest)
            self._send_serial_command(f"{direction}#{distance}\n")

            self._ack_event.clear()
            if not self._ack_event.wait(timeout=5):
                self._log_message("Timeout waiting for ACK from Arduino.")
                return

            self._log_message(f"Scanning from {nearest} mm to {farthest} mm")
            scan_start_time = time.time()

            # Prepare lists for plotting
            times = []
            positions = []
            currents = []

            while not self._scan_interrupt:
                pos = self._scan_position
                sl_status = self._scan_sl_status
                t = time.time() - scan_start_time

                # Read current from Keithley
                current = self.keithley.read_current()
                if current is None:
                    current = float("nan")

                self.scan_data.append((t, pos, current))
                times.append(t)
                positions.append(pos)
                currents.append(current)

                # Update plot in main thread
                self.update_plot_signal.emit(positions, currents)

                # Check if we've reached the target position
                if pos is not None and abs(pos - farthest) < 0.5 and sl_status == "0":
                    break

                time.sleep(UPDATE_INTERVAL)  # Has to match arduino

        except Exception as e:
            self._log_message(f"Scan error: {e}")
        finally:
            self._cleanup_scan()

    # =========================================

    def _update_scan_plot(self, positions: List[float], currents: List[float]) -> None:
        """Update the scan plot with new data.

        Args:
            positions: List of position values
            currents: List of current values
        """
        self.plotWidget.clear()
        pen = pg.mkPen(color=(0, 0, 255), width=1)
        self.plotWidget.plot(
            positions, currents, symbol="o", symbolSize=6, symbolBrush="b", pen=pen
        )
        self.plotWidget.setLabel("left", "Current (A)")
        self.plotWidget.setLabel("bottom", "Slit position (mm)")

    def _save_scan_data(self) -> None:
        """Save the collected scan data to a CSV file."""
        if not self.scan_data:
            self._log_message("No scan data to save.")
            return

        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Scan Data",
            "",
            "CSV Files (*.csv);;All Files (*)",
            options=QFileDialog.Options(),
        )

        if not filename:
            self._log_message("Save cancelled.")
            return

        try:
            with open(filename, "w", newline="") as f:
                import csv

                writer = csv.writer(f)
                writer.writerow(["time_s", "position_mm", "current_A"])
                writer.writerows(self.scan_data)
            self._log_message(f"Scan data saved to {filename}")
        except Exception as e:
            self._log_message(f"Failed to save scan data: {e}")

    def closeEvent(self, event) -> None:
        """Handle window close event."""
        self.serial_thread.stop()
        self.keithley.disconnect()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = DB_Main_Window()
    main.show()
    sys.exit(app.exec_())
