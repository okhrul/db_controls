import sys
import time
import threading
from queue import Queue
from typing import Optional, Tuple, List

import serial
import serial.tools.list_ports
import pyvisa
import pyqtgraph as pg
from PyQt5.QtCore import pyqtSignal, QThread, pyqtSlot, QMetaObject, Qt
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
        while self.is_running and self.ser and self.ser.is_open:
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
            return float(self._resource.query(":READ?").split(",")[0].strip())
        except Exception as e:
            print(f"Keithley read error: {str(e)}")
            return None


class DB_Main_Window(QMainWindow):
    def __init__(self):
        super().__init__()

        # Load UI file created for QWidget into the central widget of QMainWindow
        uic.loadUi("db_gui2.ui", self)
        # self.setFixedSize(central_widget.size())

        self.SL_pos_field.setToolTip("0 mm - OUT, 123 mm - IN")
        # self.plotWidget = pg.PlotWidget()
        self.plotWidget.setBackground("w")
        self.plotWidget.clear()
        pen = pg.mkPen(color=(0, 0, 255), width=1)
        time = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        temperature = [30, 32, 34, 32, 33, 31, 29, 32, 35, 45]
        self.plotWidget.plot(
            time, temperature, symbol="o", symbolSize=6, symbolBrush="b", pen=pen
        )
        self.plotWidget.setLabel("left", "Current (A)")
        self.plotWidget.setLabel("bottom", "Slit position (mm)")

        # Serial thread setup
        self.serial_thread = SerialThread(self)
        self.serial_thread.received_data.connect(self.update_received_data)
        self.serial_thread.error_occurred.connect(self.log_message)

        # Setup menu bar
        self.init_menu_bar()

        # Connect control buttons
        self.FC_In_Btn.clicked.connect(lambda: self.send_serial_command("FC_IN\n"))
        self.FC_Out_Btn.clicked.connect(lambda: self.send_serial_command("FC_OUT\n"))
        self.FC_Stop_Btn.clicked.connect(lambda: self.send_serial_command("FC_STOP\n"))
        self.SL_In_Btn.clicked.connect(lambda: self.send_serial_command("SL_IN\n"))
        self.SL_Out_Btn.clicked.connect(lambda: self.send_serial_command("SL_OUT\n"))
        self.SL_Stop_Btn.clicked.connect(lambda: self.send_serial_command("SL_STOP\n"))
        self.mvto_field.returnPressed.connect(self.send_mvto)
        self.mvby_field.returnPressed.connect(self.send_mvby)

        # Scan state
        self.scan_data = []
        self.is_scanning = False
        self._scan_interrupt = False
        self._scan_position = None  # Shared position for scan routine
        self._scan_sl_status = None  # track SL status
        self._pos_reached_event = threading.Event()
        self._ack_event = threading.Event()

        self.hStartScanBtn.clicked.connect(
            lambda: self.run_scan(self.hStartPosField, self.hEndPosField)
        )
        self.hStopScanBtn.clicked.connect(self.scan_cleanup)
        self.vStartScanBtn.clicked.connect(
            lambda: self.run_scan(self.vStartPosField, self.vEndPosField)
        )
        self.vStopScanBtn.clicked.connect(self.scan_cleanup)

        self.saveBtn.clicked.connect(self.save_scan_data)

    # ================ Connection ======================
    def init_menu_bar(self):
        menubar = self.menuBar()
        conn_menu = menubar.addMenu("Connection")

        connect_action = QAction("Connect to Arduino", self)
        connect_action.triggered.connect(self.show_connection_dialog)
        conn_menu.addAction(connect_action)

        self.disconnect_action = QAction(
            "Disconnect", self
        )  # Store as instance variable
        self.disconnect_action.triggered.connect(self.disconnect_serial)
        self.disconnect_action.setEnabled(False)  # Initially disabled
        conn_menu.addAction(self.disconnect_action)

        conn_menu.addSeparator()  # <-- Divider between Arduino and Keithley

        connect_keithley_action = QAction("Connect to Keithley", self)
        connect_keithley_action.triggered.connect(self.connect_keithley_dialog)
        conn_menu.addAction(connect_keithley_action)

    def show_connection_dialog(self):
        dialog = ConnectionDialog(self)
        if dialog.exec_():
            port, baud = dialog.get_selection()
            self.connect_serial(port, baud)

    def connect_serial(self, port, baud):
        if not port or not baud:
            self.log_message("Missing port or baudrate.")
            return
        try:
            self.serial_thread.setup_port(port, baud)
            if self.serial_thread.ser and self.serial_thread.ser.is_open:
                self.serial_thread.start()
                self.log_message(f"Connected to {port}.")
                self.disconnect_action.setEnabled(True)  # Enable on connect
            else:
                self.log_message(f"Failed to connect to {port}. Port might be in use.")
                self.disconnect_action.setEnabled(False)
        except Exception as e:
            self.log_message(f"Error opening serial port: {e}")
            self.disconnect_action.setEnabled(False)

    def disconnect_serial(self):
        if self.serial_thread.is_running:
            self.serial_thread.stop()
            self.log_message("Disconnected from serial port.")
        self.disconnect_action.setEnabled(False)  # Disable on disconnect

    def connect_keithley_dialog(self):
        # For simplicity, just use default address. You can expand this to a dialog if needed.
        self.keithley = setup_keithley()
        if self.keithley is not None:
            self.log_message("Keithley connected successfully.")
        else:
            self.log_message("Failed to connect to Keithley.")

    # ================= Communication with arduino ================

    @pyqtSlot(str)
    def send_serial_command(self, command):
        if self.serial_thread.is_running:
            try:
                self.serial_thread.write_data(command)
                self.log_message(f"Sent: {command.strip()}")
            except Exception as e:
                self.log_message(str(e))
        else:
            self.log_message("Error: Not connected to COM port!")

    @pyqtSlot()
    def send_mvto(self):
        if self.serial_thread.is_running:
            try:
                pos = float(self.mvto_field.text())
                self.send_serial_command(f"SL_TO#{pos}")
            except ValueError:
                self.log_message("Position must be a number!")
        else:
            self.log_message("Error: Not connected to COM port!")

    @pyqtSlot()
    def send_mvby(self):
        if self.serial_thread.is_running:
            try:
                val = float(self.mvby_field.text())
                if val > 0:
                    self.send_serial_command(f"SL_IN_BY#{val}")
                elif val < 0:
                    self.send_serial_command(f"SL_OUT_BY#{abs(val)}")
            except ValueError:
                self.log_message("Distance must be a number!")
        else:
            self.log_message("Error: Not connected to COM port!")

    # ================== Displaying data ================

    @pyqtSlot(str)
    def update_received_data(self, data):
        if data.startswith("&FC"):
            try:
                fc_status = data.split("&")[1].split(":")[1]
                sl_status = data.split("&")[2].split(":")[1]
                fc_ls_in, fc_ls_out, fc_state, _ = fc_status.split(";")
                sl_ls_in, sl_ls_out, sl_state, pos = sl_status.split(";")
                self._scan_position = float(pos)
                self._scan_sl_status = sl_state

                # Check if waiting for position
                if hasattr(self, "_target_position"):
                    if (
                        abs(self._scan_position - self._target_position) < 0.5
                        and self._scan_sl_status == "0"
                    ):
                        if hasattr(self, "_pos_reached_event"):
                            self._pos_reached_event.set()
            except:
                self.log_message(f"Unknown status format: {data}")
                return

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
        elif (
            "ACK" in data
        ):  # mostly for scan routine, to collect data only after the movement has started
            self._ack_event.set()
        else:
            self.log_message(data)

    @pyqtSlot(str)
    def log_message(self, message):
        self.logs_field.append(message)

    # ================= Scan routine ================
    def run_scan(self, start_field=None, end_field=None):
        if not self.serial_thread.is_running:
            self.log_message("Error: Not connected to COM port!")
            return
        if self.keithley is None:
            self.log_message("No connection to Keithley. Cannot start the scan")
            return
        if self.is_scanning:
            self.log_message("Scan already in progress.")
            return
        try:
            pos1 = float(start_field.text())
            pos2 = float(end_field.text())
        except ValueError:
            self.log_message("Invalid scan positions.")
            return

        self.scan_data = []
        self.is_scanning = True
        self._scan_interrupt = False

        # Start scan in a thread
        print("Started thread")
        threading.Thread(
            target=self.scan_routine, args=(pos1, pos2), daemon=True
        ).start()

    def scan_cleanup(self):
        """Cleanup after scan interruption or finish."""
        print("Stopping...")
        if self.is_scanning:
            self.is_scanning = False
            self._scan_interrupt = True
            self._pos_reached_event.set()  # Unblock any wait for position
            self._ack_event.set()  # Unblock any wait for ACK
            self.serial_thread.write_data("SL_STOP\n")
            self.serial_thread.write_data("SL_FR#800\n")
            self.log_message("Scan stopped")
        else:
            self.log_message("Scan wasn't running")

    def scan_routine(self, start, end):
        try:
            normal_speed = 800
            scan_speed = 200

            # 1) Move to nearest boundary if not already there
            current_pos = self._scan_position
            if current_pos is None:
                self.log_message("Current position unknown. Cannot start scan.")
                return
            print("Current position ", current_pos)

            dist_to_start = abs(current_pos - start)
            dist_to_end = abs(current_pos - end)
            nearest = start if dist_to_start < dist_to_end else end
            farthest = end if nearest == start else start

            self.serial_thread.write_data(f"SL_FR#{normal_speed}\n")
            time.sleep(0.2)  # * do we need it?
            self._pos_reached_event.clear()
            self._target_position = nearest
            if abs(current_pos - nearest) > 0.5:
                self.serial_thread.write_data(f"SL_TO#{nearest}\n")
                self.log_message(f"Moving to nearest scan boundary: {nearest} mm")
            else:
                self.log_message(f"Already near scan boundary: {nearest} mm")
                self._pos_reached_event.set()  # Already there

            # 1.2) Wait until position is reached (with timeout)
            if not self._pos_reached_event.wait(timeout=60):
                self.log_message("Timeout waiting for position.")
                return

            # 2) Reduce speed for scan
            self.serial_thread.write_data(f"SL_FR#{scan_speed}\n")

            # 4) Start scan: move to farthest boundary
            direction = "SL_IN_BY" if farthest > nearest else "SL_OUT_BY"
            distance = abs(farthest - nearest)
            self.serial_thread.write_data(f"{direction}#{distance}\n")
            self._ack_event.clear()
            if not self._ack_event.wait(timeout=5):
                self.log_message("Timeout waiting for ACK from Arduino.")
                return

            self.log_message(f"Scanning from {nearest} mm to {farthest} mm")
            scan_start_time = time.time()

            # Prepare lists for plotting
            times = []
            positions = []
            currents = []

            while not self._scan_interrupt:
                pos = self._scan_position  # ? Do I get a position like this??
                sl_status = self._scan_sl_status
                t = time.time() - scan_start_time

                # Read current from Keithley
                try:
                    current = float(self.keithley.query(":READ?").split(",")[0].strip())
                except Exception as e:
                    print(f"Keithley read error: {str(e)}")
                    current = float("nan")

                self.scan_data.append((t, pos, current))
                times.append(t)
                positions.append(pos)
                currents.append(current)

                # Update plot in main thread
                QMetaObject.invokeMethod(
                    self,
                    lambda: self.update_scan_plot(positions, currents),
                    Qt.QueuedConnection,
                )

                # Check both position and sl_status
                if pos is not None and abs(pos - farthest) < 0.5 and sl_status == "0":
                    break

                time.sleep(0.2)  # Slow down polling
            if self._scan_interrupt:
                self.log_message("Scan interrupted by user.")
        except Exception as e:
            self.log_message(f"Scan error: {e}")
        finally:
            self.scan_cleanup()

    # =========================================

    def save_scan_data(self):
        """Prompt user to save scan data to a CSV file."""
        if not self.scan_data:
            self.log_message("No scan data to save.")
            return
        options = QFileDialog.Options()
        filename, _ = QFileDialog.getSaveFileName(
            self,
            "Save Scan Data",
            "",
            "CSV Files (*.csv);;All Files (*)",
            options=options,
        )
        if not filename:
            self.log_message("Save cancelled.")
            return
        import csv

        try:
            with open(filename, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(["time_s", "position_mm", "current_A"])
                writer.writerows(self.scan_data)
            self.log_message(f"Scan data saved to {filename}")
        except Exception as e:
            self.log_message(f"Failed to save scan data: {e}")

    def update_scan_plot(self, positions, currents):
        """Update the scan plot with new data."""
        self.plotWidget.clear()
        pen = pg.mkPen(color=(0, 0, 255), width=1)
        self.plotWidget.plot(
            positions, currents, symbol="o", symbolSize=6, symbolBrush="b", pen=pen
        )
        self.plotWidget.setLabel("left", "Current (A)")
        self.plotWidget.setLabel("bottom", "Slit position (mm)")

    def closeEvent(self, event):
        if self.serial_thread.is_running:
            self.serial_thread.stop()
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main = DB_Main_Window()
    main.show()
    sys.exit(app.exec_())
