# -*- coding: utf-8 -*-
import sys, time
from collections import deque
from dataclasses import dataclass
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pandas as pd
import paho.mqtt.client as mqtt

# ===================== Data Model =====================
@dataclass
class Sample:
    t: float
    pressure: int
    flow: float
    temp: float
    motor_on: bool

# ===================== MQTT Source =====================
class MQTTSource(QtCore.QObject):
    new_sample = QtCore.pyqtSignal(Sample)
    connected_changed = QtCore.pyqtSignal(bool)

    def __init__(self, broker="test.mosquitto.org", port=1883, topic="esp32/demo"):
        super().__init__()
        self.broker = broker
        self.port = port
        self.topic = topic
        self.client = mqtt.Client(client_id="laboc_gui")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.loop)
        self.connected = False

    def start(self):
        try:
            self.client.connect(self.broker, self.port, 60)
            self.timer.start(50)  # частіший loop для швидшого оновлення
        except Exception as e:
            print("[MQTT] Connect error:", e)
            self.connected_changed.emit(False)

    def loop(self):
        try:
            self.client.loop(timeout=0.01)
        except Exception:
            pass

    def stop(self):
        try:
            self.client.disconnect()
            self.timer.stop()
        except Exception:
            pass
        self.connected_changed.emit(False)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MQTT] Connected")
            self.connected = True
            self.connected_changed.emit(True)
            self.client.subscribe(self.topic)
        else:
            print("[MQTT] Connect failed with code", rc)
            self.connected = False
            self.connected_changed.emit(False)

    def on_message(self, client, userdata, msg):
        try:
            line = msg.payload.decode()
            parts = line.strip().split(",")
            if len(parts) != 5:
                return
            t = float(parts[0])
            pressure = int(float(parts[1]))
            flow = float(parts[2])
            temp = float(parts[3])
            motor = bool(int(parts[4]))
            self.new_sample.emit(Sample(t, pressure, flow, temp, motor))
        except Exception:
            pass

    def set_motor(self, on: bool):
        try:
            self.client.publish("esp32/motor", "1" if on else "0")
        except Exception:
            pass

# ===================== UI Widgets =====================
class StatCard(QtWidgets.QFrame):
    def __init__(self, title, unit):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.setStyleSheet("background-color: #2d2d2d; border-radius:8px; padding:10px;")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 8, 12, 8)
        self.title = QtWidgets.QLabel(title)
        self.title.setStyleSheet("color: #ccc; font-weight:bold; font-size:13px;")
        self.value = QtWidgets.QLabel("--")
        self.value.setStyleSheet("color: #fff; font-size:16px;")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setStyleSheet("color:#888; font-size:12px;")
        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)
        layout.addStretch(1)

    def set_value(self, text):
        self.value.setText(text)

class Led(QtWidgets.QLabel):
    def __init__(self, diameter=14):
        super().__init__()
        self._diam = diameter
        self.setFixedSize(diameter, diameter)
        self.set_status(False)

    def set_status(self, on: bool, color_on="#22c55e", color_off="#ef4444"):
        color = color_on if on else color_off
        pix = QtGui.QPixmap(self._diam, self._diam)
        pix.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.setBrush(QtGui.QColor(color))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(0, 0, self._diam, self._diam)
        painter.end()
        self.setPixmap(pix)

# ===================== Dashboard =====================
class Dashboard(QtWidgets.QWidget):
    motor_toggled = QtCore.pyqtSignal(bool)
    export_csv = QtCore.pyqtSignal(pd.DataFrame)

    def __init__(self):
        super().__init__()
        self.max_points = 300
        self.buffer = {
            "t": deque(maxlen=self.max_points),
            "pressure": deque(maxlen=self.max_points),
            "flow": deque(maxlen=self.max_points),
            "temp": deque(maxlen=self.max_points),
            "motor_on": deque(maxlen=self.max_points),
        }

        # Root Layout
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12,12,12,12)
        root.setSpacing(12)

        # Картки
        statRow = QtWidgets.QHBoxLayout()
        statRow.setSpacing(12)
        self.cardP = StatCard("Pressure", "Pa")
        self.cardF = StatCard("Flow", "ml/min")
        self.cardT = StatCard("Temperature", "°C")
        statRow.addWidget(self.cardP)
        statRow.addWidget(self.cardF)
        statRow.addWidget(self.cardT)
        root.addLayout(statRow)

        # Мотор та підключення
        motorBar = QtWidgets.QHBoxLayout()
        self.ledConn = Led()
        self.lblConn = QtWidgets.QLabel("Disconnected")
        self.lblConn.setStyleSheet("color:#ef4444;")
        self.ledMotor = Led()
        motorLabel = QtWidgets.QLabel("Motor:")
        self.btnMotor = QtWidgets.QPushButton("Start")
        self.btnMotor.setCheckable(True)
        self.btnMotor.clicked.connect(self._toggle_motor)
        motorBar.addWidget(QtWidgets.QLabel("Connection:"))
        motorBar.addWidget(self.ledConn)
        motorBar.addWidget(self.lblConn)
        motorBar.addSpacing(24)
        motorBar.addWidget(motorLabel)
        motorBar.addWidget(self.ledMotor)
        motorBar.addStretch(1)
        motorBar.addWidget(self.btnMotor)

        # Темний режим
        self.btnTheme = QtWidgets.QPushButton("Toggle Dark Mode")
        self.btnTheme.clicked.connect(self.toggle_theme)
        motorBar.addWidget(self.btnTheme)

        # Експорт CSV
        self.btnExport = QtWidgets.QPushButton("Export Table CSV")
        self.btnExport.clicked.connect(self.export_table)
        motorBar.addWidget(self.btnExport)
        root.addLayout(motorBar)

        # Графіки
        pg.setConfigOptions(antialias=True)
        self.plotP = pg.PlotWidget(title="Pressure [Pa]")
        self.plotF = pg.PlotWidget(title="Flow [ml/min]")
        self.plotT = pg.PlotWidget(title="Temperature [°C]")
        for w in (self.plotP, self.plotF, self.plotT):
            w.showGrid(x=True, y=True, alpha=0.3)
            w.setBackground("#1e1e1e")
            w.getAxis('left').setTextPen("w")
            w.getAxis('bottom').setTextPen("w")
        self.curP = self.plotP.plot(pen=pg.mkPen("#ef4444", width=2))
        self.curF = self.plotF.plot(pen=pg.mkPen("#10b981", width=2))
        self.curT = self.plotT.plot(pen=pg.mkPen("#3b82f6", width=2))
        root.addWidget(self.plotP)
        root.addWidget(self.plotF)
        root.addWidget(self.plotT)

        # Таблиця
        self.table = QtWidgets.QTableWidget(0,5)
        self.table.setHorizontalHeaderLabels(["t [s]", "Pressure", "Flow", "Temp", "Motor"])
        self.table.setStyleSheet("color:#fff; background:#2d2d2d;")
        root.addWidget(self.table)

    def _toggle_motor(self, checked):
        self.ledMotor.set_status(checked, color_on="#22c55e", color_off="#9ca3af")
        self.btnMotor.setText("Stop" if checked else "Start")
        self.motor_toggled.emit(checked)

    def set_connected(self, ok: bool):
        self.lblConn.setText("Connected" if ok else "Disconnected")
        self.lblConn.setStyleSheet("color:#22c55e;" if ok else "color:#ef4444;")
        self.ledConn.set_status(ok)

    def add_sample(self, s: Sample):
        self.buffer["t"].append(s.t)
        self.buffer["pressure"].append(s.pressure)
        self.buffer["flow"].append(s.flow)
        self.buffer["temp"].append(s.temp)
        self.buffer["motor_on"].append(int(s.motor_on))

        # картки
        self.cardP.set_value(f"{s.pressure}")
        self.cardF.set_value(f"{s.flow:.2f}")
        self.cardT.set_value(f"{s.temp:.2f}")

        # графіки
        self.curP.setData(list(self.buffer["t"]), list(self.buffer["pressure"]))
        self.curF.setData(list(self.buffer["t"]), list(self.buffer["flow"]))
        self.curT.setData(list(self.buffer["t"]), list(self.buffer["temp"]))

        # таблиця
        row = self.table.rowCount()
        self.table.insertRow(row)
        self.table.setItem(row,0, QtWidgets.QTableWidgetItem(str(s.t)))
        self.table.setItem(row,1, QtWidgets.QTableWidgetItem(str(s.pressure)))
        self.table.setItem(row,2, QtWidgets.QTableWidgetItem(f"{s.flow:.2f}"))
        self.table.setItem(row,3, QtWidgets.QTableWidgetItem(f"{s.temp:.2f}"))
        self.table.setItem(row,4, QtWidgets.QTableWidgetItem("ON" if s.motor_on else "OFF"))

    def toggle_theme(self):
        palette = QtGui.QPalette()
        dark = self.palette().color(QtGui.QPalette.Window).value() < 128
        if dark:
            palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#fff"))
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#000"))
        else:
            palette.setColor(QtGui.QPalette.Window, QtGui.QColor("#121212"))
            palette.setColor(QtGui.QPalette.WindowText, QtGui.QColor("#fff"))
        self.setPalette(palette)

    def export_table(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save CSV", "", "CSV Files (*.csv)")
        if path:
            rows = self.table.rowCount()
            cols = self.table.columnCount()
            data = []
            for r in range(rows):
                row_data = []
                for c in range(cols):
                    item = self.table.item(r,c)
                    row_data.append(item.text() if item else "")
                data.append(row_data)
            df = pd.DataFrame(data, columns=["t","Pressure","Flow","Temp","Motor"])
            df.to_csv(path, index=False)
            QtWidgets.QMessageBox.information(self, "Saved", f"Table saved to {path}")

# ===================== Main Window =====================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ESP32 Dashboard")
        self.resize(900, 850)
        self.dashboard = Dashboard()
        self.setCentralWidget(self.dashboard)

        # MQTT
        self.source = MQTTSource()
        self.source.new_sample.connect(self.dashboard.add_sample)
        self.source.connected_changed.connect(self.dashboard.set_connected)
        self.dashboard.motor_toggled.connect(self.source.set_motor)
        self.source.start()

# ===================== Run App =====================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
