# -*- coding: utf-8 -*-
"""
Lab-on-a-Chip Analyzer (PyQt5 + pyqtgraph)
- Demo & Serial (ESP32)
- Realtime charts (Pressure/Flow/Temperature)
- Motor ON/OFF (M1/M0)
- Threshold alerts
- Log table + CSV export
- Status/connection indicators
- Safe serial handling (Windows-friendly)
"""
import sys, time, random, threading
from collections import deque
from dataclasses import dataclass

from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pandas as pd

# ---- serial (optional if Demo) ----
try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception:
    serial = None
    list_ports = None


# ===================== Data Model =====================
@dataclass
class Sample:
    """Модель одного зразка даних"""
    t: float        # час у секундах
    pressure: int   # тиск [Pa]
    flow: float     # потік [ml/min]
    temp: float     # температура [°C]
    motor_on: bool  # стан мотора


# ===================== Helpers =====================
def windows_ports_with_com8_first():
    """
    Повертає список портів, ставлячи COM8 першим (якщо є).
    Якщо pyserial недоступний — повертаємо ['COM8'].
    """
    if list_ports is None:
        return ["COM8"]
    found = [p.device for p in list_ports.comports()]
    if not found:
        return ["COM8"]
    if "COM8" in found:
        return ["COM8"] + [p for p in found if p != "COM8"]
    # легкий пріоритет CP210/CH340
    priority = []
    tail = []
    for dev in found:
        try:
            info = next((x for x in list_ports.comports() if x.device == dev), None)
            desc = (info.description if info else "") or ""
        except Exception:
            desc = ""
        if "CP210" in desc or "CH340" in desc or "USB-SERIAL" in desc.upper():
            priority.append(dev)
        else:
            tail.append(dev)
    return priority + tail


# ===================== Data Sources =====================
class DemoSource(QtCore.QObject):
    """
    Джерело демонстраційних даних (імітація роботи сенсорів)
    """
    new_sample = QtCore.pyqtSignal(Sample)
    connected_changed = QtCore.pyqtSignal(bool)

    def __init__(self, interval_ms=500, parent=None):
        super().__init__(parent)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.setInterval(interval_ms)
        self._t0 = time.time()
        self._motor_on = False

        # початкові значення сенсорів
        self._pressure = 115.0
        self._flow = 3.0
        self._temp = 27.0

    def start(self):
        self.timer.start()
        self.connected_changed.emit(True)

    def stop(self):
        self.timer.stop()
        self.connected_changed.emit(False)

    def set_interval(self, ms):
        """Змінити інтервал оновлення для демонстрації"""
        self.timer.setInterval(ms)

    def set_motor(self, on: bool):
        """Увімкнення/вимкнення мотору"""
        self._motor_on = on

    def _tick(self):
        """Генерує новий зразок даних"""
        self._pressure += random.uniform(-1.2, 1.2)
        self._pressure = max(85.0, min(160.0, self._pressure))

        drift = 0.15 if self._motor_on else -0.05
        self._flow += random.uniform(-0.12, 0.12) + drift
        self._flow = max(0.3, min(6.0, self._flow))

        self._temp += random.uniform(-0.05, 0.05) + (0.02 if self._motor_on else -0.005)
        self._temp = max(20.0, min(42.0, self._temp))

        t = time.time() - self._t0
        s = Sample(
            t=float(f"{t:.2f}"),
            pressure=int(round(self._pressure)),
            flow=float(f"{self._flow:.2f}"),
            temp=float(f"{self._temp:.2f}"),
            motor_on=self._motor_on
        )
        self.new_sample.emit(s)


class SerialSource(QtCore.QObject):
    """
    Джерело даних через серійний порт (ESP32)
    """
    new_sample = QtCore.pyqtSignal(Sample)
    connected_changed = QtCore.pyqtSignal(bool)

    def __init__(self, port: str = "COM8", baud: int = 115200, parent=None):
        super().__init__(parent)
        self._port = (port or "COM8").strip()
        self._baud = int(baud) if baud else 115200
        self._ser = None
        self._running = False
        self._thread = None
        self._last_motor = False

    def start(self):
        if serial is None:
            print("[WARN] pyserial not installed; cannot use Serial mode.")
            self.connected_changed.emit(False)
            return
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=1)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._ser.write_timeout = 0.2
            self._ser.timeout = 1
            time.sleep(0.2)
            self._running = True
            self._thread = threading.Thread(target=self._reader, daemon=True)
            self._thread.start()
            self.connected_changed.emit(True)
        except Exception as e:
            print("[ERROR] Serial open error:", e)
            self.connected_changed.emit(False)

    def stop(self):
        self._running = False
        try:
            if self._ser and self._ser.is_open:
                self._ser.close()
        except Exception:
            pass
        self.connected_changed.emit(False)

    def set_motor(self, on: bool):
        """Управління мотором через ESP32"""
        self._last_motor = on
        try:
            if self._ser and self._ser.is_open:
                self._ser.write(b"M1\n" if on else b"M0\n")
        except Exception:
            pass

    def _reader(self):
        """Читання та парсинг рядків серійного порту"""
        while self._running and self._ser and self._ser.is_open:
            try:
                raw = self._ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="ignore").strip()
                if not line:
                    continue

                # Основний CSV-формат
                if (line[0].isdigit() or line[0] == '.') and line.count(",") == 4:
                    parts = line.split(",")
                    t = float(parts[0])
                    pressure = int(float(parts[1]))
                    flow = float(parts[2])
                    temp = float(parts[3])
                    motor = bool(int(float(parts[4])))
                    self.new_sample.emit(Sample(t, pressure, flow, temp, motor))
                    continue

                # Формат ключ=значення
                if '=' in line and line.count(",") >= 4:
                    kv = {}
                    for token in line.split(","):
                        token = token.strip()
                        if "=" in token:
                            k, v = token.split("=", 1)
                            kv[k.strip().lower()] = v.strip()
                    keys = ("t", "p", "f", "temp", "m")
                    if all(k in kv for k in keys):
                        t = float(kv["t"])
                        pressure = int(float(kv["p"]))
                        flow = float(kv["f"])
                        temp = float(kv["temp"])
                        motor = bool(int(float(kv["m"])))
                        self.new_sample.emit(Sample(t, pressure, flow, temp, motor))
            except Exception:
                pass


# ===================== UI Widgets =====================
class StatCard(QtWidgets.QFrame):
    """Картка зі статистикою (Pressure/Flow/Temp)"""
    def __init__(self, title, unit):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setObjectName("card")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(6)

        self.title = QtWidgets.QLabel(title)
        self.title.setObjectName("cardTitle")
        self.value = QtWidgets.QLabel("--")
        self.value.setObjectName("cardValue")
        self.unit = QtWidgets.QLabel(unit)
        self.unit.setObjectName("cardUnit")

        layout.addWidget(self.title)
        layout.addWidget(self.value)
        layout.addWidget(self.unit)
        layout.addStretch(1)

    def set_value(self, text):
        """Оновлює значення картки"""
        self.value.setText(text)


class Led(QtWidgets.QLabel):
    """Маленький круглий індикатор"""
    def __init__(self, diameter=14):
        super().__init__()
        self._diam = diameter
        self.setFixedSize(diameter, diameter)
        self.set_status(False)

    def set_status(self, on: bool, color_on="#22c55e", color_off="#ef4444"):
        """Установити стан LED"""
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


class Dashboard(QtWidgets.QWidget):
    """
    Основна панель Dashboard з графіками, картками та логами
    """
    export_csv = QtCore.pyqtSignal(pd.DataFrame)
    motor_toggled = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.max_points = 300  # максимум точок на графіку
        self.buffer = {
            "t": deque(maxlen=self.max_points),
            "pressure": deque(maxlen=self.max_points),
            "flow": deque(maxlen=self.max_points),
            "temp": deque(maxlen=self.max_points),
            "motor_on": deque(maxlen=self.max_points),
        }

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        # ---- картки статистики ----
        statRow = QtWidgets.QHBoxLayout()
        statRow.setSpacing(12)
        self.cardP = StatCard("Pressure", "Pa")
        self.cardF = StatCard("Flow", "ml/min")
        self.cardT = StatCard("Temperature", "°C")
        statRow.addWidget(self.cardP)
        statRow.addWidget(self.cardF)
        statRow.addWidget(self.cardT)
        statWrap = QtWidgets.QWidget()
        statWrap.setLayout(statRow)
        root.addWidget(statWrap)

        # ---- мотор та індикатор підключення ----
        motorBar = QtWidgets.QHBoxLayout()
        motorBar.setSpacing(8)
        motorBox = QtWidgets.QGroupBox("Motor & Connection")
        motorBox.setLayout(motorBar)

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
        root.addWidget(motorBox)

        # ---- графіки ----
        charts = QtWidgets.QGridLayout()
        charts.setHorizontalSpacing(12)
        charts.setVerticalSpacing(12)

        pg.setConfigOptions(antialias=True)
        self.plotP = pg.PlotWidget(title="Pressure [Pa]")
        self.plotF = pg.PlotWidget(title="Flow [ml/min]")
        self.plotT = pg.PlotWidget(title="Temperature [°C]")

        for w in (self.plotP, self.plotF, self.plotT):
            w.showGrid(x=True, y=True, alpha=0.3)
            w.setBackground("w")
            w.getAxis("left").setTextPen("black")
            w.getAxis("bottom").setTextPen("black")
            w.getAxis("left").setPen(pg.mkPen("#e5e7eb"))
            w.getAxis("bottom").setPen(pg.mkPen("#e5e7eb"))

        self.curP = self.plotP.plot(pen=pg.mkPen("#ef4444", width=2))
        self.curF = self.plotF.plot(pen=pg.mkPen("#10b981", width=2))
        self.curT = self.plotT.plot(pen=pg.mkPen("#3b82f6", width=2))

        charts.addWidget(self.plotP, 0, 0)
        charts.addWidget(self.plotF, 1, 0)
        charts.addWidget(self.plotT, 2, 0)

        chartsBox = QtWidgets.QGroupBox("Sensor Data")
        chartsBox.setLayout(charts)
        root.addWidget(chartsBox, 1)

        # ---- лог таблиця + експорт ----
        bottomBar = QtWidgets.QHBoxLayout()
        self.table = QtWidgets.QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["t [s]", "Pressure", "Flow", "Temp", "Motor"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        bottomBar.addWidget(self.table)

        btns = QtWidgets.QVBoxLayout()
        self.btnExport = QtWidgets.QPushButton("Export CSV")
        self.btnExport.clicked.connect(self._export)
        btns.addWidget(self.btnExport)
        self.btnClear = QtWidgets.QPushButton("Clear Log")
        self.btnClear.clicked.connect(lambda: self.table.setRowCount(0))
        btns.addWidget(self.btnClear)
        btns.addStretch(1)
        bottomBar.addLayout(btns)

        bottomBox = QtWidgets.QGroupBox("Log")
        bottomBox.setLayout(bottomBar)
        root.addWidget(bottomBox)

        # ---- стиль ----
        self.setStyleSheet("""
            QWidget#card { background: #f8fafc; border: 1px solid #e5e7eb; border-radius: 16px; }
            QLabel#cardTitle { color: #64748b; font-size: 12px; }
            QLabel#cardValue { font-size: 28px; font-weight: 600; }
            QLabel#cardUnit { color: #94a3b8; font-size: 12px; }
            QGroupBox { border: 1px solid #e5e7eb; border-radius: 12px; margin-top: 14px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #334155; }
            QPushButton { padding: 8px 14px; border-radius: 10px; }
            QPushButton:checked { background: #22c55e; color: white; }
            QTableWidget::item { padding: 6px; }
        """)

    # ---- публічні методи ----
    def set_connected(self, ok: bool):
        """Оновити стан підключення"""
        self.lblConn.setText("Connected" if ok else "Disconnected")
        self.lblConn.setStyleSheet("color:#22c55e;" if ok else "color:#ef4444;")
        self.ledConn.set_status(ok)

    # ---- внутрішні ----
    def _toggle_motor(self, checked: bool):
        """Обробка кнопки Start/Stop мотору"""
        self.ledMotor.set_status(checked, color_on="#22c55e", color_off="#9ca3af")
        self.btnMotor.setText("Stop" if checked else "Start")
        self.motor_toggled.emit(checked)

    def _export(self):
        """Експорт даних у CSV"""
        df = pd.DataFrame({
            "t_s": list(self.buffer["t"]),
            "pressure_Pa": list(self.buffer["pressure"]),
            "flow_ml_min": list(self.buffer["flow"]),
            "temp_C": list(self.buffer["temp"]),
            "motor_on": list(self.buffer["motor_on"]),
        })
        self.export_csv.emit(df)

    def add_sample(self, s: Sample):
        """Додає новий зразок у буфер, графік, таблицю та картки"""
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
        x = list(range(len(self.buffer["t"])))
        self.curP.setData(x, list(self.buffer["pressure"]))
        self.curF.setData(x, list(self.buffer["flow"]))
        self.curT.setData(x, list(self.buffer["temp"]))

        # динамічний Y-діапазон
        for plot, key, pad in (
            (self.plotP, "pressure", 10),
            (self.plotF, "flow", 0.5),
            (self.plotT, "temp", 1.0),
        ):
            vals = self.buffer[key]
            if vals:
                vmin, vmax = min(vals), max(vals)
                if vmin == vmax:
                    vmin -= pad
                    vmax += pad
                else:
                    vmin -= pad
                    vmax += pad
                plot.setYRange(vmin, vmax, padding=0)

        # таблиця
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c, val in enumerate([s.t, s.pressure, s.flow, s.temp, "ON" if s.motor_on else "OFF"]):
            item = QtWidgets.QTableWidgetItem(str(val))
            self.table.setItem(r, c, item)
        if self.table.rowCount() > self.max_points:
            self.table.removeRow(0)


# ===================== Settings =====================
class SettingsView(QtWidgets.QWidget):
    """Панель налаштувань джерела, інтервалу та порогів"""
    apply_settings = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        # ---- джерело даних ----
        srcGroup = QtWidgets.QGroupBox("Data source")
        srcLay = QtWidgets.QFormLayout()
        srcGroup.setLayout(srcLay)

        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["Demo (offline)", "Serial (ESP32)"])
        self.mode.setCurrentIndex(1)  # за замовчуванням Serial

        self.sampling = QtWidgets.QSpinBox()
        self.sampling.setRange(50, 5000)
        self.sampling.setSingleStep(50)
        self.sampling.setValue(500)

        # Порти
        portRow = QtWidgets.QHBoxLayout()
        self.port = QtWidgets.QComboBox()
        self.btnRefresh = QtWidgets.QPushButton("Refresh")
        self.btnRefresh.clicked.connect(self._refresh_ports)
        portRow.addWidget(self.port)
        portRow.addWidget(self.btnRefresh)
        self._refresh_ports()

        srcLay.addRow("Mode:", self.mode)
        srcLay.addRow("Interval [ms]:", self.sampling)
        srcLay.addRow("Serial port:", portRow)
        layout.addWidget(srcGroup)

        # ---- пороги ----
        thresholdGroup = QtWidgets.QGroupBox("Thresholds / Alerts")
        thresholdLay = QtWidgets.QFormLayout()
        thresholdGroup.setLayout(thresholdLay)
        self.thP = QtWidgets.QSpinBox()
        self.thP.setRange(0, 500)
        self.thP.setValue(150)
        self.thF = QtWidgets.QDoubleSpinBox()
        self.thF.setRange(0, 10.0)
        self.thF.setSingleStep(0.1)
        self.thF.setValue(5.5)
        self.thT = QtWidgets.QDoubleSpinBox()
        self.thT.setRange(0, 100.0)
        self.thT.setValue(40.0)
        thresholdLay.addRow("Pressure max [Pa]:", self.thP)
        thresholdLay.addRow("Flow max [ml/min]:", self.thF)
        thresholdLay.addRow("Temp max [°C]:", self.thT)
        layout.addWidget(thresholdGroup)

        # ---- кнопка застосування ----
        self.btnApply = QtWidgets.QPushButton("Apply")
        self.btnApply.clicked.connect(self._apply)
        layout.addWidget(self.btnApply, alignment=QtCore.Qt.AlignRight)

        layout.addStretch(1)

    def _refresh_ports(self):
        """Оновити список доступних COM портів"""
        ports = windows_ports_with_com8_first()
        self.port.clear()
        self.port.addItems(ports)

    def _apply(self):
        """Відправляє словник налаштувань"""
        cfg = {
            "mode": self.mode.currentText(),
            "interval": self.sampling.value(),
            "port": self.port.currentText(),
            "threshold_pressure": self.thP.value(),
            "threshold_flow": self.thF.value(),
            "threshold_temp": self.thT.value(),
        }
        self.apply_settings.emit(cfg)


# ===================== Main Window =====================
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lab-on-a-Chip Analyzer")
        self.resize(1100, 780)

        self.dashboard = Dashboard()
        self.settings = SettingsView()

        self._source = None  # джерело даних

        # Tab widget
        tabs = QtWidgets.QTabWidget()
        tabs.addTab(self.dashboard, "Dashboard")
        tabs.addTab(self.settings, "Settings")
        self.setCentralWidget(tabs)

        # Signal connections
        self.settings.apply_settings.connect(self._apply_settings)
        self.dashboard.motor_toggled.connect(self._motor_toggle)
        self.dashboard.export_csv.connect(self._do_export)

    def _apply_settings(self, cfg: dict):
        """Створення нового джерела або зміна інтервалу"""
        # видалити старе джерело
        if self._source:
            self._source.stop()
            self._source.new_sample.disconnect(self.dashboard.add_sample)
            self._source.connected_changed.disconnect(self.dashboard.set_connected)

        mode = cfg.get("mode", "Demo (offline)")
        if mode.startswith("Demo"):
            self._source = DemoSource(interval_ms=cfg.get("interval", 500))
        else:
            self._source = SerialSource(port=cfg.get("port", "COM8"))
        self._source.new_sample.connect(self.dashboard.add_sample)
        self._source.connected_changed.connect(self.dashboard.set_connected)
        self._source.start()

    def _motor_toggle(self, on: bool):
        """Керування мотором джерела"""
        if self._source:
            self._source.set_motor(on)

    def _do_export(self, df: pd.DataFrame):
        """Експорт CSV через стандартний діалог"""
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save CSV", "", "CSV Files (*.csv)"
        )
        if path:
            df.to_csv(path, index=False)
            QtWidgets.QMessageBox.information(self, "Export", f"Saved to {path}")


# ===================== Entry =====================
def main():
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
