import sys, random, time, csv
from collections import deque
from dataclasses import dataclass
from PyQt5 import QtWidgets, QtCore, QtGui
import pyqtgraph as pg
import pandas as pd


@dataclass
class Sample:
    t: float       
    pressure: int  
    flow: float    
    temp: float    
    motor_on: bool


class DemoSource(QtCore.QObject):
    new_sample = QtCore.pyqtSignal(Sample)

    def __init__(self, interval_ms=500, parent=None):
        super().__init__(parent)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.setInterval(interval_ms)
        self._t0 = time.time()
        self._motor_on = False
        
        self._pressure = 115
        self._flow = 3.0
        self._temp = 27.0

    def start(self): self.timer.start()
    def stop(self): self.timer.stop()
    def set_interval(self, ms): self.timer.setInterval(ms)
    def set_motor(self, on: bool): self._motor_on = on

    def _tick(self):
        
        self._pressure += random.uniform(-1.2, 1.2)
        self._pressure = max(85, min(160, self._pressure))

        
        flow_drift = 0.15 if self._motor_on else -0.05
        self._flow += random.uniform(-0.12, 0.12) + flow_drift
        self._flow = max(0.3, min(6.0, self._flow))

        
        self._temp += random.uniform(-0.05, 0.05) + (0.02 if self._motor_on else -0.005)
        self._temp = max(20.0, min(42.0, self._temp))

        t = time.time() - self._t0
        s = Sample(
            t=float(f"{t:.2f}"),
            pressure=int(self._pressure),
            flow=float(f"{self._flow:.2f}"),
            temp=float(f"{self._temp:.2f}"),
            motor_on=self._motor_on
        )
        self.new_sample.emit(s)


class StatCard(QtWidgets.QFrame):
    def __init__(self, title, unit):
        super().__init__()
        self.setFrameShape(QtWidgets.QFrame.NoFrame)
        self.setObjectName("card")
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 12, 16, 12)
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

    def set_value(self, text): self.value.setText(text)

class Led(QtWidgets.QLabel):
    def __init__(self, diameter=14):
        super().__init__()
        self._diam = diameter
        self._color = QtGui.QColor("#aaaaaa")
        self.setFixedSize(diameter, diameter)
        self.set_status(False)

    def set_status(self, on: bool):
        self._color = QtGui.QColor("#22c55e" if on else "#9ca3af")  # green / gray
        pix = QtGui.QPixmap(self._diam, self._diam)
        pix.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(pix)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        brush = QtGui.QBrush(self._color)
        painter.setBrush(brush)
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawEllipse(0, 0, self._diam, self._diam)
        painter.end()
        self.setPixmap(pix)


class Dashboard(QtWidgets.QWidget):
    export_csv = QtCore.pyqtSignal(pd.DataFrame)
    motor_toggled = QtCore.pyqtSignal(bool)

    def __init__(self):
        super().__init__()
        self.max_points = 300
        self.buffer = {
            "t": deque(maxlen=self.max_points),
            "pressure": deque(maxlen=self.max_points),
            "flow": deque(maxlen=self.max_points),
            "temp": deque(maxlen=self.max_points),
            "motor_on": deque(maxlen=self.max_points)
        }

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(16, 16, 16, 16)
        root.setSpacing(12)

        
        statRow = QtWidgets.QHBoxLayout()
        statRow.setSpacing(12)
        self.cardP = StatCard("Pressure", "Pa")
        self.cardF = StatCard("Flow", "ml/min")
        self.cardT = StatCard("Temperature", "°C")

        statContainer = QtWidgets.QWidget()
        statContainer.setLayout(statRow)
        statRow.addWidget(self.cardP)
        statRow.addWidget(self.cardF)
        statRow.addWidget(self.cardT)
        root.addWidget(statContainer)

        
        motorBar = QtWidgets.QHBoxLayout()
        motorBox = QtWidgets.QGroupBox("Motor")
        motorBox.setLayout(motorBar)
        self.led = Led()
        motorLabel = QtWidgets.QLabel("Status:")
        self.btnMotor = QtWidgets.QPushButton("Start")
        self.btnMotor.setCheckable(True)
        self.btnMotor.clicked.connect(self._toggle_motor)
        motorBar.addWidget(motorLabel)
        motorBar.addWidget(self.led)
        motorBar.addStretch(1)
        motorBar.addWidget(self.btnMotor)
        root.addWidget(motorBox)

        
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
        btns.addStretch(1)
        bottomBar.addLayout(btns)

        bottomBox = QtWidgets.QGroupBox("Log")
        bottomBox.setLayout(bottomBar)
        root.addWidget(bottomBox)

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

    def _toggle_motor(self, checked: bool):
        self.led.set_status(checked)
        self.btnMotor.setText("Stop" if checked else "Start")
        self.motor_toggled.emit(checked)

    def _export(self):
        
        df = pd.DataFrame({
            "t_s": list(self.buffer["t"]),
            "pressure_Pa": list(self.buffer["pressure"]),
            "flow_ml_min": list(self.buffer["flow"]),
            "temp_C": list(self.buffer["temp"]),
            "motor_on": list(self.buffer["motor_on"]),
        })
        self.export_csv.emit(df)

   
    def add_sample(self, s: Sample):
        # Update buffers
        self.buffer["t"].append(s.t)
        self.buffer["pressure"].append(s.pressure)
        self.buffer["flow"].append(s.flow)
        self.buffer["temp"].append(s.temp)
        self.buffer["motor_on"].append(int(s.motor_on))

       
        self.cardP.set_value(f"{s.pressure}")
        self.cardF.set_value(f"{s.flow:.2f}")
        self.cardT.set_value(f"{s.temp:.2f}")

        
        x = list(range(len(self.buffer["t"])))
        self.curP.setData(x, list(self.buffer["pressure"]))
        self.curF.setData(x, list(self.buffer["flow"]))
        self.curT.setData(x, list(self.buffer["temp"]))

        
        for plot, key, pad in (
            (self.plotP, "pressure", 10),
            (self.plotF, "flow", 0.5),
            (self.plotT, "temp", 1.0),
        ):
            values = list(self.buffer[key])
            if values:
                vmin, vmax = min(values), max(values)
                if vmin == vmax:
                    vmin -= pad
                    vmax += pad
                else:
                    vmin -= pad
                    vmax += pad
                plot.setYRange(vmin, vmax, padding=0)

        
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c, val in enumerate([s.t, s.pressure, s.flow, s.temp, "ON" if s.motor_on else "OFF"]):
            item = QtWidgets.QTableWidgetItem(str(val))
            self.table.setItem(r, c, item)
        
        if self.table.rowCount() > self.max_points:
            self.table.removeRow(0)

class SettingsView(QtWidgets.QWidget):
    apply_settings = QtCore.pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(12)

        
        srcGroup = QtWidgets.QGroupBox("Data source")
        srcLay = QtWidgets.QFormLayout()
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(["Demo (offline)", "Serial (ESP32)"])
        self.sampling = QtWidgets.QSpinBox()
        self.sampling.setRange(50, 5000)
        self.sampling.setSingleStep(50)
        self.sampling.setValue(500)

        
        self.port = QtWidgets.QLineEdit()
        self.port.setPlaceholderText("COM3  or  /dev/ttyUSB0")
        self.baud = QtWidgets.QComboBox()
        self.baud.addItems(["115200", "57600", "38400", "9600"])

        srcLay.addRow("Mode:", self.mode)
        srcLay.addRow("Sampling, ms:", self.sampling)
        srcLay.addRow("Serial port:", self.port)
        srcLay.addRow("Baud:", self.baud)
        srcGroup.setLayout(srcLay)

        
        thrGroup = QtWidgets.QGroupBox("Thresholds / Alerts")
        thrLay = QtWidgets.QFormLayout()
        self.p_hi = QtWidgets.QSpinBox(); self.p_hi.setRange(50, 500); self.p_hi.setValue(150)
        self.f_min = QtWidgets.QDoubleSpinBox(); self.f_min.setRange(0.0, 20.0); self.f_min.setDecimals(2); self.f_min.setValue(0.8)
        self.t_hi = QtWidgets.QDoubleSpinBox(); self.t_hi.setRange(0.0, 100.0); self.t_hi.setDecimals(1); self.t_hi.setValue(40.0)
        thrLay.addRow("Pressure high [Pa]:", self.p_hi)
        thrLay.addRow("Flow min [ml/min]:", self.f_min)
        thrLay.addRow("Temp high [°C]:", self.t_hi)
        thrGroup.setLayout(thrLay)

        btnApply = QtWidgets.QPushButton("Apply")
        btnApply.clicked.connect(self._emit_apply)

        layout.addWidget(srcGroup)
        layout.addWidget(thrGroup)
        layout.addStretch(1)
        layout.addWidget(btnApply)

        self.setStyleSheet("""
            QGroupBox { border: 1px solid #e5e7eb; border-radius: 12px; margin-top: 14px; }
            QGroupBox::title { subcontrol-origin: margin; left: 12px; padding: 0 4px; color: #334155; }
            QPushButton { padding: 8px 14px; border-radius: 10px; }
        """)

    def _emit_apply(self):
        cfg = dict(
            mode="demo" if self.mode.currentIndex()==0 else "serial",
            sampling_ms=int(self.sampling.value()),
            port=self.port.text().strip(),
            baud=int(self.baud.currentText()),
            p_hi=int(self.p_hi.value()),
            f_min=float(self.f_min.value()),
            t_hi=float(self.t_hi.value()),
        )
        self.apply_settings.emit(cfg)


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Lab-on-a-Chip Analyzer")
        self.resize(980, 820)
        QtWidgets.QApplication.setStyle("Fusion")

        
        self.tabs = QtWidgets.QTabWidget()
        self.dashboard = Dashboard()
        self.settings = SettingsView()
        self.tabs.addTab(self.dashboard, "Dashboard")
        self.tabs.addTab(self.settings, "Settings")
        self.setCentralWidget(self.tabs)

        
        self.status = self.statusBar()
        self.status.showMessage("Demo mode · Not connected")

        # Data source
        self.source = DemoSource(interval_ms=500, parent=self)
        self.source.new_sample.connect(self._on_sample)
        self.dashboard.motor_toggled.connect(self.source.set_motor)
        self.dashboard.export_csv.connect(self._save_csv)
        self.settings.apply_settings.connect(self._apply_cfg)
        self.source.start()

    
    @QtCore.pyqtSlot(Sample)
    def _on_sample(self, s: Sample):
        
        msg = []
        
        p_hi = self.settings.p_hi.value()
        f_min = self.settings.f_min.value()
        t_hi = self.settings.t_hi.value()
        if s.pressure > p_hi: msg.append(f"High pressure: {s.pressure} Pa")
        if s.flow < f_min:    msg.append(f"Low flow: {s.flow:.2f} ml/min")
        if s.temp > t_hi:     msg.append(f"High temp: {s.temp:.1f} °C")
        if msg:
            self.status.showMessage(" · ".join(msg))
        else:
            self.status.showMessage("OK")

        self.dashboard.add_sample(s)

    def _apply_cfg(self, cfg: dict):
        
        self.source.set_interval(cfg["sampling_ms"])
        self.status.showMessage(
            f"{'Demo' if cfg['mode']=='demo' else 'Serial'} mode · interval {cfg['sampling_ms']} ms"
        )

    def _save_csv(self, df: pd.DataFrame):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(
            self, "Save CSV", "log.csv", "CSV files (*.csv)"
        )
        if path:
            df.to_csv(path, index=False)
            self.status.showMessage(f"Saved: {path}")


def main():
    app = QtWidgets.QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
