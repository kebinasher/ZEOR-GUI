
import sys
import time
import csv
import math
import json  # NEW (used by the Live Map tab to pass data safely to JavaScript)
import os
from collections import deque

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QComboBox, QTextEdit, QGroupBox, QGridLayout, QMessageBox,
    QSizePolicy, QDialog, QStackedLayout,
    QTabWidget, QCheckBox, QFileDialog,  # NEW
)
from PyQt6.QtCore import (
    QThread, pyqtSignal, Qt, QRectF, QPointF,
    QTimer,  # NEW (was previously imported locally inside __init__)
)
from PyQt6.QtGui import (
    QPainter, QColor, QPen, QBrush, QFont, QPolygonF, QPainterPath,
    QLinearGradient,  # NEW
)

import pyqtgraph as pg
import serial
import serial.tools.list_ports

try:
    import folium
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    MAP_SUPPORT_AVAILABLE = True
except ImportError:
    folium = None
    QWebEngineView = None
    MAP_SUPPORT_AVAILABLE = False


try:
    import numpy as np
    import trimesh
    import pyqtgraph.opengl as gl
    from PyQt6.QtGui import QVector3D
    STL_3D_AVAILABLE = True
except ImportError:
    np = None
    trimesh = None
    gl = None
    QVector3D = None
    STL_3D_AVAILABLE = False

VERSION = "Zenith Orbitron Ground Station v6.0 — Teensy 4.1 / LoRa E22 / BNO085"

# ============================================================
# CONFIGURATION
# ============================================================

# E22-900T22D factory UART default is 9600 8N1. Must match the
# ground-side module's UART setting (NOT the LoRa air rate).
BAUD_CHOICES = ["9600", "19200", "38400", "57600", "115200"]
DEFAULT_BAUD = "9600"

EXPECTED_FIELDS = 20
GPS_FIELDS = 22

LOW_VOLTAGE_WARN = 6.6      # volts, change for your battery pack
LINK_LOST_TIMEOUT_S = 5.0   # "NO SIGNAL" if no packet for this long
MAP_DEFAULT_LAT = 28.6139   # launch-site default
MAP_DEFAULT_LON = 77.2090
MAP_REFRESH_S = 2.0         # folium re-render throttle

FIELD_KEYS = [
    "time_ms",
    "bmp_alt", "bmp_temp", "bmp_press",
    "dps_alt", "dps_temp", "dps_press",
    "ax", "ay", "az",
    "gx", "gy", "gz",
    "roll", "pitch", "yaw",
    "vbus", "cur", "pwr",
]

# id, title, y label, units, [(key, color, legend name)]
PLOT_SPECS = [
    ("alt", "Altitude", "Altitude", "m",
     [("bmp_alt", "#00FFCC", "BMP581"), ("dps_alt", "#FF8A00", "DPS310")]),
    ("temp", "Temperature", "Temperature", "°C",
     [("bmp_temp", "#00FFCC", "BMP581"), ("dps_temp", "#FF8A00", "DPS310")]),
    ("press", "Pressure", "Pressure", "hPa",
     [("bmp_press", "#00FFCC", "BMP581"), ("dps_press", "#FF8A00", "DPS310")]),
    ("accel", "BNO085 Acceleration", "Acceleration", "m/s²",
     [("ax", "#FF5252", "Ax"), ("ay", "#69F0AE", "Ay"), ("az", "#40C4FF", "Az")]),
    ("gyro", "BNO085 Angular Rate", "Angular Rate", "°/s",
     [("gx", "#FF5252", "Gx"), ("gy", "#69F0AE", "Gy"), ("gz", "#40C4FF", "Gz")]),
    ("euler", "BNO085 Orientation", "Angle", "°",
     [("roll", "#FF4081", "Roll"), ("pitch", "#00E5FF", "Pitch"),
      ("yaw", "#FFD400", "Yaw")]),
    ("vbus", "INA219 Bus Voltage", "Voltage", "V",
     [("vbus", "#B388FF", "Vbus")]),
    ("cur", "INA219 Current", "Current", "mA",
     [("cur", "#FFD400", "Current")]),
]

# ============================================================
# NEW — CONFIGURATION FOR THE TWO NEW TABS
# ============================================================

# ---- Tab 2: Live Map ----
MAP_TAB_DEFAULT_ZOOM = 15      # zoom before the first GPS fix
MAP_TAB_FOLLOW_ZOOM = 17       # zoom applied on the first GPS fix
MAP_TAB_FLUSH_MS = 400         # max rate at which the marker is pushed to the map
MAP_TAB_GPS_STALE_S = 10.0     # "GPS STALE" if no new fix for this long
MAP_TAB_TRAIL_MAX = 5000       # max points kept in the flight-path polyline

# ---- Tab 3: 2D Flight Simulation ----
SIM_ALT_KEY = "bmp_alt"             # telemetry field that drives the simulation
SIM_ALT_FALLBACK_KEY = "dps_alt"    # used only if SIM_ALT_KEY is NaN/inf
# True : altitude shown = sensor altitude minus the pad (ground) reference that
#        is measured from the first packets, so the rocket starts at 0 m even if
#        the barometer reports height above sea level.
# False: the raw sensor value is used as the altitude directly.
SIM_USE_RELATIVE_ALTITUDE = True
SIM_BASELINE_SAMPLES = 10           # packets averaged to find the ground level
SIM_BASELINE_EMA = 0.02             # slow weather-drift tracking while READY
SIM_HISTORY = 5000                  # altitude-graph points kept
SIM_FRAME_MS = 33                   # animation timer (~30 fps)
SIM_SMOOTH_TAU_S = 0.25             # visual smoothing time constant
SIM_VIEW_TAU_S = 0.5                # smoothing of the altitude-scale zoom
SIM_MIN_VIEW_M = 100.0              # smallest altitude range shown on the scale
SIM_SPEED_WINDOW_S = 1.0            # vertical speed is measured over ~this window
SIM_SPEED_MIN_DT_S = 0.15           # never divide by a smaller time step
SIM_LAUNCH_ALT_M = 3.0              # READY -> ASCENDING needs altitude above this ...
SIM_LAUNCH_SPEED_MS = 2.0           # ... and climb rate above this ...
SIM_LAUNCH_CONFIRM = 2              # ... on this many consecutive packets
SIM_DESCENT_SPEED_MS = 1.0          # ASCENDING -> DESCENDING: sink rate above this ...
SIM_DESCENT_CONFIRM = 2             # ... on this many consecutive packets ...
SIM_APOGEE_DROP_M = 1.0             # ... and altitude at least this far below apogee
SIM_LANDED_ALT_M = 2.5              # LANDED: below this altitude ...
SIM_LANDED_SPEED_MS = 1.0           # ... with |vertical speed| below this ...
SIM_LANDED_HOLD_S = 3.0             # ... for this long (flight time)

SIM_STATUS_COLORS = {
    "READY": "#FFD400",
    "ASCENDING": "#00FFCC",
    "DESCENDING": "#FF8A00",
    "LANDED": "#40C4FF",
}


# ============================================================
# ATTITUDE INDICATOR (driven directly by BNO085 roll / pitch)
# ============================================================

class AttitudeIndicator(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.pitch_deg = 0.0
        self.roll_deg = 0.0
        self.yaw_deg = 0.0
        self.setMinimumSize(220, 190)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)

    def set_attitude(self, pitch, roll, yaw=0.0):
        self.pitch_deg = max(min(float(pitch), 90.0), -90.0)
        self.roll_deg = max(min(float(roll), 180.0), -180.0)
        self.yaw_deg = float(yaw)
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        cx, cy = w / 2, h / 2 - 8
        r = min(w, h) * 0.36
        p.fillRect(self.rect(), QColor("#121212"))

        circle = QRectF(cx - r, cy - r, r * 2, r * 2)
        p.setPen(QPen(QColor("#CFCFCF"), 4))
        p.setBrush(QBrush(QColor("#050505")))
        p.drawEllipse(circle)

        # --- moving horizon, clipped to the dial ---
        p.save()
        clip = QPainterPath()
        clip.addEllipse(QRectF(cx - r + 5, cy - r + 5, 2 * r - 10, 2 * r - 10))
        p.setClipPath(clip)
        p.translate(cx, cy + self.pitch_deg * 3.0)
        p.rotate(-self.roll_deg)
        big = r * 4
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#1976D2"))
        p.drawRect(QRectF(-big, -big, big * 2, big))
        p.setBrush(QColor("#8B4A20"))
        p.drawRect(QRectF(-big, 0, big * 2, big))
        p.setPen(QPen(QColor("#FFFFFF"), 3))
        p.drawLine(int(-big), 0, int(big), 0)

        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        p.setPen(QPen(QColor("#FFFFFF"), 2))
        for mark in range(-60, 65, 10):
            if mark == 0:
                continue
            y = -mark * 3.0
            half = 40 if mark > 0 else 26
            p.drawLine(-half, int(y), half, int(y))
            p.drawText(-half - 28, int(y + 4), str(abs(mark)))
            p.drawText(half + 8, int(y + 4), str(abs(mark)))
        p.restore()

        # --- outline, fixed aircraft symbol, roll pointer ---
        p.setPen(QPen(QColor("#D0D0D0"), 4))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(circle)

        p.setPen(QPen(QColor("#FFD400"), 4))
        ix = lambda v: int(v)
        p.drawLine(ix(cx - r * .5), ix(cy), ix(cx - r * .12), ix(cy))
        p.drawLine(ix(cx + r * .12), ix(cy), ix(cx + r * .5), ix(cy))
        p.drawLine(ix(cx - r * .12), ix(cy), ix(cx), ix(cy + r * .12))
        p.drawLine(ix(cx), ix(cy + r * .12), ix(cx + r * .12), ix(cy))

        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#FFFFFF"))
        p.drawPolygon(QPolygonF([
            QPointF(cx, cy - r + 9),
            QPointF(cx - 10, cy - r + 30),
            QPointF(cx + 10, cy - r + 30),
        ]))

        p.save()
        p.translate(cx, cy)
        p.setPen(QPen(QColor("#FFFFFF"), 2))
        for a in (-60, -45, -30, -20, -10, 10, 20, 30, 45, 60):
            p.save()
            p.rotate(a)
            p.drawLine(0, int(-(r - 17)), 0, int(-(r - 6)))
            p.restore()
        p.restore()

        # --- numeric readout ---
        p.setPen(QColor("#FFFFFF"))
        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        ty = int(cy + r + 20)
        p.drawText(6, ty, f"P:{self.pitch_deg:+.1f}°")
        p.drawText(int(w * 0.34), ty, f"R:{self.roll_deg:+.1f}°")
        p.drawText(int(w * 0.67), ty, f"Y:{self.yaw_deg:+.0f}°")
        p.end()


# ============================================================
# MAP OVERLAY + FULLSCREEN DIALOG
# ============================================================

class ClickableMapOverlay(QWidget):
    """Transparent widget above the QWebEngineView to catch clicks."""
    clicked = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        self.setToolTip("Click to view fullscreen")
        self.setStyleSheet("background-color: rgba(0,0,0,0);")

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self.clicked.emit()
        super().mousePressEvent(event)

    def enterEvent(self, event):
        self.setStyleSheet("background-color: rgba(255,255,255,25);")
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.setStyleSheet("background-color: rgba(0,0,0,0);")
        super().leaveEvent(event)


class MapFullscreenDialog(QDialog):
    def __init__(self, html, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Payload Location — Fullscreen")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        bar = QHBoxLayout()
        bar.setContentsMargins(10, 8, 10, 8)
        hint = QLabel("Press ESC or click Close to exit fullscreen")
        hint.setStyleSheet("color: #BDBDBD;")
        btn = QPushButton("Close (ESC)")
        btn.clicked.connect(self.close)
        bar.addWidget(hint)
        bar.addStretch()
        bar.addWidget(btn)
        layout.addLayout(bar)

        self.view = QWebEngineView()
        layout.addWidget(self.view, 1)
        self.view.setHtml(html)
        self.showFullScreen()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key.Key_Escape:
            self.close()
        else:
            super().keyPressEvent(event)


# ============================================================
# NEW — TAB 2 : FOLIUM LIVE MAP
# ============================================================

class LiveMapTab(QWidget):
    """
    Dedicated, self-contained Folium map tab.

    * Fed by GroundStationGUI.handle_telemetry_packet() through
      update_from_packet(pkt) — it never touches the serial port.
    * The Folium page is generated ONCE. Marker / trail updates are pushed to
      the already-loaded Leaflet map with runJavaScript(), so the map does not
      flicker or reload and keeps whatever zoom / pan the user chose.
    * No GPS fix  -> no marker is drawn and no coordinates are invented.
      The map is simply centred on MAP_DEFAULT_LAT / MAP_DEFAULT_LON
      (a viewing position only, not a rocket position).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.lat = None
        self.lon = None
        self.alt = None
        self.fix_count = 0
        self.last_fix_wall = None
        self.page_ready = False
        self.map_name = None
        self.view = None
        self._dirty = False

        root = QVBoxLayout(self)

        # ---- info panel ----
        info = QGroupBox("ROCKET LOCATION (GPS)")
        g = QGridLayout(info)
        self.lbl_gps_status = QLabel("GPS: WAITING FOR DATA")
        self.lbl_gps_status.setStyleSheet(
            "color: #FF8A00; font-size: 12pt; font-weight: bold;")
        self.lbl_lat = QLabel("Latitude: --")
        self.lbl_lon = QLabel("Longitude: --")
        self.lbl_alt = QLabel("Altitude (BMP581): --")
        self.lbl_fixes = QLabel("GPS fixes received: 0")
        self.lbl_last_fix = QLabel("Last fix: --")
        for lbl in (self.lbl_lat, self.lbl_lon, self.lbl_alt):
            lbl.setStyleSheet("color: #00FFCC; font-weight: bold;")
        g.addWidget(self.lbl_gps_status, 0, 0, 1, 3)
        g.addWidget(self.lbl_lat, 1, 0)
        g.addWidget(self.lbl_lon, 1, 1)
        g.addWidget(self.lbl_alt, 1, 2)
        g.addWidget(self.lbl_fixes, 2, 0)
        g.addWidget(self.lbl_last_fix, 2, 1)

        self.lbl_note = QLabel(
            "No GPS coordinates received yet. No marker is shown and no position "
            "is simulated. The marker appears automatically as soon as the "
            "telemetry carries the optional latitude/longitude fields "
            "(22-field packet with a valid, non-zero fix).")
        self.lbl_note.setWordWrap(True)
        self.lbl_note.setStyleSheet("color: #BDBDBD;")
        g.addWidget(self.lbl_note, 3, 0, 1, 3)
        root.addWidget(info)

        # ---- controls ----
        ctl = QHBoxLayout()
        self.chk_follow = QCheckBox("Auto-follow rocket")
        self.chk_follow.setChecked(True)
        self.btn_center = QPushButton("Center on rocket")
        self.btn_center.clicked.connect(self._center_on_rocket)
        self.btn_clear = QPushButton("Clear trail")
        self.btn_clear.clicked.connect(self._clear_trail)
        ctl.addWidget(self.chk_follow)
        ctl.addWidget(self.btn_center)
        ctl.addWidget(self.btn_clear)
        ctl.addStretch()
        root.addLayout(ctl)

        # ---- map ----
        if MAP_SUPPORT_AVAILABLE:
            self.view = QWebEngineView()
            self.view.loadFinished.connect(self._on_loaded)
            root.addWidget(self.view, 1)
            self._load_map()
        else:
            fb = QLabel("Map unavailable — install the map dependencies with:\n\n"
                        "pip install folium PyQt6-WebEngine\n\n"
                        "(GPS values above will still update.)")
            fb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fb.setStyleSheet("color: #FF8A00; font-size: 12pt;")
            root.addWidget(fb, 1)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(MAP_TAB_FLUSH_MS)

    # ---------------- map page ----------------
    def _load_map(self):
        fmap = folium.Map(
            location=[MAP_DEFAULT_LAT, MAP_DEFAULT_LON],
            zoom_start=MAP_TAB_DEFAULT_ZOOM,
            tiles="OpenStreetMap",
            zoom_control=True,       # +/- zoom buttons
            control_scale=True,
        )
        self.map_name = fmap.get_name()   # JS variable name of the Leaflet map
        self.page_ready = False
        self.view.setHtml(fmap.get_root().render())

    def _on_loaded(self, ok):
        self.page_ready = bool(ok)
        if ok and self.lat is not None:
            self._dirty = True

    def _run_js(self, js, callback=None):
        if self.view is None:
            return
        page = self.view.page()
        try:
            if callback is None:
                page.runJavaScript(js)
            else:
                page.runJavaScript(js, callback)
        except TypeError:
            page.runJavaScript(js, 0, callback)

    def _js_update(self):
        lat, lon = self.lat, self.lon
        tip = f"<b>Rocket</b><br>Lat {lat:.6f}<br>Lon {lon:.6f}"
        if self.alt is not None:
            tip += f"<br>Alt {self.alt:.1f} m"
        follow = "true" if self.chk_follow.isChecked() else "false"
        return f"""
(function() {{
  var m = window[{json.dumps(self.map_name)}];
  if (!m || typeof L === 'undefined') {{ return false; }}
  var ll = [{lat:.7f}, {lon:.7f}];
  var tip = {json.dumps(tip)};
  if (!window.__rocketMarker) {{
    var icon = L.divIcon({{
      html: '<div style="font-size:26px;line-height:26px;">🚀</div>',
      className: '', iconSize: [28, 28], iconAnchor: [14, 14]
    }});
    window.__rocketMarker = L.marker(ll, {{icon: icon, title: 'Rocket'}}).addTo(m);
    window.__rocketMarker.bindTooltip(tip, {{permanent: true, direction: 'right', offset: [14, 0]}});
    window.__rocketTrail = L.polyline([ll], {{color: '#FF3333', weight: 3, opacity: 0.8}}).addTo(m);
    m.setView(ll, {MAP_TAB_FOLLOW_ZOOM});
  }} else {{
    window.__rocketMarker.setLatLng(ll);
    window.__rocketMarker.setTooltipContent(tip);
    window.__rocketTrail.addLatLng(ll);
    var pts = window.__rocketTrail.getLatLngs();
    if (pts.length > {MAP_TAB_TRAIL_MAX}) {{
      pts.shift();
      window.__rocketTrail.setLatLngs(pts);
    }}
  }}
  if ({follow}) {{ m.panTo(ll); }}
  return true;
}})();
"""

    def _on_update_result(self, ok):
        if ok is not True:
            # Leaflet not loaded yet (e.g. offline) -> retry on the next tick
            self._dirty = True
            self.lbl_note.setText(
                "Map library has not loaded (internet connection needed for "
                "map tiles / Leaflet). Retrying automatically…")
        else:
            self.lbl_note.setText(
                "Live GPS marker active. Scroll / +/- buttons to zoom, drag to "
                "pan. 'Auto-follow' keeps the rocket centred.")

    def _center_on_rocket(self):
        if self.view is None or not self.page_ready or self.lat is None:
            return
        self._run_js(f"""
(function() {{
  var m = window[{json.dumps(self.map_name)}];
  if (m && window.__rocketMarker) {{
    m.setView(window.__rocketMarker.getLatLng(), Math.max(m.getZoom(), 15));
  }}
}})();""")

    def _clear_trail(self):
        if self.view is None or not self.page_ready:
            return
        self._run_js("""
(function() {
  if (window.__rocketTrail && window.__rocketMarker) {
    window.__rocketTrail.setLatLngs([window.__rocketMarker.getLatLng()]);
  }
})();""")

    # ---------------- telemetry hook ----------------
    def update_from_packet(self, pkt):
        alt = pkt.get("bmp_alt")
        if alt is not None and math.isfinite(alt):
            self.alt = alt
            self.lbl_alt.setText(f"Altitude (BMP581): {alt:.2f} m")

        if "latitude" in pkt and "longitude" in pkt:
            self.lat = pkt["latitude"]
            self.lon = pkt["longitude"]
            self.fix_count += 1
            self.last_fix_wall = time.time()
            self.lbl_lat.setText(f"Latitude: {self.lat:.6f}°")
            self.lbl_lon.setText(f"Longitude: {self.lon:.6f}°")
            self.lbl_fixes.setText(f"GPS fixes received: {self.fix_count}")
            self.lbl_last_fix.setText(
                f"Last fix: {time.strftime('%H:%M:%S')}")
            self._dirty = True

    def _refresh_status(self):
        if self.last_fix_wall is None:
            self.lbl_gps_status.setText("GPS: WAITING FOR DATA")
            self.lbl_gps_status.setStyleSheet(
                "color: #FF8A00; font-size: 12pt; font-weight: bold;")
            return
        age = time.time() - self.last_fix_wall
        if age > MAP_TAB_GPS_STALE_S:
            self.lbl_gps_status.setText(f"GPS: STALE (last fix {age:.0f}s ago)")
            self.lbl_gps_status.setStyleSheet(
                "color: #FF3333; font-size: 12pt; font-weight: bold;")
        else:
            self.lbl_gps_status.setText("GPS: FIX OK")
            self.lbl_gps_status.setStyleSheet(
                "color: #00FFCC; font-size: 12pt; font-weight: bold;")

    def _tick(self):
        self._refresh_status()
        if (self._dirty and self.view is not None and self.page_ready
                and self.lat is not None):
            self._dirty = False
            self._run_js(self._js_update(), self._on_update_result)


# ============================================================
# NEW — TAB 3 : 2D ROCKET FLIGHT SIMULATION
#       Mode 1 "Live Sensor Data"      (driven by received altitude only)
#       Mode 2 "Automatic Simulation"  (internal flight-profile engine)
#
#   Only this section differs from the previous revision. Tabs 1 and 2,
#   the serial thread and the telemetry handler are untouched; this tab is
#   still fed through RocketSimulationTab.update_from_packet(pkt), which the
#   existing GroundStationGUI._feed_new_tabs() calls. No second serial
#   connection is ever opened.
# ============================================================

from PyQt6.QtWidgets import QDoubleSpinBox  # NEW (tab 3 only)

SIM_MODE_LIVE = "Live Sensor Data"
SIM_MODE_AUTO = "Automatic Simulation"

# ---- live mode: protection against bad data ----
SIM_MAX_PLAUSIBLE_SPEED_MS = 600.0   # altitude jumps implying more are glitches
SIM_GLITCH_MAX_CONSEC = 5            # ... but accept a new level after this many
SIM_TRAJ_DRAW_POINTS = 800           # max trajectory points painted per frame

# ---- automatic mode: flight-profile constants ----
AUTO_G = 9.81                  # m/s^2, deceleration while coasting
AUTO_IGNITION_S = 1.0          # motor ignition on the pad before lift-off
AUTO_CLEAR_TOWER_M = 5.0       # LAUNCH phase ends once the rocket is this high
AUTO_APOGEE_HOLD_S = 1.5       # time spent in the APOGEE phase
AUTO_CHUTE_RAMP_S = 2.0        # parachute deployment: sink rate ramps 0 -> descent speed
AUTO_MIN_DESCENT_MS = 1.0
AUTO_MAX_DESCENT_MS = 200.0
AUTO_DEFAULT_MAX_ALT_M = 1000.0
AUTO_DEFAULT_BURN_S = 3.5
AUTO_DEFAULT_DESCENT_MS = 12.0
AUTO_DEFAULT_DURATION_S = 120.0
AUTO_SPEEDS = [("0.5x", 0.5), ("1x", 1.0), ("2x", 2.0), ("5x", 5.0), ("10x", 10.0)]

SIM_PHASE_COLORS = dict(SIM_STATUS_COLORS)
SIM_PHASE_COLORS.update({
    "STANDBY": "#9E9E9E",
    "LAUNCH": "#FF5252",
    "POWERED ASCENT": "#00FFCC",
    "COASTING": "#40C4FF",
    "APOGEE": "#E040FB",
    "DESCENT": "#FF8A00",
    "LANDING": "#B2FF59",
})


def _mix(c1, c2, f):
    f = max(0.0, min(1.0, f))
    a, b = QColor(c1), QColor(c2)
    return QColor(int(a.red() + (b.red() - a.red()) * f),
                  int(a.green() + (b.green() - a.green()) * f),
                  int(a.blue() + (b.blue() - a.blue()) * f))


def _nice_ceil(v):
    """Round v up to a 'nice' scale limit (1, 2, 2.5, 5, 10 x 10^n)."""
    if v <= 0:
        return 1.0
    e = math.floor(math.log10(v))
    base = 10.0 ** e
    f = v / base
    for n in (1, 2, 2.5, 5, 10):
        if f <= n + 1e-9:
            return n * base
    return 10.0 * base


def _nice_step(raw):
    """Tick spacing (1, 2, 5, 10 x 10^n) close to raw."""
    if raw <= 0:
        return 1.0
    e = math.floor(math.log10(raw))
    base = 10.0 ** e
    f = raw / base
    for n in (1, 2, 5, 10):
        if f <= n + 1e-9:
            return n * base
    return 10.0 * base


# ------------------------------------------------------------------
# AUTOMATIC SIMULATION ENGINE (no telemetry involved)
# ------------------------------------------------------------------

class FlightProfile:
    """
    Closed-form flight profile used by the automatic mode.

        t0 .. t_ign   LAUNCH            ignition on the pad, then lift-off
        .. t2         POWERED ASCENT    constant net acceleration for burn time
        .. t3         COASTING          decelerates at g until v = 0
        .. t4         APOGEE            short hold at maximum altitude
        .. t_end      DESCENT / LANDING parachute: sink rate ramps up to the
                                        descent speed, then constant
        >= t_end      LANDED

    The net burn acceleration is solved so that the apogee equals exactly the
    requested maximum altitude for the chosen burn time.
    """

    def __init__(self, max_alt, burn_s, descent_speed,
                 duration=None, fit_duration=False):
        g = AUTO_G
        self.H = max(float(max_alt), 1.0)
        self.tb = max(float(burn_s), 0.2)
        self.note = ""

        A = self.tb ** 2 / (2.0 * g)
        B = self.tb ** 2 / 2.0
        self.a = (-B + math.sqrt(B * B + 4.0 * A * self.H)) / (2.0 * A)
        self.vb = self.a * self.tb                  # burnout speed (max speed)
        self.hb = 0.5 * self.a * self.tb ** 2       # burnout altitude

        self.t_ign = AUTO_IGNITION_S
        clear = min(AUTO_CLEAR_TOWER_M, 0.8 * self.hb)
        self.t_clear = math.sqrt(2.0 * clear / self.a)
        self.t1 = self.t_ign + self.t_clear         # end of LAUNCH
        self.t2 = self.t_ign + self.tb              # burnout
        self.t3 = self.t2 + self.vb / g             # apogee reached
        self.t4 = self.t3 + AUTO_APOGEE_HOLD_S      # start of descent

        if fit_duration and duration is not None:
            raw_vd = self._fit_descent_speed(self.H, float(duration) - self.t4)
        else:
            raw_vd = float(descent_speed)
        vd = min(max(raw_vd, AUTO_MIN_DESCENT_MS), AUTO_MAX_DESCENT_MS)
        if fit_duration and abs(vd - raw_vd) > 1e-9:
            self.note = "Requested duration not reachable - descent speed limited."
        self.vd = vd
        self.tr = min(AUTO_CHUTE_RAMP_S, self.H / self.vd)
        self.t_end = self.t4 + self.tr + (self.H - self.vd * self.tr / 2.0) / self.vd
        self.duration = self.t_end
        self.zone = min(max(10.0, 0.03 * self.H), 0.3 * self.H)  # LANDING zone
        self.sample_dt = max(0.05, self.duration / 2000.0)

        n = 240
        self.ghost = [(self.duration * i / (n - 1),
                       self.state(self.duration * i / (n - 1))[0])
                      for i in range(n)]

    @staticmethod
    def _fit_descent_speed(H, td):
        """Descent speed that makes the descent last td seconds."""
        ramp = AUTO_CHUTE_RAMP_S
        if td > ramp / 2.0:
            v1 = H / (td - ramp / 2.0)
            if H / v1 >= ramp:
                return v1
        return 1.5 * H / max(td, 0.2)

    def state(self, t):
        """-> (altitude m, vertical speed m/s, phase, flame 0..1, chute 0..1)"""
        g = AUTO_G
        if t < self.t_ign:
            return 0.0, 0.0, "LAUNCH", 0.15 + 0.35 * (t / self.t_ign), 0.0
        if t < self.t2:
            tau = t - self.t_ign
            ph = "LAUNCH" if t < self.t1 else "POWERED ASCENT"
            return 0.5 * self.a * tau * tau, self.a * tau, ph, 1.0, 0.0
        if t < self.t3:
            d = t - self.t2
            return (self.hb + self.vb * d - 0.5 * g * d * d,
                    self.vb - g * d, "COASTING", 0.0, 0.0)
        if t < self.t4:
            return self.H, 0.0, "APOGEE", 0.0, 0.0
        if t < self.t4 + self.tr:
            d = t - self.t4
            return (self.H - self.vd * d * d / (2.0 * self.tr),
                    -self.vd * d / self.tr, "DESCENT", 0.0, d / self.tr)
        if t < self.t_end:
            alt = max(self.H - self.vd * self.tr / 2.0
                      - self.vd * (t - self.t4 - self.tr), 0.0)
            ph = "LANDING" if alt <= self.zone else "DESCENT"
            return alt, -self.vd, ph, 0.0, 1.0
        return 0.0, 0.0, "LANDED", 0.0, 0.0


class AutoSimulationEngine:
    """Clock + state holder for the automatic mode. Pure Python (no Qt)."""

    def __init__(self):
        self.profile = None
        self.speed = 1.0
        self.reset()

    def reset(self):
        self.state = "idle"          # idle | running | paused | finished
        self.t = 0.0
        self.alt = 0.0
        self.vel = 0.0
        self.phase = "READY"
        self.flame = 0.0
        self.chute = 0.0
        self.separation = 0.0
        self.cansat_alt = 0.0
        self.rocket_chute = 0.0
        self.cansat_chute = 0.0
        self.max_alt = 0.0
        self.tx = deque(maxlen=SIM_HISTORY)
        self.ty = deque(maxlen=SIM_HISTORY)
        self._last_sample_t = -1e9

    def configure(self, profile):
        self.reset()
        self.profile = profile

    def start(self):
        if self.profile is None or self.state != "idle":
            return
        self.state = "running"
        self._evaluate(force_sample=True)

    def pause(self):
        if self.state == "running":
            self.state = "paused"

    def resume(self):
        if self.state == "paused":
            self.state = "running"

    def advance(self, dt):
        if self.state != "running" or self.profile is None:
            return False
        self.t += dt * self.speed
        finished = self.t >= self.profile.t_end
        if finished:
            self.t = self.profile.t_end
        self._evaluate(force_sample=finished)
        if finished:
            self.state = "finished"
        return True

    def _evaluate(self, force_sample=False):
        alt, vel, phase, flame, chute = self.profile.state(self.t)
        self.alt, self.vel, self.phase = alt, vel, phase
        self.flame, self.chute = flame, chute
        self.max_alt = max(self.max_alt, alt)

        # Mid-body CanSat separation is armed only after apogee, on descent.
        # The trigger is the first descending crossing of 800 m.
        prof = self.profile
        if prof.H > 800.0:
            ramp_end_alt = prof.H - prof.vd * prof.tr / 2.0
            if 800.0 >= ramp_end_alt:
                t_sep = prof.t4 + math.sqrt(
                    max(0.0, 2.0 * (prof.H - 800.0) * prof.tr / prof.vd))
            else:
                t_sep = prof.t4 + prof.tr + (ramp_end_alt - 800.0) / prof.vd
            elapsed = self.t - t_sep
            self.separation = min(1.0, max(0.0, elapsed / 1.8))
            # Both vehicles receive their own parachute after the CanSat clears.
            deploy = min(1.0, max(0.0, (elapsed - 1.0) / 1.5))
            self.rocket_chute = deploy
            self.cansat_chute = deploy
            self.cansat_alt = (800.0 if elapsed <= 1.0 else
                               max(0.0, 800.0 - 5.0 * (elapsed - 1.0)))
        else:
            self.separation = 0.0
            self.rocket_chute = 0.0
            self.cansat_chute = 0.0
            self.cansat_alt = 0.0

        if force_sample or self.t - self._last_sample_t >= self.profile.sample_dt:
            self.tx.append(self.t)
            self.ty.append(alt)
            self._last_sample_t = self.t


# ------------------------------------------------------------------
# SCENE WIDGET
# ------------------------------------------------------------------

class RocketSceneWidget(QWidget):
    """
    Painter-based 2D scene: launch pad, ground line, altitude scale, rocket
    and a flight-trajectory (altitude vs time) panel.

    `target_alt` is the REAL altitude (source of truth, shown as text).
    `disp_alt` only exists to make the drawing glide smoothly toward it; it is
    never used for any number on screen.
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(520, 420)
        self.setSizePolicy(QSizePolicy.Policy.Expanding,
                           QSizePolicy.Policy.Expanding)
        self.smooth = True
        self.reset()

    def reset(self):
        self.target_alt = 0.0
        self.disp_alt = 0.0
        self.max_alt = 0.0
        self.view_max = SIM_MIN_VIEW_M
        self.fixed_view = None
        self.status = "READY"
        self.vspeed = None
        self.signal_ok = True
        self.has_data = False
        self.note = ""
        self.flame = 0.0
        self.chute = 0.0
        self.separation = 0.0
        self.cansat_alt = 0.0
        self.rocket_chute = 0.0
        self.cansat_chute = 0.0
        self.streak = False
        self.traj_x = []
        self.traj_y = []
        self.span = None
        self.ghost = None
        self.update()

    def set_state(self, alt, max_alt, status, vspeed, has_data, note="",
                  flame=0.0, chute=0.0, streak=False, separation=0.0,
                  cansat_alt=0.0, rocket_chute=None, cansat_chute=0.0):
        self.target_alt = alt
        self.max_alt = max_alt
        self.status = status
        self.vspeed = vspeed
        self.has_data = has_data
        self.note = note
        self.flame = flame
        self.chute = chute
        self.separation = min(1.0, max(0.0, separation))
        self.cansat_alt = max(0.0, cansat_alt)
        self.rocket_chute = chute if rocket_chute is None else rocket_chute
        self.cansat_chute = cansat_chute
        self.streak = streak
        self.update()

    def set_trajectory(self, xs, ys, span=None, ghost=None):
        self.traj_x = xs
        self.traj_y = ys
        self.span = span
        self.ghost = ghost

    def set_signal(self, ok):
        if ok != self.signal_ok:
            self.signal_ok = ok
            self.update()

    def set_smooth(self, flag):
        self.smooth = bool(flag)

    def set_fixed_view(self, v):
        self.fixed_view = v
        if v:
            self.view_max = v
        self.update()

    def snap(self):
        self.disp_alt = self.target_alt
        self.view_max = self._target_view_max()
        self.update()

    def _target_view_max(self):
        top = max(self.target_alt, self.max_alt, 0.0) * 1.15
        if self.fixed_view:
            return max(self.fixed_view, _nice_ceil(top))
        return _nice_ceil(max(SIM_MIN_VIEW_M, top))

    def advance(self, dt):
        """Smoothing step. Returns True if something moved and a repaint is needed."""
        changed = False
        d = self.target_alt - self.disp_alt
        if self.smooth:
            if abs(d) > 0.005:
                self.disp_alt += d * (1.0 - math.exp(-dt / SIM_SMOOTH_TAU_S))
                changed = True
            elif d != 0.0:
                self.disp_alt = self.target_alt
                changed = True
        elif d != 0.0:
            self.disp_alt = self.target_alt
            changed = True
        dv = self._target_view_max() - self.view_max
        if abs(dv) > 0.01:
            self.view_max += dv * (1.0 - math.exp(-dt / SIM_VIEW_TAU_S))
            changed = True
        if self.flame > 0.02:
            changed = True            # flame flickers every frame
        return changed

    # ---------------- trajectory panel ----------------
    def _paint_trajectory(self, p, left, w, pad_y, y_of):
        """Draws the panel + trace. Returns tick labels to draw after the ground."""
        tx0 = left + (w - left) * 0.52
        tx1 = w - 16.0
        if tx1 - tx0 < 90:
            return []
        top = 72.0
        xs = list(self.traj_x)
        ys = list(self.traj_y)
        if self.span is not None:
            t0, t1 = self.span
        elif xs:
            t0 = xs[0]
            t1 = t0 + max(xs[-1] - t0, 10.0) * 1.04
        else:
            t0, t1 = 0.0, 10.0
        if t1 - t0 < 1e-6:
            t1 = t0 + 1.0
        span_t = t1 - t0

        def x_of(t):
            return tx0 + (t - t0) / span_t * (tx1 - tx0)

        panel = QRectF(tx0 - 6, top, (tx1 - tx0) + 12, pad_y - top)
        p.setPen(QPen(QColor(255, 255, 255, 40), 1))
        p.setBrush(QColor(0, 0, 0, 70))
        p.drawRect(panel)
        p.setFont(QFont("Consolas", 8, QFont.Weight.Bold))
        p.setPen(QColor(220, 220, 220, 200))
        p.drawText(QPointF(tx0 + 2, top + 14), "FLIGHT TRAJECTORY  (altitude vs time)")

        labels = []
        step = _nice_step(span_t / 4.0)
        k = 0
        while k * step <= span_t + 1e-9 and k < 40:
            x = x_of(t0 + k * step)
            p.setPen(QPen(QColor(255, 255, 255, 28), 1, Qt.PenStyle.DotLine))
            p.drawLine(QPointF(x, top), QPointF(x, pad_y))
            labels.append((x, f"{k * step:g}s"))
            k += 1

        p.save()
        p.setClipRect(panel)
        p.setBrush(Qt.BrushStyle.NoBrush)

        if self.ghost:                                  # planned profile (auto mode)
            gp = QPainterPath()
            for i, (t, a) in enumerate(self.ghost):
                pt = QPointF(x_of(t), y_of(a))
                if i == 0:
                    gp.moveTo(pt)
                else:
                    gp.lineTo(pt)
            p.setPen(QPen(QColor(255, 255, 255, 85), 1.5, Qt.PenStyle.DotLine))
            p.drawPath(gp)

        n = min(len(xs), len(ys))
        if n >= 1:
            stride = max(1, n // SIM_TRAJ_DRAW_POINTS)
            idx = list(range(0, n, stride))
            if idx[-1] != n - 1:
                idx.append(n - 1)
            tp = QPainterPath()
            for j, i in enumerate(idx):
                pt = QPointF(x_of(xs[i]), y_of(ys[i]))
                if j == 0:
                    tp.moveTo(pt)
                else:
                    tp.lineTo(pt)
            p.setPen(QPen(QColor("#00FFCC"), 2))
            p.drawPath(tp)
            p.setPen(QPen(QColor("#FFFFFF"), 1.5))
            p.setBrush(QColor("#00FFCC"))
            p.drawEllipse(QPointF(x_of(xs[n - 1]), y_of(ys[n - 1])), 4.5, 4.5)
        p.restore()
        return labels

    # ---------------- drawing ----------------
    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        w, h = self.width(), self.height()
        left = 66                                   # altitude-scale strip
        ground_h = max(56, int(h * 0.13))
        pad_y = h - ground_h                        # y of altitude = 0
        rh = max(60.0, min(120.0, h * 0.16))        # rocket height (px)
        rw = rh * 0.26
        top_m = rh * 1.9 + 40                       # room for nose + parachute
        scale_px = max(10.0, pad_y - top_m)
        cx = left + (w - left) * 0.28
        vmax = max(self.view_max, 1.0)

        def y_of(alt):
            return pad_y - (max(alt, 0.0) / vmax) * scale_px

        # --- sky (darkens with altitude) + stars ---
        f = min(1.0, max(0.0, self.disp_alt) / 3000.0)
        g = QLinearGradient(0, 0, 0, pad_y)
        g.setColorAt(0.0, _mix("#071226", "#000000", f))
        g.setColorAt(1.0, _mix("#2B5F8E", "#0B1B33", f))
        p.fillRect(QRectF(0, 0, w, pad_y), QBrush(g))

        p.setPen(Qt.PenStyle.NoPen)
        star_alpha = int(50 + 205 * f)
        for i in range(70):   # fixed, decorative star positions (not data)
            sx = left + ((math.sin(i * 12.9898) * 43758.5453) % 1.0) * (w - left)
            sy = ((math.sin(i * 78.233) * 12345.6789) % 1.0) * pad_y * 0.85
            sr = 0.8 + ((i * 37) % 10) / 10.0
            p.setBrush(QColor(255, 255, 255, star_alpha))
            p.drawEllipse(QPointF(sx, sy), sr, sr)

        # --- flight trajectory panel (shares the altitude scale) ---
        tick_labels = self._paint_trajectory(p, left, w, pad_y, y_of)

        # --- altitude scale ---
        p.fillRect(QRectF(0, 0, left, pad_y), QColor(0, 0, 0, 110))
        p.setPen(QPen(QColor("#CFCFCF"), 1))
        p.drawLine(QPointF(left, 0), QPointF(left, pad_y))
        step = _nice_step(vmax / 6.0)
        p.setFont(QFont("Consolas", 8))
        k = 0
        while k * step <= vmax + 1e-9 and k < 60:
            a = k * step
            y = y_of(a)
            p.setPen(QPen(QColor(255, 255, 255, 45), 1, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(left, y), QPointF(w, y))
            p.setPen(QColor("#CFCFCF"))
            p.drawText(QRectF(0, y - 8, left - 8, 16),
                       Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter,
                       f"{a:g} m")
            k += 1

        # --- apogee marker ---
        if self.max_alt > 0.5:
            y = y_of(self.max_alt)
            p.setPen(QPen(QColor("#FF8A00"), 1.5, Qt.PenStyle.DashLine))
            p.drawLine(QPointF(left, y), QPointF(w, y))
            p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
            p.setPen(QColor("#FF8A00"))
            p.drawText(QPointF(left + 8, y - 5), f"APOGEE {self.max_alt:.1f} m")

        # --- ground ---
        gg = QLinearGradient(0, pad_y, 0, h)
        gg.setColorAt(0.0, QColor("#3A5A3A"))
        gg.setColorAt(1.0, QColor("#142014"))
        p.fillRect(QRectF(0, pad_y, w, ground_h), QBrush(gg))
        p.setPen(QPen(QColor("#E0E0E0"), 2))
        p.drawLine(QPointF(0, pad_y), QPointF(w, pad_y))
        p.setFont(QFont("Consolas", 9, QFont.Weight.Bold))
        p.setPen(QColor("#E6E6E6"))
        p.drawText(QPointF(left + 8, pad_y + 18), "GROUND LEVEL  ·  0 m")
        p.setFont(QFont("Consolas", 8))
        p.setPen(QColor("#E6E6E6"))
        for x, txt in tick_labels:
            p.drawText(QPointF(x - 10, pad_y + 18), txt)

        # --- launch pad + gantry tower ---
        pad_w = rw * 3.4
        p.setPen(QPen(QColor("#BDBDBD"), 1))
        p.setBrush(QColor("#6E6E6E"))
        p.drawRect(QRectF(cx - pad_w / 2, pad_y, pad_w, 10))
        p.setBrush(QColor("#4A4A4A"))
        p.drawRect(QRectF(cx - pad_w / 2 + 6, pad_y + 10, 8, ground_h * 0.35))
        p.drawRect(QRectF(cx + pad_w / 2 - 14, pad_y + 10, 8, ground_h * 0.35))

        tx = cx - pad_w / 2 - 14
        th = rh * 1.25
        p.setPen(QPen(QColor("#9E9E9E"), 2))
        p.drawLine(QPointF(tx, pad_y), QPointF(tx, pad_y - th))
        p.drawLine(QPointF(tx + 12, pad_y), QPointF(tx + 12, pad_y - th))
        n_seg = 6
        for i in range(n_seg):
            y0 = pad_y - th * i / n_seg
            y1 = pad_y - th * (i + 1) / n_seg
            p.drawLine(QPointF(tx, y0), QPointF(tx + 12, y1))
            p.drawLine(QPointF(tx + 12, y0), QPointF(tx, y1))

        # --- rocket (vertical position = smoothed real altitude) ---
        by = y_of(self.disp_alt)
        p.save()
        p.translate(cx, by)

        # live mode: motion streak while the measured altitude is increasing
        if self.streak:
            trail = min(rh * 1.8, max(0.0, pad_y - by))
            if trail > 4:
                tg = QLinearGradient(0, 0, 0, trail)
                tg.setColorAt(0.0, QColor(255, 190, 70, 170))
                tg.setColorAt(1.0, QColor(255, 190, 70, 0))
                p.setPen(Qt.PenStyle.NoPen)
                p.setBrush(QBrush(tg))
                p.drawRect(QRectF(-rw * 0.2, 0, rw * 0.4, trail))

        # automatic mode: engine flame
        if self.flame > 0.02:
            now = time.monotonic()
            fl = (rh * 0.6 * self.flame
                  * (0.85 + 0.15 * math.sin(now * 40.0)
                     + 0.08 * math.sin(now * 97.0)))
            fw = rw * 0.34
            outer = QPainterPath()
            outer.moveTo(-fw, 0)
            outer.quadTo(-fw * 0.6, fl * 0.55, 0, fl)
            outer.quadTo(fw * 0.6, fl * 0.55, fw, 0)
            outer.closeSubpath()
            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QColor(255, 120, 0, 210))
            p.drawPath(outer)
            inner = QPainterPath()
            inner.moveTo(-fw * 0.5, 0)
            inner.quadTo(-fw * 0.3, fl * 0.35, 0, fl * 0.62)
            inner.quadTo(fw * 0.3, fl * 0.35, fw * 0.5, 0)
            inner.closeSubpath()
            p.setBrush(QColor(255, 235, 130, 235))
            p.drawPath(inner)

        nh = rh * 0.05                 # nozzle height
        bh = rh * 0.67                 # body height
        body_bottom = -nh
        body_top = -nh - bh

        # fins
        p.setPen(QPen(QColor("#8E0000"), 1))
        p.setBrush(QColor("#E53935"))
        p.drawPolygon(QPolygonF([
            QPointF(-rw / 2, body_top + bh * 0.62),
            QPointF(-rw / 2 - rw * 0.6, body_bottom + rh * 0.02),
            QPointF(-rw / 2, body_bottom)]))
        p.drawPolygon(QPolygonF([
            QPointF(rw / 2, body_top + bh * 0.62),
            QPointF(rw / 2 + rw * 0.6, body_bottom + rh * 0.02),
            QPointF(rw / 2, body_bottom)]))

        # nozzle
        p.setPen(QPen(QColor("#222222"), 1))
        p.setBrush(QColor("#4A4A4A"))
        p.drawPolygon(QPolygonF([
            QPointF(-rw * 0.28, body_bottom), QPointF(rw * 0.28, body_bottom),
            QPointF(rw * 0.38, 0), QPointF(-rw * 0.38, 0)]))

        # body (shaded cylinder)
        bg = QLinearGradient(-rw / 2, 0, rw / 2, 0)
        bg.setColorAt(0.0, QColor("#90A4AE"))
        bg.setColorAt(0.45, QColor("#FFFFFF"))
        bg.setColorAt(1.0, QColor("#78909C"))
        p.setPen(QPen(QColor("#546E7A"), 1))
        p.setBrush(QBrush(bg))
        p.drawRect(QRectF(-rw / 2, body_top, rw, bh))

        # colour band
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QColor("#E53935"))
        p.drawRect(QRectF(-rw / 2, body_bottom - bh * 0.22, rw, bh * 0.1))

        # nose cone
        nose = QPainterPath()
        nose.moveTo(-rw / 2, body_top)
        nose.cubicTo(-rw / 2, body_top - rh * 0.16,
                     -rw * 0.12, -rh + rh * 0.03, 0, -rh)
        nose.cubicTo(rw * 0.12, -rh + rh * 0.03,
                     rw / 2, body_top - rh * 0.16, rw / 2, body_top)
        nose.closeSubpath()
        p.setPen(QPen(QColor("#8E0000"), 1))
        p.setBrush(QColor("#E53935"))
        p.drawPath(nose)

        # window
        p.setPen(QPen(QColor("#37474F"), 1.5))
        p.setBrush(QColor("#4FC3F7"))
        p.drawEllipse(QPointF(0, body_top + bh * 0.38), rw * 0.17, rw * 0.17)

        # Mid-body payload bay: CanSat is visible inside before ejection.
        bay_y = body_top + bh * 0.60
        bay_h = bh * 0.22
        p.setPen(QPen(QColor("#263238"), 1.2))
        p.setBrush(QColor("#263238"))
        p.drawRect(QRectF(-rw * 0.34, bay_y, rw * 0.68, bay_h))
        # The CanSat stays seated in the payload bay while the rocket is
        # ascending, at apogee, and while descending above the 800 m trigger.
        # Separation progress is zero in all those phases, so both doors remain
        # fully closed and the CanSat remains inside the rocket.

        # Two-stage sequence after the descent-only separation trigger:
        #   0.00-0.35: both mid-body doors open outward together.
        #   0.35-1.00: the CanSat is pushed outward through the open bay.
        sep_progress = min(1.0, max(0.0, self.separation))
        door_progress = min(1.0, sep_progress / 0.35)
        eject_progress = min(1.0, max(0.0, (sep_progress - 0.35) / 0.65))
        door_angle = 82.0 * door_progress
        door_w = rw * 0.42
        door_h = bay_h * 1.10

        # Left mid-body door and hinge.
        p.save()
        p.translate(-rw / 2, bay_y + bay_h / 2)
        p.rotate(-door_angle)
        p.setPen(QPen(QColor("#8E0000"), 1))
        p.setBrush(QColor("#EF5350"))
        p.drawRect(QRectF(-door_w, -door_h / 2, door_w, door_h))
        p.setBrush(QColor("#B0BEC5"))
        p.drawEllipse(QPointF(0, 0), 2.5, 2.5)
        p.restore()

        # Right mid-body door and hinge.
        p.save()
        p.translate(rw / 2, bay_y + bay_h / 2)
        p.rotate(door_angle)
        p.setPen(QPen(QColor("#8E0000"), 1))
        p.setBrush(QColor("#EF5350"))
        p.drawRect(QRectF(0, -door_h / 2, door_w, door_h))
        p.setBrush(QColor("#B0BEC5"))
        p.drawEllipse(QPointF(0, 0), 2.5, 2.5)
        p.restore()

        # Draw one CanSat only. It is initially centred inside the bay, then
        # moves smoothly outwards only after the doors have opened.
        p.save()
        p.translate(rw * (0.12 + 2.0 * eject_progress),
                    bay_y + bay_h * 0.5)
        p.setPen(QPen(QColor("#004D40"), 1))
        p.setBrush(QColor("#26A69A"))
        p.drawRoundedRect(QRectF(-rw * 0.22, -bay_h * 0.38,
                                 rw * 0.44, bay_h * 0.76), 2, 2)
        p.setPen(QPen(QColor("#E0F2F1"), 1))
        p.drawLine(QPointF(-rw * 0.13, -bay_h * 0.20),
                   QPointF(rw * 0.13, -bay_h * 0.20))
        p.restore()

        # Independent rocket parachute deploys after separation.
        if self.rocket_chute > 0.02:
            c = min(1.0, self.rocket_chute)
            cw = rw * 1.8 * c + 2.0
            cy = -rh - rh * 0.8 * c
            ch = rh * 0.42 * c + 1.0
            p.setPen(QPen(QColor(235, 235, 235, 190), 1))
            for sx in (-cw, -cw * 0.5, cw * 0.5, cw):
                p.drawLine(QPointF(sx, cy), QPointF(0, -rh))
            canopy = QPainterPath()
            canopy.moveTo(-cw, cy)
            canopy.cubicTo(-cw, cy - ch * 1.4, cw, cy - ch * 1.4, cw, cy)
            canopy.quadTo(cw * 0.5, cy + ch * 0.25, 0, cy)
            canopy.quadTo(-cw * 0.5, cy + ch * 0.25, -cw, cy)
            p.setPen(QPen(QColor("#BF360C"), 1.2))
            p.setBrush(QColor("#FF7043"))
            p.drawPath(canopy)
        p.restore()

        # Separate CanSat + its own parachute after it has cleared the bay.
        if self.separation >= 1.0:
            cs_y = y_of(self.cansat_alt) - rh * 0.45
            cs_x = cx + rw * 2.12
            cs_h = rh * 0.17
            cs_w = rw * 0.62
            p.save()
            p.translate(cs_x, cs_y)
            p.setPen(QPen(QColor("#004D40"), 1.2))
            p.setBrush(QColor("#26A69A"))
            p.drawRoundedRect(QRectF(-cs_w / 2, -cs_h, cs_w, cs_h), 3, 3)
            p.setPen(QPen(QColor("#E0F2F1"), 1))
            p.drawLine(QPointF(-cs_w * 0.25, -cs_h * 0.72),
                       QPointF(cs_w * 0.25, -cs_h * 0.72))
            if self.cansat_chute > 0.02:
                cc = min(1.0, self.cansat_chute)
                canopy_w = rw * 1.05 * cc + 1.0
                canopy_y = -cs_h - rh * 0.48 * cc
                canopy_h = rh * 0.24 * cc + 1.0
                p.setPen(QPen(QColor("#E0E0E0"), 1))
                p.drawLine(QPointF(-cs_w * 0.25, -cs_h),
                           QPointF(-canopy_w, canopy_y))
                p.drawLine(QPointF(cs_w * 0.25, -cs_h),
                           QPointF(canopy_w, canopy_y))
                cp = QPainterPath()
                cp.moveTo(-canopy_w, canopy_y)
                cp.cubicTo(-canopy_w, canopy_y - canopy_h,
                           canopy_w, canopy_y - canopy_h,
                           canopy_w, canopy_y)
                cp.quadTo(0, canopy_y + canopy_h * 0.25,
                          -canopy_w, canopy_y)
                p.setPen(QPen(QColor("#1565C0"), 1.2))
                p.setBrush(QColor("#42A5F5"))
                p.drawPath(cp)
            p.restore()

        # real (unsmoothed) altitude tag beside the rocket
        tag_x = cx + rw * 0.5 + rw * 0.6 + 12
        tag_y = min(max(by - rh * 0.5, 70), pad_y - 8)
        p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
        p.setPen(QColor("#00FFCC"))
        p.drawText(QPointF(tag_x, tag_y), f"{self.target_alt:.1f} m")
        if self.vspeed is not None and self.signal_ok:
            p.setFont(QFont("Consolas", 9))
            p.setPen(QColor("#E6E6E6"))
            p.drawText(QPointF(tag_x, tag_y + 15), f"{self.vspeed:+.1f} m/s")

        # --- HUD: phase pill ---
        col = QColor(SIM_PHASE_COLORS.get(self.status, "#FFFFFF"))
        p.setFont(QFont("Consolas", 12, QFont.Weight.Bold))
        tw = p.fontMetrics().horizontalAdvance(self.status)
        pill = QRectF(left + 10, 10, tw + 44, 28)
        p.setPen(QPen(col, 2))
        p.setBrush(QColor(0, 0, 0, 150))
        p.drawRoundedRect(pill, 8, 8)
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(col)
        p.drawEllipse(QPointF(pill.left() + 14, pill.center().y()), 5, 5)
        p.setPen(col)
        p.drawText(QPointF(pill.left() + 28, pill.top() + 19), self.status)

        note = self.note
        if not self.has_data:
            note = "Awaiting telemetry…"
        if note:
            p.setFont(QFont("Consolas", 9))
            p.setPen(QColor("#E6E6E6"))
            p.drawText(QPointF(left + 12, 56), note)

        if self.has_data and not self.signal_ok:
            bw = 330.0
            ban = QRectF(w - bw - 10, 10, bw, 28)
            p.setPen(QPen(QColor("#FF3333"), 2))
            p.setBrush(QColor(60, 0, 0, 200))
            p.drawRoundedRect(ban, 8, 8)
            p.setFont(QFont("Consolas", 10, QFont.Weight.Bold))
            p.setPen(QColor("#FF3333"))
            p.drawText(ban, Qt.AlignmentFlag.AlignCenter,
                       "⚠ TELEMETRY LOST — HOLDING LAST ALTITUDE")
        p.end()


# ------------------------------------------------------------------
# TAB 3 WIDGET
# ------------------------------------------------------------------

class RocketSimulationTab(QWidget):
    """
    Tab 3: 2D rocket flight simulation with two selectable modes.

    LIVE SENSOR DATA
        Data path:  SerialReaderThread -> telemetry_packet_signal ->
                    GroundStationGUI.handle_telemetry_packet(pkt) ->
                    GroundStationGUI._feed_new_tabs(pkt) ->
                    RocketSimulationTab.update_from_packet(pkt)
        Altitude comes only from pkt[SIM_ALT_KEY] (bmp_alt). Vertical speed is
        the slope of recent altitude samples and the flight status is derived
        from those two measured values. Missing / invalid / duplicate / glitch
        packets are rejected and the last valid position is held.

    AUTOMATIC SIMULATION
        Independent AutoSimulationEngine + FlightProfile; no telemetry needed.

    Both modes share the animation timer (QTimer, GUI thread, ~30 fps); all
    work per frame is a few arithmetic operations, so the GUI never blocks.
    Live telemetry keeps being tracked in the background while the automatic
    mode is displayed (and while the live display is paused).
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.mode = SIM_MODE_LIVE
        self.live_state = "running"          # idle | running | paused
        self.engine = AutoSimulationEngine()
        self._preview = None
        self._status_col = None
        self._link_col = None
        self._reset_live_state()
        self._build_ui()

        self._last_frame = time.monotonic()
        self._frame_n = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_frame)
        self.timer.start(SIM_FRAME_MS)
        self._on_mode_changed()

    # ------------------------------------------------------------
    # UI
    # ------------------------------------------------------------
    def _build_ui(self):
        self.setStyleSheet(
            "QDoubleSpinBox { background-color: #1B1B1B;"
            " border: 1px solid #383838; padding: 3px; }")
        root = QVBoxLayout(self)

        # ---- control bar ----
        gb_ctl = QGroupBox("SIMULATION CONTROL")
        cl = QHBoxLayout(gb_ctl)
        cl.addWidget(QLabel("Mode:"))
        self.cmb_mode = QComboBox()
        self.cmb_mode.addItems([SIM_MODE_LIVE, SIM_MODE_AUTO])
        self.cmb_mode.setMinimumWidth(190)
        cl.addWidget(self.cmb_mode)
        cl.addSpacing(14)

        def mkbtn(text, tip, color):
            b = QPushButton(text)
            b.setToolTip(tip)
            b.setMinimumWidth(84)
            b.setStyleSheet(
                f"QPushButton {{ color: {color}; font-weight: bold; }}"
                "QPushButton:disabled { color: #555555; }")
            return b

        self.btn_start = mkbtn("▶ Start", "Start the simulation / live tracking",
                               "#00FFCC")
        self.btn_pause = mkbtn("⏸ Pause", "Freeze the animation", "#FFD400")
        self.btn_resume = mkbtn("⏵ Resume", "Continue after a pause", "#69F0AE")
        self.btn_reset = mkbtn("⟲ Reset", "Clear the simulation back to its start",
                               "#FF8A00")
        for b in (self.btn_start, self.btn_pause, self.btn_resume, self.btn_reset):
            cl.addWidget(b)
        cl.addSpacing(14)
        self.btn_rezero = QPushButton("Re-zero ground level")
        self.btn_rezero.setToolTip(
            "Live mode: use the latest sensor altitude as the 0 m reference "
            "(only while READY or LANDED)")
        cl.addWidget(self.btn_rezero)
        self.lbl_speed_cap = QLabel("Playback speed:")
        self.cmb_speed = QComboBox()
        for txt, _v in AUTO_SPEEDS:
            self.cmb_speed.addItem(txt)
        self.cmb_speed.setCurrentIndex(1)
        cl.addWidget(self.lbl_speed_cap)
        cl.addWidget(self.cmb_speed)
        cl.addStretch()
        root.addWidget(gb_ctl)

        # ---- main row: scene | side panel ----
        row = QHBoxLayout()
        self.scene = RocketSceneWidget()
        gb_scene = QGroupBox("2D ROCKET FLIGHT SIMULATION")
        QVBoxLayout(gb_scene).addWidget(self.scene)
        row.addWidget(gb_scene, 3)

        side = QWidget()
        side.setMinimumWidth(380)
        side_l = QVBoxLayout(side)
        side_l.setContentsMargins(0, 0, 0, 0)

        # ---- automatic-mode parameters ----
        self.gb_params = QGroupBox("AUTOMATIC SIMULATION PARAMETERS")
        pg_ = QGridLayout(self.gb_params)

        def spin(lo, hi, val, dec, step, suffix):
            s = QDoubleSpinBox()
            s.setRange(lo, hi)
            s.setDecimals(dec)
            s.setSingleStep(step)
            s.setValue(val)
            s.setSuffix(suffix)
            return s

        self.spn_alt = spin(1000, 1000, 1000, 0, 50, " m")
        self.spn_alt.setToolTip("Fixed mission apogee: 1000 m")
        self.spn_burn = spin(0.5, 60, AUTO_DEFAULT_BURN_S, 1, 0.5, " s")
        self.spn_desc = spin(AUTO_MIN_DESCENT_MS, AUTO_MAX_DESCENT_MS,
                             AUTO_DEFAULT_DESCENT_MS, 1, 1.0, " m/s")
        self.spn_dur = spin(10, 7200, AUTO_DEFAULT_DURATION_S, 0, 10, " s")
        self.chk_fit = QCheckBox("Fit descent speed to the flight duration")
        self.chk_fit.setToolTip(
            "Off: you set the descent speed and the duration is calculated.\n"
            "On : you set the duration and the descent speed is calculated.")
        pg_.addWidget(QLabel("Maximum altitude (apogee):"), 0, 0)
        pg_.addWidget(self.spn_alt, 0, 1)
        pg_.addWidget(QLabel("Motor burn time (ascent):"), 1, 0)
        pg_.addWidget(self.spn_burn, 1, 1)
        pg_.addWidget(QLabel("Descent speed:"), 2, 0)
        pg_.addWidget(self.spn_desc, 2, 1)
        pg_.addWidget(QLabel("Total flight duration:"), 3, 0)
        pg_.addWidget(self.spn_dur, 3, 1)
        pg_.addWidget(self.chk_fit, 4, 0, 1, 2)
        self.lbl_profile = QLabel("")
        self.lbl_profile.setWordWrap(True)
        self.lbl_profile.setStyleSheet("color: #BDBDBD;")
        pg_.addWidget(self.lbl_profile, 5, 0, 1, 2)
        side_l.addWidget(self.gb_params)

        # ---- flight data ----
        gb = QGroupBox("FLIGHT DATA")
        g = QGridLayout(gb)
        self.lbl_status = QLabel("READY")
        self.lbl_status.setStyleSheet(
            f"color: {SIM_STATUS_COLORS['READY']}; font-size: 18pt; font-weight: bold;")
        self.lbl_alt = QLabel("Altitude: 0.0 m")
        self.lbl_alt.setStyleSheet(
            "color: #00FFCC; font-size: 15pt; font-weight: bold;")
        self.lbl_apogee = QLabel("Apogee: 0.0 m")
        self.lbl_apogee.setStyleSheet(
            "color: #FF8A00; font-size: 13pt; font-weight: bold;")
        self.lbl_speed = QLabel("Vertical Speed: -- m/s")
        self.lbl_speed.setStyleSheet("font-size: 13pt; font-weight: bold;")
        self.lbl_info1 = QLabel("")
        self.lbl_info2 = QLabel("")
        self.lbl_info3 = QLabel("")
        self.lbl_time = QLabel("Flight Time: 0.00 s")
        self.lbl_link = QLabel("")
        self.lbl_link.setStyleSheet("color: #888888; font-weight: bold;")
        g.addWidget(self.lbl_status, 0, 0, 1, 2)
        g.addWidget(self.lbl_alt, 1, 0, 1, 2)
        g.addWidget(self.lbl_apogee, 2, 0, 1, 2)
        g.addWidget(self.lbl_speed, 3, 0, 1, 2)
        g.addWidget(self.lbl_info1, 4, 0, 1, 2)
        g.addWidget(self.lbl_info2, 5, 0, 1, 2)
        g.addWidget(self.lbl_info3, 6, 0, 1, 2)
        g.addWidget(self.lbl_time, 7, 0)
        g.addWidget(self.lbl_link, 7, 1)
        side_l.addWidget(gb)

        # ---- altitude vs time graph ----
        gb_plot = QGroupBox("ALTITUDE vs TIME")
        pw = pg.PlotWidget(title="Altitude above launch pad")
        pw.setLabel("left", "Altitude", units="m")
        pw.setLabel("bottom", "Time", units="s")
        pw.showGrid(x=True, y=True, alpha=0.25)
        pw.setBackground("#121212")
        pw.setMinimumHeight(200)
        pw.getPlotItem().getAxis("left").setWidth(70)
        self.plot = pw
        self.curve = pw.plot(pen=pg.mkPen(color="#00FFCC", width=2))
        self.ground_line = pg.InfiniteLine(
            pos=0, angle=0, pen=pg.mkPen("#777777", width=1))
        self.apogee_line = pg.InfiniteLine(
            pos=0, angle=0,
            pen=pg.mkPen("#FF8A00", width=1, style=Qt.PenStyle.DashLine))
        pw.addItem(self.ground_line, ignoreBounds=True)
        pw.addItem(self.apogee_line, ignoreBounds=True)
        self.apogee_line.setVisible(False)
        QVBoxLayout(gb_plot).addWidget(pw)
        side_l.addWidget(gb_plot, 1)
        row.addWidget(side, 2)
        root.addLayout(row, 1)

        # ---- wiring (after every widget exists) ----
        self.cmb_mode.currentIndexChanged.connect(self._on_mode_changed)
        self.cmb_speed.currentIndexChanged.connect(self._on_speed_changed)
        self.btn_start.clicked.connect(self._on_start)
        self.btn_pause.clicked.connect(self._on_pause)
        self.btn_resume.clicked.connect(self._on_resume)
        self.btn_reset.clicked.connect(self._on_reset)
        self.btn_rezero.clicked.connect(self._rezero)
        for s in (self.spn_alt, self.spn_burn, self.spn_desc, self.spn_dur):
            s.valueChanged.connect(self._on_params_changed)
        self.chk_fit.toggled.connect(self._on_params_changed)
        self._on_params_changed()

    # ------------------------------------------------------------
    # LIVE-MODE STATE
    # ------------------------------------------------------------
    def _reset_live_state(self):
        self.status = "READY"
        self.baseline = None if SIM_USE_RELATIVE_ALTITUDE else 0.0
        self.base_samples = []
        self.win = deque()                       # (time_s, altitude_m)
        self.hx = deque(maxlen=SIM_HISTORY)
        self.hy = deque(maxlen=SIM_HISTORY)
        self.t_offset = 0.0
        self.prev_raw_t = None
        self.last_x = None
        self.max_alt = 0.0
        self.cur_alt = 0.0
        self.raw_alt = None
        self.alt_source = SIM_ALT_KEY
        self.vspeed = None
        self.launch_count = 0
        self.descent_count = 0
        self.land_since = None
        self.launch_x = None
        self._launch_cand = None
        self.land_x = None
        self.glitch_count = 0
        self.last_packet_wall = None
        self.signal_ok = True
        self.packets = 0
        self.rejected = 0
        self._live_sep_started = None

    def _live_flight_time(self):
        if self.launch_x is None or self.last_x is None:
            return 0.0
        end = self.land_x if self.land_x is not None else self.last_x
        return max(0.0, end - self.launch_x)

    # ------------------------------------------------------------
    # BUTTON / MODE HANDLERS
    # ------------------------------------------------------------
    def _on_mode_changed(self, _idx=None):
        new = self.cmb_mode.currentText()
        if (new != self.mode and self.mode == SIM_MODE_AUTO
                and self.engine.state == "running"):
            self.engine.pause()          # never run unseen in the background
        self.mode = new
        live = new == SIM_MODE_LIVE
        self.gb_params.setVisible(not live)
        self.lbl_speed_cap.setVisible(not live)
        self.cmb_speed.setVisible(not live)
        self.btn_rezero.setVisible(live)
        self.scene.reset()
        self.scene.set_smooth(live)
        self._last_frame = time.monotonic()
        self._present()
        self.scene.snap()

    def _on_speed_changed(self, _idx=None):
        self.engine.speed = AUTO_SPEEDS[max(0, self.cmb_speed.currentIndex())][1]
        if self.mode == SIM_MODE_AUTO:
            self._present_auto(graph=False)

    def _on_start(self):
        if self.mode == SIM_MODE_LIVE:
            if self.live_state == "idle":
                self.live_state = "running"
                self._present()
            return
        if self.engine.state == "finished":
            self.engine.reset()
        if self.engine.state == "idle":
            self.engine.configure(self._make_profile())
            self.engine.speed = AUTO_SPEEDS[self.cmb_speed.currentIndex()][1]
            self.engine.start()
            self._last_frame = time.monotonic()
        self._present()

    def _on_pause(self):
        if self.mode == SIM_MODE_LIVE:
            if self.live_state == "running":
                self.live_state = "paused"
        else:
            self.engine.pause()
        self._present()

    def _on_resume(self):
        if self.mode == SIM_MODE_LIVE:
            if self.live_state == "paused":
                self.live_state = "running"
        else:
            self.engine.resume()
            self._last_frame = time.monotonic()
        self._present()
        if self.mode == SIM_MODE_LIVE:
            self.scene.snap()

    def _on_reset(self):
        if self.mode == SIM_MODE_LIVE:
            self._reset_live_state()
            self.live_state = "idle"     # press Start to track again
            self.scene.reset()
            self.scene.set_smooth(True)
        else:
            self.engine.reset()
            self.engine.speed = AUTO_SPEEDS[self.cmb_speed.currentIndex()][1]
            self.scene.reset()
            self.scene.set_smooth(False)
            self._on_params_changed()
        self._present()
        self.scene.snap()

    def reset(self):
        """Kept for compatibility with the previous revision."""
        self._on_reset()

    def _rezero(self):
        if (self.status in ("READY", "LANDED") and self.raw_alt is not None
                and self.live_state != "idle"):
            self.baseline = self.raw_alt
            self.base_samples = [self.raw_alt]
            self.cur_alt = 0.0
            self.max_alt = 0.0
            self.status = "READY"
            self.launch_count = 0
            self.launch_x = None
            self.land_x = None
            self.land_since = None
            self.win.clear()
            self.vspeed = None
            if self.mode == SIM_MODE_LIVE and self.live_state == "running":
                self._present()

    # ------------------------------------------------------------
    # AUTOMATIC-MODE PARAMETERS
    # ------------------------------------------------------------
    def _make_profile(self):
        return FlightProfile(self.spn_alt.value(), self.spn_burn.value(),
                             self.spn_desc.value(), self.spn_dur.value(),
                             self.chk_fit.isChecked())

    def _on_params_changed(self, *_args):
        prof = self._make_profile()
        self._preview = prof
        # the field that is "derived" is shown for information (and disabled)
        if self.chk_fit.isChecked():
            self.spn_desc.blockSignals(True)
            self.spn_desc.setValue(prof.vd)
            self.spn_desc.blockSignals(False)
        else:
            self.spn_dur.blockSignals(True)
            self.spn_dur.setValue(prof.duration)
            self.spn_dur.blockSignals(False)
        txt = (f"Burnout: {prof.hb:.0f} m at {prof.vb:.0f} m/s "
               f"(net accel {prof.a:.1f} m/s²)\n"
               f"Apogee at T+{prof.t3:.1f} s · landing at T+{prof.t_end:.1f} s · "
               f"descent {prof.vd:.1f} m/s\n"
               "Mid-body CanSat ejection: 800 m on descent only")
        if prof.note:
            txt += f"\n{prof.note}"
        self.lbl_profile.setText(txt)
        self._apply_param_enable()
        if self.mode == SIM_MODE_AUTO and self.engine.state == "idle":
            self._present_auto(graph=True)

    def _apply_param_enable(self):
        unlocked = self.engine.state in ("idle", "finished")
        fit = self.chk_fit.isChecked()
        self.spn_alt.setEnabled(unlocked)
        self.spn_burn.setEnabled(unlocked)
        self.chk_fit.setEnabled(unlocked)
        self.spn_desc.setEnabled(unlocked and not fit)
        self.spn_dur.setEnabled(unlocked and fit)

    # ------------------------------------------------------------
    # LIVE TELEMETRY HOOK
    # ------------------------------------------------------------
    def update_from_packet(self, pkt):
        # --- choose altitude source, reject invalid values ---
        alt = pkt.get(SIM_ALT_KEY)
        source = SIM_ALT_KEY
        if alt is None or not math.isfinite(alt):
            alt = pkt.get(SIM_ALT_FALLBACK_KEY)
            source = SIM_ALT_FALLBACK_KEY
        t_ms = pkt.get("time_ms")
        self.last_packet_wall = time.time()
        valid = alt is not None and math.isfinite(alt) and t_ms is not None

        live_view = (self.mode == SIM_MODE_LIVE
                     and self.live_state == "running")

        if self.live_state == "idle":
            # tracking stopped: only remember the latest raw reading
            if valid:
                self.raw_alt = alt
                self.alt_source = source
            if self.mode == SIM_MODE_LIVE:
                self._live_labels()
            return
        if not valid:
            self.rejected += 1
            if live_view:
                self._live_labels()
            return

        # --- payload time handling (duplicates / restart) ---
        raw_t = t_ms / 1000.0
        if self.prev_raw_t is not None and raw_t < self.prev_raw_t - 2.0:
            if self.status in ("READY", "LANDED"):
                self._reset_live_state()          # payload rebooted = new flight
            else:
                # restarted mid-flight: keep apogee/state, continue the time axis
                self.t_offset = (self.last_x or 0.0) - raw_t + 0.5
                self.win.clear()
        x = raw_t + self.t_offset
        if self.last_x is not None and x <= self.last_x:
            return                                 # duplicate / out-of-order packet

        # --- glitch filter: physically impossible altitude jumps ---
        if self.raw_alt is not None and self.last_x is not None:
            dt = max(x - self.last_x, 0.05)
            if (abs(alt - self.raw_alt) / dt > SIM_MAX_PLAUSIBLE_SPEED_MS
                    and self.glitch_count < SIM_GLITCH_MAX_CONSEC):
                self.glitch_count += 1
                self.rejected += 1
                if live_view:
                    self._live_labels()
                return
        self.glitch_count = 0

        self.prev_raw_t = raw_t
        self.last_x = x
        self.last_packet_wall = time.time()
        self.packets += 1
        self.alt_source = source
        self.raw_alt = alt

        # --- ground reference ---
        if self.baseline is None:
            self.base_samples.append(alt)
            ref = sum(self.base_samples) / len(self.base_samples)
            if len(self.base_samples) >= SIM_BASELINE_SAMPLES:
                self.baseline = ref
        else:
            ref = self.baseline
        rel = alt - ref
        self.cur_alt = rel

        # --- vertical speed from measured altitude ---
        self.win.append((x, rel))
        while self.win and x - self.win[0][0] > 6.0:
            self.win.popleft()
        vz = self._vertical_speed()
        self.vspeed = vz

        # --- flight status from measured altitude + speed ---
        self._update_state(x, rel, vz)
        if (self.vspeed is not None and self.vspeed < 0.0
                and self.max_alt >= 800.0 and rel <= 800.0
                and self._live_sep_started is None):
            # Interpolate the telemetry segment to the exact 800 m crossing,
            # so a packet that jumps from (for example) 820 m to 780 m does
            # not move the separation trigger down to 780 m.
            crossing_x = x
            samples = list(self.win)
            for (t0, a0), (t1, a1) in zip(samples[:-1], samples[1:]):
                if a0 > 800.0 >= a1 and t1 > t0 and a1 < a0:
                    fraction = (800.0 - a0) / (a1 - a0)
                    crossing_x = t0 + fraction * (t1 - t0)
                    break
            crossing_delay = max(0.0, x - crossing_x)
            self._live_sep_started = time.monotonic() - crossing_delay
        if self.status != "READY":
            self.max_alt = max(self.max_alt, rel)
        elif (self.baseline is not None and SIM_USE_RELATIVE_ALTITUDE
              and abs(rel) < SIM_LAUNCH_ALT_M
              and (vz is None or abs(vz) < 0.5)):
            self.baseline += SIM_BASELINE_EMA * rel   # track slow weather drift

        # --- history ---
        self.hx.append(x)
        self.hy.append(rel)

        if self.mode == SIM_MODE_LIVE and self.live_state == "running":
            self._present()

    def _vertical_speed(self):
        if len(self.win) < 2:
            return None
        t1, a1 = self.win[-1]
        ref = None
        for t0, a0 in self.win:                    # oldest sample inside the window
            dt = t1 - t0
            if SIM_SPEED_MIN_DT_S <= dt <= SIM_SPEED_WINDOW_S:
                ref = (t0, a0)
                break
        if ref is None:                            # slow telemetry: newest old-enough sample
            for t0, a0 in reversed(self.win):
                if t1 - t0 >= SIM_SPEED_MIN_DT_S:
                    ref = (t0, a0)
                    break
        if ref is None:
            return None
        return (a1 - ref[1]) / (t1 - ref[0])

    def _update_state(self, x, rel, vz):
        s = self.status
        if s == "READY":
            if (self.baseline is not None and vz is not None
                    and rel >= SIM_LAUNCH_ALT_M and vz >= SIM_LAUNCH_SPEED_MS):
                if self.launch_count == 0:
                    self._launch_cand = x
                self.launch_count += 1
            else:
                self.launch_count = 0
            if self.launch_count >= SIM_LAUNCH_CONFIRM:
                self.status = "ASCENDING"
                self.launch_x = self._launch_cand
                self.descent_count = 0
                self.land_since = None
            return

        if s == "ASCENDING":
            if (vz is not None and vz <= -SIM_DESCENT_SPEED_MS
                    and rel <= self.max_alt - SIM_APOGEE_DROP_M):
                self.descent_count += 1
            else:
                self.descent_count = 0
            if self.descent_count >= SIM_DESCENT_CONFIRM:
                self.status = "DESCENDING"

        if self.status in ("ASCENDING", "DESCENDING"):
            slow = vz is not None and abs(vz) <= SIM_LANDED_SPEED_MS
            if rel <= SIM_LANDED_ALT_M and slow:
                if self.land_since is None:
                    self.land_since = x
                elif x - self.land_since >= SIM_LANDED_HOLD_S:
                    self.status = "LANDED"
                    self.land_x = x
            else:
                self.land_since = None

    # ------------------------------------------------------------
    # PRESENTATION (labels / scene / graph)
    # ------------------------------------------------------------
    def _present(self, graph=True):
        if self.mode == SIM_MODE_LIVE:
            self._present_live()
        else:
            self._present_auto(graph)
        self._update_controls()

    def _update_controls(self):
        st = self.live_state if self.mode == SIM_MODE_LIVE else self.engine.state
        self.btn_start.setEnabled(st in ("idle", "finished"))
        self.btn_pause.setEnabled(st == "running")
        self.btn_resume.setEnabled(st == "paused")
        self.btn_reset.setEnabled(True)
        self.btn_rezero.setEnabled(
            self.mode == SIM_MODE_LIVE and self.live_state != "idle"
            and self.status in ("READY", "LANDED") and self.raw_alt is not None)
        self._apply_param_enable()

    def _set_labels(self, phase, alt, apogee, vz, time_caption, tval,
                    i1, i2, i3, link, link_col):
        col = SIM_PHASE_COLORS.get(phase, "#FFFFFF")
        self.lbl_status.setText(phase)
        if col != self._status_col:
            self._status_col = col
            self.lbl_status.setStyleSheet(
                f"color: {col}; font-size: 18pt; font-weight: bold;")
        self.lbl_alt.setText(f"Altitude: {alt:.1f} m")
        self.lbl_apogee.setText(f"Apogee: {apogee:.1f} m")
        self.lbl_speed.setText("Vertical Speed: -- m/s" if vz is None
                               else f"Vertical Speed: {vz:+.1f} m/s")
        self.lbl_info1.setText(i1)
        self.lbl_info2.setText(i2)
        self.lbl_info3.setText(i3)
        self.lbl_time.setText(f"{time_caption}: {tval:.2f} s")
        self.lbl_link.setText(link)
        if link_col != self._link_col:
            self._link_col = link_col
            self.lbl_link.setStyleSheet(f"color: {link_col}; font-weight: bold;")

    # ---------------- live ----------------
    def _live_status_text(self):
        return "STANDBY" if self.live_state == "idle" else self.status

    def _live_link(self):
        if self.last_packet_wall is None:
            return "Telemetry: waiting", "#888888"
        age = time.time() - self.last_packet_wall
        if age <= LINK_LOST_TIMEOUT_S:
            return "Telemetry: LIVE", "#00FFCC"
        return f"Telemetry: NO SIGNAL ({age:.0f}s)", "#FF3333"

    def _live_labels(self):
        vz = self.vspeed if (self.signal_ok and self.live_state != "idle") else None
        if self.raw_alt is None:
            i1 = f"Sensor altitude ({self.alt_source}): --"
        else:
            i1 = f"Sensor altitude ({self.alt_source}): {self.raw_alt:.2f} m"
        if not SIM_USE_RELATIVE_ALTITUDE:
            i2 = "Ground reference: raw sensor value (0 m)"
        elif self.baseline is None:
            i2 = (f"Ground reference: calibrating "
                  f"({len(self.base_samples)}/{SIM_BASELINE_SAMPLES})")
        else:
            i2 = f"Ground reference: {self.baseline:.2f} m"
        i3 = f"Packets used: {self.packets}  ·  rejected: {self.rejected}"
        link, lcol = self._live_link()
        self._set_labels(self._live_status_text(), self.cur_alt, self.max_alt, vz,
                         "Flight Time", self._live_flight_time(),
                         i1, i2, i3, link, lcol)

    def _present_live(self):
        if self.live_state == "idle":
            note = "Live tracking stopped — press Start"
        elif self.live_state == "paused":
            note = "PAUSED — display frozen, telemetry still recorded"
        elif self.baseline is None:
            note = (f"Calibrating ground level "
                    f"({len(self.base_samples)}/{SIM_BASELINE_SAMPLES})…")
        else:
            note = ""
        streak = (self.status == "ASCENDING" and self.vspeed is not None
                  and self.vspeed > 1.0 and self.signal_ok)
        self.scene.set_signal(self.signal_ok)
        sep = 0.0
        cs_alt = 0.0
        cs_chute = 0.0
        if self._live_sep_started is not None:
            sep_elapsed = max(0.0, time.monotonic() - self._live_sep_started)
            sep = min(1.0, sep_elapsed / 1.8)
            cs_alt = max(0.0, 800.0 - 5.0 * max(0.0, sep_elapsed - 1.0))
            cs_chute = min(1.0, max(0.0, (sep_elapsed - 1.0) / 1.5))
        self.scene.set_state(
            self.cur_alt, self.max_alt, self._live_status_text(),
            self.vspeed, self.packets > 0, note, streak=streak,
            separation=sep, cansat_alt=cs_alt,
            rocket_chute=cs_chute, cansat_chute=cs_chute)
        self.scene.set_trajectory(self.hx, self.hy)
        self.curve.setData(list(self.hx), list(self.hy))
        if self.status != "READY" and self.live_state != "idle":
            self.apogee_line.setPos(self.max_alt)
            self.apogee_line.setVisible(True)
        else:
            self.apogee_line.setVisible(False)
        self._live_labels()

    # ---------------- automatic ----------------
    def _present_auto(self, graph=True):
        e = self.engine
        prof = e.profile if (e.profile is not None and e.state != "idle") \
            else self._preview
        if prof is None:
            return
        fixed = _nice_ceil(max(SIM_MIN_VIEW_M, prof.H * 1.15))
        if self.scene.fixed_view != fixed:
            self.scene.set_fixed_view(fixed)

        if e.state == "idle":
            note = "AUTOMATIC SIMULATION — set parameters and press Start"
        elif e.state == "paused":
            note = f"PAUSED · T+{e.t:.1f} s"
        elif e.state == "finished":
            note = f"SIMULATION COMPLETE · T+{e.t:.1f} s"
        else:
            note = f"AUTOMATIC SIMULATION · T+{e.t:.1f} s"
        self.scene.set_signal(True)
        self.scene.set_state(
            e.alt, e.max_alt, e.phase, e.vel, True, note,
            flame=e.flame, chute=e.chute, separation=e.separation,
            cansat_alt=e.cansat_alt, rocket_chute=e.rocket_chute,
            cansat_chute=e.cansat_chute)
        self.scene.set_trajectory(e.tx, e.ty, span=(0.0, prof.duration),
                                  ghost=prof.ghost)

        if graph:
            self.curve.setData(list(e.tx), list(e.ty))
            if e.max_alt > 0.0:
                self.apogee_line.setPos(e.max_alt)
                self.apogee_line.setVisible(True)
            else:
                self.apogee_line.setVisible(False)

        link_txt, link_col = {
            "idle": ("Simulation: READY", "#888888"),
            "running": ("Simulation: RUNNING", "#00FFCC"),
            "paused": ("Simulation: PAUSED", "#FFD400"),
            "finished": ("Simulation: COMPLETE", "#40C4FF"),
        }[e.state]
        speed_txt = AUTO_SPEEDS[self.cmb_speed.currentIndex()][0]
        self._set_labels(
            e.phase, e.alt, e.max_alt, e.vel if e.state != "idle" else 0.0,
            "Elapsed Time", e.t,
            f"Target apogee: {prof.H:.0f} m  ·  max speed {prof.vb:.0f} m/s",
            f"Descent speed: {prof.vd:.1f} m/s  ·  total {prof.duration:.0f} s",
            f"Playback speed: {speed_txt}  ·  source: internal simulation",
            link_txt, link_col)

    # ------------------------------------------------------------
    # FRAME TIMER (GUI thread, non-blocking)
    # ------------------------------------------------------------
    def _on_frame(self):
        now = time.monotonic()
        dt = min(now - self._last_frame, 0.25)
        self._last_frame = now
        self._frame_n += 1

        if self.mode == SIM_MODE_AUTO and self.engine.state == "running":
            self.engine.advance(dt)
            done = self.engine.state == "finished"
            self._present_auto(graph=(done or self._frame_n % 4 == 0))
            if done:
                self._update_controls()

        if self.scene.advance(dt):
            self.scene.update()

        if self._frame_n % 15 == 0 and self.last_packet_wall is not None:
            age = time.time() - self.last_packet_wall
            ok = age <= LINK_LOST_TIMEOUT_S
            changed = ok != self.signal_ok
            self.signal_ok = ok
            if self.mode == SIM_MODE_LIVE:
                self.scene.set_signal(ok)
                self._live_labels()
                if changed:
                    self._update_controls()


# ============================================================
# SERIAL READER THREAD
# ============================================================

class SerialReaderThread(QThread):
    telemetry_packet_signal = pyqtSignal(dict, int)
    raw_console_signal = pyqtSignal(str)
    connection_error_signal = pyqtSignal(str)
    connection_status_signal = pyqtSignal(str)

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.running = True
        self.ser = None
        self.buffer = bytearray()

    def run(self):
        try:
            self.raw_console_signal.emit(
                f"[SERIAL] Opening {self.port} @ {self.baudrate} baud...")
            self.ser = serial.Serial(self.port, self.baudrate,
                                     timeout=0.05, write_timeout=0.5)
            time.sleep(0.2)
            self.connection_status_signal.emit(f"CONNECTED: {self.port}")
            self.raw_console_signal.emit(
                f"[SERIAL] Waiting for {EXPECTED_FIELDS}-field LoRa telemetry...")

            while self.running and self.ser is not None:
                try:
                    waiting = self.ser.in_waiting
                    if waiting > 0:
                        self.buffer.extend(self.ser.read(waiting))
                        if len(self.buffer) > 8192:      # runaway-noise guard
                            del self.buffer[:-2048]
                        self.process_buffer()
                    else:
                        self.msleep(5)
                except serial.SerialException as e:
                    self.connection_error_signal.emit(
                        f"Serial communication error: {e}")
                    break
                except Exception as e:
                    self.raw_console_signal.emit(
                        f"[PARSER ERROR] {type(e).__name__}: {e}")
                    self.msleep(10)

        except serial.SerialException as e:
            self.connection_error_signal.emit(
                f"Could not open {self.port}\nReason: {e}")
        except Exception as e:
            self.connection_error_signal.emit(
                f"Unexpected serial error\n{type(e).__name__}: {e}")
        finally:
            try:
                if self.ser is not None and self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.connection_status_signal.emit("DISCONNECTED")
            self.raw_console_signal.emit("[SERIAL] Reader thread stopped.")

    def process_buffer(self):
        while self.running:
            idx = self.buffer.find(b"\n")
            if idx == -1:
                return
            raw = bytes(self.buffer[:idx])
            del self.buffer[:idx + 1]
            self.parse_ascii_line(raw, len(raw) + 1)

    def parse_ascii_line(self, raw_bytes, packet_size):
        line = raw_bytes.decode("utf-8", errors="ignore").strip()
        if not line:
            return
        if not line.startswith("CAN,"):
            self.raw_console_signal.emit(f"[RX] {line}")
            return

        parts = line.split(",")
        if len(parts) not in (EXPECTED_FIELDS, GPS_FIELDS):
            self.raw_console_signal.emit(
                f"[PACKET ERROR] Expected {EXPECTED_FIELDS} or "
                f"{GPS_FIELDS} fields, got {len(parts)}")
            self.raw_console_signal.emit(f"[RAW] {line}")
            return

        try:
            pkt = {"time_ms": int(parts[1])}
            for i, key in enumerate(FIELD_KEYS[1:], start=2):
                pkt[key] = float(parts[i])

            pkt["accel_mag"] = math.sqrt(
                pkt["ax"] ** 2 + pkt["ay"] ** 2 + pkt["az"] ** 2)
            pkt["alt_diff"] = pkt["bmp_alt"] - pkt["dps_alt"]

            if len(parts) == GPS_FIELDS:
                lat, lon = float(parts[20]), float(parts[21])
                if not (math.isfinite(lat) and math.isfinite(lon)
                        and -90 <= lat <= 90 and -180 <= lon <= 180):
                    raise ValueError("Invalid GPS latitude or longitude")
                # (0,0) is what most GPS modules output with no fix
                if not (lat == 0.0 and lon == 0.0):
                    pkt["latitude"], pkt["longitude"] = lat, lon

            self.telemetry_packet_signal.emit(pkt, packet_size)

        except (ValueError, IndexError) as e:
            self.raw_console_signal.emit(f"[ASCII PARSE ERROR] {e}")
            self.raw_console_signal.emit(f"[RAW] {line}")

    def stop(self):
        self.running = False
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.wait(2000)




# ============================================================
# TAB 4 — STL-BASED INTERACTIVE 3D ROCKET SIMULATION
# ============================================================

class Rocket3DSimulationTab(QWidget):
    """
    Tab 4 only: STL rocket + STL CanSat separation scene.
    The telemetry path is unchanged; this widget only consumes already-parsed
    altitude packets from GroundStationGUI.
    """

    FRAME_MS = 33
    ROCKET_DEFAULT = "full body(1).stl"
    CANSAT_DEFAULT = "cansat.stl"
    ALTITUDE_TO_MODEL_UNITS = 1000.0  # STL geometry is millimetres
    # UPDATED: raise the bay and enlarge its vertical aperture.
    BAY_BOTTOM_MM = 700.0
    BAY_TOP_MM = 950.0
    BAY_CENTER_MM = (BAY_BOTTOM_MM + BAY_TOP_MM) / 2.0
    # Oversized mid-body doors overlap the rectangular bay aperture to hide
    # shell gaps at the outer edges when closed.
    DOOR_HALF_WIDTH_MM = 83.5  # narrow enough to stay within the rocket silhouette while overlapping the 78 mm opening
    BAY_OPENING_HALF_WIDTH_MM = 78.0
    EJECTION_DURATION_S = 1.8
    EJECTION_DISTANCE_MM = 1170.0  # 50% farther than the current 780 mm distance
    CANSAT_DESCENT_MS = 12.0  # retained for compatibility with earlier descent logic
    CANSAT_VERTICAL_OFFSET_M = 1.0  # keep CanSat within 1 m vertically of the rocket

    def __init__(self, parent=None):
        super().__init__(parent)
        base_dir = os.path.dirname(os.path.abspath(__file__))
        self.stl_path = os.path.join(base_dir, self.ROCKET_DEFAULT)
        self.cansat_path = os.path.join(base_dir, self.CANSAT_DEFAULT)

        self.mesh_items = []
        self.mesh_components = []
        self.cansat_item = None
        self.cansat_mesh = None
        self.door_items = []
        self.door_closed_vertices = []
        # NEW: independent canopy meshes and suspension-line items for each vehicle.
        self.parachute_items = {"rocket": None, "cansat": None}
        self.parachute_line_items = {"rocket": None, "cansat": None}
        self.ground_item = None
        self.model_center = np.zeros(3, dtype=float) if STL_3D_AVAILABLE else None
        self.rocket_bottom_mm = 0.0
        self.rocket_fire_items = []
        self.live_ascending = False

        self.current_altitude = 0.0
        self.max_altitude = 0.0
        self.mode = "Live Sensor Data"
        self.auto_elapsed = 0.0
        self.auto_running = False
        self.auto_paused = False
        self.last_tick = time.monotonic()

        self.previous_live_altitude = None
        self.live_ascending = False
        self.live_descent_confirm = 0
        self.live_descent_armed = False
        self.ejection_triggered = False
        self.separation_elapsed = -1.0
        self.separation_altitude = None
        self.cansat_altitude = 0.0
        self.cansat_x = 0.0
        self.cansat_y = 0.0
        self.rocket_landed = False
        self.cansat_landed = False

        self._build_ui()
        self._load_rocket_model(self.stl_path)
        self._load_cansat_model(self.cansat_path)

        self.timer = QTimer(self)
        self.timer.timeout.connect(self._on_frame)
        self.timer.start(self.FRAME_MS)

    def _build_ui(self):
        root = QVBoxLayout(self)
        controls = QHBoxLayout()

        controls.addWidget(QLabel("Mode:"))
        self.cmb_mode = QComboBox()
        self.cmb_mode.addItems(["Live Sensor Data", "Automatic Simulation"])
        self.cmb_mode.currentTextChanged.connect(self._mode_changed)
        controls.addWidget(self.cmb_mode)

        self.btn_load = QPushButton("Load Rocket STL")
        self.btn_load.clicked.connect(self._choose_rocket_stl)
        controls.addWidget(self.btn_load)

        self.btn_load_cansat = QPushButton("Load CanSat STL")
        self.btn_load_cansat.clicked.connect(self._choose_cansat_stl)
        controls.addWidget(self.btn_load_cansat)

        self.btn_start = QPushButton("▶ Start")
        self.btn_start.clicked.connect(self._start_auto)
        controls.addWidget(self.btn_start)
        self.btn_pause = QPushButton("⏸ Pause")
        self.btn_pause.clicked.connect(self._pause_auto)
        controls.addWidget(self.btn_pause)
        self.btn_reset = QPushButton("⟲ Reset")
        self.btn_reset.clicked.connect(self._reset_sim)
        controls.addWidget(self.btn_reset)
        self.chk_track = QCheckBox("Track rocket + CanSat")
        self.chk_track.setChecked(True)
        self.chk_track.setToolTip(
            "Keep both vehicles centred through ascent, ejection, descent, and landing. "
            "Uncheck to keep the camera target fixed for a fully free view.")
        controls.addWidget(self.chk_track)
        controls.addStretch()
        root.addLayout(controls)

        info = QHBoxLayout()
        self.lbl_model = QLabel("Rocket STL: waiting to load")
        self.lbl_model.setStyleSheet("color:#00FFCC;font-weight:bold;")
        self.lbl_alt = QLabel("Altitude: 0.0 m")
        self.lbl_apogee = QLabel("Apogee: 0.0 m")
        self.lbl_phase = QLabel("Phase: READY")
        self.lbl_cansat = QLabel("CanSat: INSIDE")
        for label in (self.lbl_alt, self.lbl_apogee, self.lbl_phase,
                      self.lbl_cansat):
            label.setStyleSheet("color:#00FFCC;font-weight:bold;")
        info.addWidget(self.lbl_model, 2)
        info.addWidget(self.lbl_alt)
        info.addWidget(self.lbl_apogee)
        info.addWidget(self.lbl_phase)
        info.addWidget(self.lbl_cansat)
        root.addLayout(info)

        if STL_3D_AVAILABLE:
            self.view = gl.GLViewWidget()
            # NEW: clear sky-blue background gives strong contrast with the vehicle.
            self.view.setBackgroundColor((105, 185, 235, 255))
            self.view.opts["distance"] = 3300
            self.view.opts["elevation"] = 13
            self.view.opts["azimuth"] = 35
            self.view.opts["center"] = QVector3D(0, 0, 900)
            # Broad, flat green terrain: the surface extends far beyond the
            # camera's normal viewing range so no small rectangular platform edge
            # is visible during orbiting, panning, or zooming.
            ground_mesh = trimesh.creation.box(extents=(200000, 200000, 4))
            ground_mesh.apply_translation((0, 0, -28))
            self.ground_item = self._make_mesh_item(
                ground_mesh, (0.12, 0.52, 0.16, 1.0))
            self.view.addItem(self.ground_item)
            axes = gl.GLAxisItem()
            axes.setSize(350, 350, 1900)
            self.view.addItem(axes)
            root.addWidget(self.view, 1)
            self.lbl_hint = QLabel(
                "Mouse: left-drag rotate · right-drag pan · wheel zoom. "
                "CanSat is housed in the mid-body bay; doors open on the "
                "descending 800 m crossing.")
        else:
            self.view = None
            self.lbl_hint = QLabel(
                "3D view dependencies are missing. Install with:\n"
                "pip install pyqtgraph PyOpenGL trimesh numpy")
            self.lbl_hint.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.lbl_hint.setStyleSheet("color:#FF8A00;font-size:12pt;")
            root.addWidget(self.lbl_hint, 1)

        self.lbl_hint.setStyleSheet("color:#BDBDBD;")
        self.lbl_hint.setWordWrap(True)
        root.addWidget(self.lbl_hint)

    @staticmethod
    def _make_mesh_item(mesh, color):
        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        faces = np.asarray(mesh.faces, dtype=np.uint32)
        md = gl.MeshData(vertexes=vertices, faces=faces)
        return gl.GLMeshItem(meshdata=md, smooth=True, drawFaces=True,
                             drawEdges=False, shader="shaded", color=color)

    def _remove_items(self, items):
        if self.view is not None:
            for item in list(items):
                try:
                    self.view.removeItem(item)
                except Exception:
                    pass
        items.clear()

    def _choose_rocket_stl(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select rocket STL model", self.stl_path,
            "STL files (*.stl);;All files (*)")
        if path:
            self.stl_path = path
            self._load_rocket_model(path)

    def _choose_cansat_stl(self):
        path, _ = QFileDialog.getOpenFileName(
            self, "Select CanSat STL model", self.cansat_path,
            "STL files (*.stl);;All files (*)")
        if path:
            self.cansat_path = path
            self._load_cansat_model(path)

    def _load_rocket_model(self, path):
        if not STL_3D_AVAILABLE or self.view is None:
            return
        if not os.path.isfile(path):
            self.lbl_model.setText("Rocket STL not found — use Load Rocket STL")
            return
        try:
            loaded = trimesh.load(path, force="mesh")
            if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
                raise ValueError("No triangular rocket mesh found")

            self._remove_items(self.mesh_items)
            self.mesh_components = loaded.split(only_watertight=False)
            if not self.mesh_components:
                self.mesh_components = [loaded]

            # The provided rocket STL has a continuous cylindrical mid-body.
            # Cut a clean, sharp rectangular aperture in the front-facing
            # mid-body shell. The wider door leaves overlap its side edges.
            for index, component in enumerate(self.mesh_components):
                component = component.copy()
                if index == 0:
                    centers = component.triangles_center
                    z = centers[:, 2]
                    radius = np.hypot(centers[:, 0], centers[:, 1])
                    angle = np.arctan2(centers[:, 1], centers[:, 0])
                    front_delta = np.arctan2(
                        np.sin(angle + math.pi / 2.0),
                        np.cos(angle + math.pi / 2.0))
                    opening_faces = (
                        (z >= self.BAY_BOTTOM_MM) &
                        (z <= self.BAY_TOP_MM) &
                        (np.abs(component.triangles_center[:, 0]) <=
                         self.BAY_OPENING_HALF_WIDTH_MM) &
                        (np.abs(front_delta) <= math.radians(60.0)) &
                        (radius > 65.0)
                    )
                    # Remove triangles that cross the rectangular cut boundary
                    # as well as triangles whose centres lie inside it. This
                    # prevents thin STL slivers from being left behind as
                    # spike/teeth-like structures around the bay opening.
                    cut_margin_mm = 2.5
                    tri_vertices = component.triangles
                    vertex_z = tri_vertices[:, :, 2]
                    vertex_x = np.abs(tri_vertices[:, :, 0])
                    vertex_radius = np.hypot(
                        tri_vertices[:, :, 0], tri_vertices[:, :, 1])
                    vertex_angle = np.arctan2(
                        tri_vertices[:, :, 1], tri_vertices[:, :, 0])
                    vertex_front_delta = np.arctan2(
                        np.sin(vertex_angle + math.pi / 2.0),
                        np.cos(vertex_angle + math.pi / 2.0))
                    vertex_in_opening = (
                        (vertex_z >= self.BAY_BOTTOM_MM - cut_margin_mm) &
                        (vertex_z <= self.BAY_TOP_MM + cut_margin_mm) &
                        (vertex_x <= self.BAY_OPENING_HALF_WIDTH_MM + cut_margin_mm) &
                        (np.abs(vertex_front_delta) <= math.radians(60.0)) &
                        (vertex_radius > 65.0)
                    )
                    boundary_slivers = np.any(vertex_in_opening, axis=1)
                    opening_faces |= boundary_slivers
                    component.update_faces(~opening_faces)
                    component.remove_unreferenced_vertices()
                if len(component.faces) == 0:
                    continue
                item = self._make_mesh_item(component, (0.72, 0.78, 0.84, 1.0))
                self.view.addItem(item)
                self.mesh_items.append(item)

            bounds = loaded.bounds
            ext = bounds[1] - bounds[0]
            center = (bounds[1] + bounds[0]) / 2.0
            self.model_center = center
            self.rocket_bottom_mm = float(bounds[0][2])
            self._create_rocket_fire_effect()
            self.view.opts["center"] = QVector3D(
                float(center[0]), float(center[1]), float(center[2]))
            self.view.opts["distance"] = max(float(max(ext)) * 2.15, 500.0)
            self._create_bay_doors()
            self.view.update()
            self.lbl_model.setText(
                f"Rocket STL: {os.path.basename(path)} | "
                f"X {ext[0]:.1f} · Y {ext[1]:.1f} · Z {ext[2]:.1f} mm")
            self.lbl_hint.setText(
                "Rocket STL loaded. Clean rectangular mid-body bay with "
                "oversized hinged doors covering the outer shell gaps.")
        except Exception as exc:
            self.lbl_model.setText(
                f"Rocket STL load error: {type(exc).__name__}: {exc}")

    def _load_cansat_model(self, path):
        if not STL_3D_AVAILABLE or self.view is None:
            return
        if not os.path.isfile(path):
            self.lbl_cansat.setText("CanSat STL: FILE NOT FOUND")
            return
        try:
            loaded = trimesh.load(path, force="mesh")
            if not isinstance(loaded, trimesh.Trimesh) or len(loaded.faces) == 0:
                raise ValueError("No triangular CanSat mesh found")
            self.cansat_mesh = loaded.copy()
            if self.cansat_item is not None:
                self.view.removeItem(self.cansat_item)
            self.cansat_item = self._make_mesh_item(
                self.cansat_mesh, (0.95, 0.62, 0.18, 1.0))
            self.view.addItem(self.cansat_item)
            ext = self.cansat_mesh.extents
            # Model coordinates are centred on X/Y with its base at Z=0.
            self.lbl_cansat.setText(
                f"CanSat STL: LOADED ({ext[0]:.0f}×{ext[1]:.0f}×{ext[2]:.0f} mm)")
            self._apply_scene_transforms()
        except Exception as exc:
            self.lbl_cansat.setText(
                f"CanSat STL load error: {type(exc).__name__}: {exc}")

    def _create_bay_doors(self):
        self._remove_items(self.door_items)
        self.door_closed_vertices.clear()
        if self.view is None:
            return
        # Two rectangular door leaves cover the opening with a small side
        # overlap. Their edges are fitted to the cylindrical shell so the
        # closed doors do not protrude beyond the rocket silhouette.
        # The hinge axis is placed directly on the same shell surface used by
        # the door vertices, preventing a visible gap as the doors swing open.
        shell_radius = 90.5  # tiny outward clearance avoids z-fighting, not hovering
        # Extend each door by 50% of the bay height above and below the opening,
        # so the closed panels overlap the shell generously at both ends.
        bay_height = self.BAY_TOP_MM - self.BAY_BOTTOM_MM
        vertical_overlap = 0.50 * bay_height
        z0 = self.BAY_BOTTOM_MM - vertical_overlap
        z1 = self.BAY_TOP_MM + vertical_overlap
        for side in (-1, 1):
            if side < 0:
                x0, x1 = -self.DOOR_HALF_WIDTH_MM, 0.0
            else:
                x0, x1 = 0.0, self.DOOR_HALF_WIDTH_MM
            # Keep every door edge within the body profile while maintaining
            # a narrow overlap over the 78 mm half-width bay opening.
            shell_x0 = min(abs(x0), shell_radius - 0.5)
            shell_x1 = min(abs(x1), shell_radius - 0.5)
            y0 = -math.sqrt(max(0.0, shell_radius ** 2 - shell_x0 ** 2))
            y1 = -math.sqrt(max(0.0, shell_radius ** 2 - shell_x1 ** 2))
            vertices = np.array([
                [x0, y0, z0], [x1, y1, z0],
                [x1, y1, z1], [x0, y0, z1]
            ], dtype=np.float32)
            faces = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.uint32)
            item = gl.GLMeshItem(
                meshdata=gl.MeshData(vertexes=vertices, faces=faces),
                smooth=False, drawFaces=True, drawEdges=False,
                shader="shaded", color=(0.95, 0.42, 0.12, 1.0))
            self.view.addItem(item)
            self.door_items.append(item)
            self.door_closed_vertices.append((side, vertices.copy(), faces))

    def update_from_packet(self, pkt):
        """Consume the existing telemetry packet; never opens a serial port."""
        if self.mode != "Live Sensor Data":
            return
        value = pkt.get(SIM_ALT_KEY)
        if value is None or not math.isfinite(float(value)):
            value = pkt.get(SIM_ALT_FALLBACK_KEY)
        if value is None:
            return
        try:
            altitude = max(0.0, float(value))
        except (TypeError, ValueError):
            return
        if not math.isfinite(altitude):
            return

        previous = self.previous_live_altitude
        self.current_altitude = altitude
        self.max_altitude = max(self.max_altitude, altitude)
        self.live_ascending = (previous is not None and altitude > previous + 0.02
                               and altitude < 1000.0)

        if previous is not None:
            delta = altitude - previous
            if delta < -0.25:
                self.live_descent_confirm += 1
            elif delta > 0.25:
                self.live_descent_confirm = 0
            if self.live_descent_confirm >= 2 and self.max_altitude > 800.0:
                self.live_descent_armed = True

            # Strict descending crossing: the ascent crossing is ignored.
            if (not self.ejection_triggered and self.live_descent_armed
                    and previous > 800.0 >= altitude):
                self._trigger_ejection(altitude)

        self.previous_live_altitude = altitude
        self._update_readouts(
            "CANSAT SEPARATION" if self.ejection_triggered else
            ("DESCENDING" if self.live_descent_armed else "LIVE TELEMETRY"))

    def _reset_flight_state(self):
        self.current_altitude = 0.0
        self.max_altitude = 0.0
        self.previous_live_altitude = None
        self.live_descent_confirm = 0
        self.live_descent_armed = False
        self.ejection_triggered = False
        self.separation_elapsed = -1.0
        self.separation_altitude = None
        self.cansat_altitude = 0.0
        self.cansat_x = 0.0
        self.cansat_y = 0.0
        self.rocket_landed = False
        self.cansat_landed = False
        self._apply_scene_transforms()

    def _mode_changed(self, mode):
        self.mode = mode
        self.auto_running = False
        self.auto_paused = False
        self.auto_elapsed = 0.0
        self._reset_flight_state()
        self._update_readouts(
            "AUTO READY" if mode == "Automatic Simulation"
            else "WAITING FOR TELEMETRY")

    def _start_auto(self):
        if self.mode != "Automatic Simulation":
            self.cmb_mode.setCurrentText("Automatic Simulation")
        if self.auto_elapsed >= 110.0 or self.ejection_triggered:
            self.auto_elapsed = 0.0
            self._reset_flight_state()
        self.auto_running = True
        self.auto_paused = False
        self.last_tick = time.monotonic()

    def _pause_auto(self):
        if self.mode == "Automatic Simulation":
            self.auto_paused = not self.auto_paused
            self.auto_running = not self.auto_paused

    def _reset_sim(self):
        self.auto_elapsed = 0.0
        self.auto_running = False
        self.auto_paused = False
        self._reset_flight_state()
        self._update_readouts("READY")

    def _automatic_state(self, t):
        # 1 km apogee followed by a 12 m/s descent.
        ignition = 1.0
        apogee_t = 24.0
        descent_start = apogee_t + 1.5
        if t < ignition:
            return 0.0, "LAUNCH"
        if t < apogee_t:
            u = (t - ignition) / (apogee_t - ignition)
            return 1000.0 * (1.0 - (1.0 - u) ** 2), (
                "POWERED ASCENT" if t < 8.0 else "COASTING")
        if t < descent_start:
            return 1000.0, "APOGEE"
        descent_t = t - descent_start
        altitude = max(0.0, 1000.0 - 12.0 * descent_t)
        return altitude, ("DESCENT" if altitude > 0.0 else "LANDED")

    def _trigger_ejection(self, rocket_altitude):
        if self.ejection_triggered:
            return
        self.ejection_triggered = True
        self.separation_elapsed = 0.0
        self.separation_altitude = float(rocket_altitude)
        self.cansat_altitude = float(rocket_altitude) + (
            self.BAY_BOTTOM_MM / self.ALTITUDE_TO_MODEL_UNITS)
        self.cansat_x = 0.0
        self.cansat_y = 0.0

    def _update_separation(self, dt):
        if not self.ejection_triggered:
            return
        self.separation_elapsed += dt
        u = min(1.0, self.separation_elapsed / self.EJECTION_DURATION_S)
        # Smoothstep outward translation through the opened mid-body bay.
        smooth = u * u * (3.0 - 2.0 * u)
        self.cansat_y = -self.EJECTION_DISTANCE_MM * smooth

        # Parallel descent is an AUTOMATIC-SIMULATION behaviour only.
        # In Live Sensor Data mode, use the altitude from the latest received
        # telemetry packet; do not invent an independent CanSat descent profile
        # or add a simulated vertical offset. This project currently receives
        # one flight-altitude value (bmp_alt, with dps_alt as fallback), so the
        # CanSat follows that received altitude until dedicated CanSat telemetry
        # is available.
        # Once the rocket reaches ground level, bring the CanSat all the way
        # to ground too. Do not leave it hovering at the automatic mode's
        # small vertical-following offset. In live mode, ground contact is
        # driven by the altitude value received from telemetry.
        if self.current_altitude <= 0.0:
            self.cansat_altitude = 0.0
            self.cansat_landed = True
        elif self.mode == "Automatic Simulation":
            self.cansat_altitude = max(
                0.0, self.current_altitude + self.CANSAT_VERTICAL_OFFSET_M)
            self.cansat_landed = False
        else:
            self.cansat_altitude = max(0.0, self.current_altitude)
            self.cansat_landed = self.cansat_altitude <= 0.0

    def _on_frame(self):
        now = time.monotonic()
        dt = min(max(now - self.last_tick, 0.0), 0.1)
        self.last_tick = now

        if self.mode == "Automatic Simulation" and self.auto_running:
            old_alt = self.current_altitude
            self.auto_elapsed += dt
            self.current_altitude, phase = self._automatic_state(
                self.auto_elapsed)
            self.max_altitude = max(self.max_altitude, self.current_altitude)

            # This event can only happen after apogee, on a descending crossing.
            if (not self.ejection_triggered and old_alt > 800.0 >=
                    self.current_altitude and self.auto_elapsed > 25.5):
                self._trigger_ejection(800.0)

            if self.current_altitude <= 0.0 and self.auto_elapsed > 25.5:
                self.rocket_landed = True
                self.auto_running = False
                phase = "LANDED"

            self._update_readouts(
                "CANSAT SEPARATION" if self.ejection_triggered and
                self.separation_elapsed < self.EJECTION_DURATION_S else phase)

        self._update_separation(dt)
        self._apply_scene_transforms()

    def _set_item_translation(self, item, x, y, z):
        item.resetTransform()
        item.translate(float(x), float(y), float(z))

    def _apply_door_animation(self):
        if not self.door_items:
            return
        angle = 0.0
        if self.ejection_triggered and self.separation_elapsed >= 0.0:
            angle = 82.0 * min(1.0, self.separation_elapsed / 0.8)

        rocket_z = self.current_altitude * self.ALTITUDE_TO_MODEL_UNITS
        for item, (side, closed, faces) in zip(
                self.door_items, self.door_closed_vertices):
            pivot_x = -self.DOOR_HALF_WIDTH_MM if side < 0 else self.DOOR_HALF_WIDTH_MM
            # Hinge at the actual curved shell surface, not at y=0; this
            # keeps the door edge physically attached instead of floating.
            pivot_y = -math.sqrt(max(0.0, 90.5 ** 2 - pivot_x ** 2))
            theta = math.radians(-angle if side < 0 else angle)
            c, s = math.cos(theta), math.sin(theta)
            verts = closed.astype(float).copy()
            dx = verts[:, 0] - pivot_x
            dy = verts[:, 1] - pivot_y
            verts[:, 0] = pivot_x + dx * c - dy * s
            verts[:, 1] = pivot_y + dx * s + dy * c
            verts[:, 2] += rocket_z
            item.setMeshData(meshdata=gl.MeshData(
                vertexes=verts.astype(np.float32), faces=faces))

    def _apply_scene_transforms(self):
        if self.view is None:
            return
        rocket_z = self.current_altitude * self.ALTITUDE_TO_MODEL_UNITS
        for item in self.mesh_items:
            self._set_item_translation(item, 0.0, 0.0, rocket_z)

        # Before ejection the actual CanSat STL is placed inside the rocket bay.
        if self.cansat_item is not None:
            if not self.ejection_triggered:
                self._set_item_translation(
                    self.cansat_item, 0.0, 0.0,
                    rocket_z + self.BAY_BOTTOM_MM)
            else:
                if self.separation_elapsed <= self.EJECTION_DURATION_S:
                    cansat_z = rocket_z + self.BAY_BOTTOM_MM
                else:
                    cansat_z = self.cansat_altitude * self.ALTITUDE_TO_MODEL_UNITS
                self._set_item_translation(
                    self.cansat_item, self.cansat_x, self.cansat_y, cansat_z)

        self._apply_door_animation()
        self._update_rocket_fire_effect(rocket_z)
        # NEW: parachutes deploy on the exact same frame as CanSat ejection.
        # The rocket canopy attaches to the rocket's upper body; the second
        # canopy attaches to the CanSat. Suspension lines are refreshed each frame.
        self._update_parachute_visuals(rocket_z)

        # Auto-track the rocket and CanSat throughout the entire flight. The
        # camera orientation and user zoom remain untouched, so orbiting and
        # zooming still work; uncheck the control to freeze the camera target.
        self._update_camera_tracking(rocket_z)
        self.view.update()

    def _update_camera_tracking(self, rocket_z):
        if self.view is None or not hasattr(self, "chk_track"):
            return
        if not self.chk_track.isChecked():
            return

        if self.cansat_item is None or not self.ejection_triggered:
            cansat_x, cansat_y = 0.0, 0.0
            cansat_z = rocket_z + self.BAY_CENTER_MM
        elif self.separation_elapsed <= self.EJECTION_DURATION_S:
            cansat_x, cansat_y = self.cansat_x, self.cansat_y
            cansat_z = rocket_z + self.BAY_CENTER_MM
        else:
            cansat_x, cansat_y = self.cansat_x, self.cansat_y
            cansat_z = self.cansat_altitude * self.ALTITUDE_TO_MODEL_UNITS

        # Centre between both craft, with a small upward bias to keep the
        # deployed canopies in frame. Keep the current distance and angles so
        # the camera continues to feel manually controllable.
        center_x = (0.0 + cansat_x) / 2.0
        center_y = (0.0 + cansat_y) / 2.0
        center_z = (rocket_z + cansat_z) / 2.0 + 350.0
        self.view.opts["center"] = QVector3D(
            float(center_x), float(center_y), float(center_z))

    def _create_rocket_fire_effect(self):
        """Create layered 3D exhaust flames anchored to the rocket's nozzle."""
        if self.view is None or not STL_3D_AVAILABLE:
            return
        self._remove_items(self.rocket_fire_items)
        # A cone's tip is normally above its base; flip it so the tip points down.
        for radius, height, color in (
                (105.0, 620.0, (1.0, 0.20, 0.025, 1.0)),
                (62.0, 430.0, (1.0, 0.78, 0.08, 1.0)),
                (28.0, 270.0, (1.0, 0.98, 0.70, 1.0))):
            mesh = trimesh.creation.cone(radius=radius, height=height, sections=20)
            flip = trimesh.transformations.rotation_matrix(math.pi, [1, 0, 0])
            mesh.apply_transform(flip)
            item = self._make_mesh_item(mesh, color)
            item.setVisible(False)
            self.view.addItem(item)
            self.rocket_fire_items.append(item)

    def _update_rocket_fire_effect(self, rocket_z):
        """Show a flickering exhaust while the rocket rises, stopping at 1,000 m."""
        if not self.rocket_fire_items:
            return
        if self.mode == "Automatic Simulation":
            # The automatic flight reaches its 1,000 m apogee at T+24 s.
            active = 1.0 <= self.auto_elapsed < 24.0 and self.current_altitude < 1000.0
        else:
            active = self.live_ascending and self.current_altitude < 1000.0
        for i, item in enumerate(self.rocket_fire_items):
            item.setVisible(active)
            if not active:
                continue
            # Small deterministic flicker gives the exhaust a lively flame shape.
            phase = time.monotonic() * (13.0 + i * 2.5) + i * 1.7
            flicker = 0.88 + 0.12 * math.sin(phase)
            item.resetTransform()
            item.translate(0.0, 0.0, float(rocket_z + self.rocket_bottom_mm))
            item.scale(flicker, flicker, flicker)

    def _make_parachute_canopy(self, radius=150.0, height=95.0, color=(1.0, 0.25, 0.18, 1.0)):
        """NEW: shallow domed canopy mesh (parametric hemisphere) for one chute."""
        rings, segments = 7, 24
        vertices = [[0.0, 0.0, height]]
        for ri in range(1, rings + 1):
            theta = (math.pi / 2.0) * ri / rings
            for j in range(segments):
                phi = 2.0 * math.pi * j / segments
                vertices.append([radius * math.sin(theta) * math.cos(phi),
                                 radius * math.sin(theta) * math.sin(phi),
                                 height * math.cos(theta)])
        faces = []
        for j in range(segments):
            faces.append([0, 1 + j, 1 + (j + 1) % segments])
        for ri in range(rings - 1):
            a0 = 1 + ri * segments
            b0 = a0 + segments
            for j in range(segments):
                jn = (j + 1) % segments
                faces.extend([[a0 + j, b0 + j, b0 + jn],
                              [a0 + j, b0 + jn, a0 + jn]])
        mesh = trimesh.Trimesh(vertices=np.asarray(vertices), faces=np.asarray(faces), process=False)
        return self._make_mesh_item(mesh, color)

    def _update_parachute_visuals(self, rocket_z):
        """NEW: animate two separate canopies + eight suspension lines each."""
        if self.view is None:
            return
        deployed = self.ejection_triggered
        # Before deployment, hide the canopy and lines without removing objects.
        for key in ("rocket", "cansat"):
            canopy = self.parachute_items.get(key)
            lines = self.parachute_line_items.get(key)
            if canopy is None:
                # Large canopies: 3x the previous radius (9x projected area),
                # giving both vehicles clearly visible parachutes in the 3D scene.
                canopy = self._make_parachute_canopy(
                    radius=525.0 if key == "rocket" else 315.0,
                    height=330.0 if key == "rocket" else 210.0,
                    color=(1.0, 0.28, 0.12, 1.0) if key == "rocket" else (1.0, 0.78, 0.12, 1.0))
                self.view.addItem(canopy)
                self.parachute_items[key] = canopy
            if lines is None:
                lines = gl.GLLinePlotItem(pos=np.zeros((2, 3), dtype=np.float32),
                                          color=(0.98, 0.98, 0.92, 1.0), width=2, antialias=True, mode="lines")
                self.view.addItem(lines)
                self.parachute_line_items[key] = lines
            # Hide each canopy and its suspension lines as soon as its vehicle
            # has landed, so the vehicle visibly rests on the ground without
            # parachute geometry floating above it.
            landed = self.rocket_landed if key == "rocket" else self.cansat_landed
            # The rocket chute may open at separation, but the CanSat chute must
            # remain packed/hidden until the CanSat has fully cleared the bay.
            # A short 0.25 s delay after the ejection animation prevents the
            # canopy from appearing inside or attached to the rocket.
            if key == "cansat":
                chute_deployed = (
                    deployed and
                    self.separation_elapsed >= self.EJECTION_DURATION_S + 0.25
                )
            else:
                chute_deployed = deployed
            visible = chute_deployed and not landed
            canopy.setVisible(visible)
            lines.setVisible(visible)
            if not visible:
                continue
            if key == "rocket":
                anchor = np.array([0.0, 0.0, rocket_z + self.BAY_TOP_MM + 180.0])
                canopy_center = anchor + np.array([0.0, 0.0, 1080.0])  # 50% longer than the previous 720 mm line length
                radius, height = 525.0, 330.0
            else:
                if self.separation_elapsed <= self.EJECTION_DURATION_S:
                    can_z = rocket_z + self.BAY_BOTTOM_MM
                    can_x, can_y = self.cansat_x, self.cansat_y
                else:
                    can_z = self.cansat_altitude * self.ALTITUDE_TO_MODEL_UNITS
                    can_x, can_y = self.cansat_x, self.cansat_y
                anchor = np.array([can_x, can_y, can_z + 100.0])
                canopy_center = anchor + np.array([0.0, 0.0, 690.0])  # 50% longer than the previous 460 mm line length
                radius, height = 315.0, 210.0
            canopy.resetTransform()
            canopy.translate(float(canopy_center[0]), float(canopy_center[1]), float(canopy_center[2]))
            # Eight evenly-spaced line pairs connect canopy rim securely to its craft.
            line_points = []
            for j in range(8):
                phi = 2.0 * math.pi * j / 8.0
                rim = canopy_center + np.array([radius * math.cos(phi), radius * math.sin(phi), 0.0])
                line_points.extend([rim, anchor])
            lines.setData(pos=np.asarray(line_points, dtype=np.float32), mode="lines")

    def _update_readouts(self, phase):
        self.lbl_alt.setText(f"Altitude: {self.current_altitude:.1f} m")
        self.lbl_apogee.setText(f"Apogee: {self.max_altitude:.1f} m")
        self.lbl_phase.setText(f"Phase: {phase}")
        if self.ejection_triggered:
            if self.cansat_landed:
                status = "CanSat: LANDED"
            elif self.separation_elapsed < self.EJECTION_DURATION_S:
                status = "CanSat: EJECTING"
            elif self.separation_elapsed < self.EJECTION_DURATION_S + 0.25:
                status = "CanSat: CLEAR OF ROCKET"
            else:
                status = "CanSat: PARACHUTE DEPLOYED / DESCENT"
        else:
            status = "CanSat: INSIDE"
        self.lbl_cansat.setText(status)


# ============================================================
# GROUND STATION GUI
# ============================================================

class GroundStationGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IN-SPACe 2026 — Zenith Orbitron Ground Station")
        self.resize(1500, 950)

        self.hist = {k: deque(maxlen=300) for k in FIELD_KEYS}
        self.hist["t"] = deque(maxlen=300)

        self.packet_count = 0
        self.total_packet_bytes = 0
        self.last_packet_size = 0
        self.last_packet_wall = None
        self.start_wall = time.time()
        self.max_alt = float("-inf")
        self.max_accel = 0.0

        # map state
        self.map_marker_lat = None
        self.map_marker_lon = None
        self.map_view = None
        self.map_fullscreen_dialog = None
        self.last_map_render = 0.0

        # CSV log
        self.csv_filename = f"telemetry_{int(time.time())}.csv"
        self.csv_file = open(self.csv_filename, "w", newline="", encoding="utf-8")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "Wall_Time", "Time_MS",
            "BMP581_Alt_m", "BMP581_Temp_C", "BMP581_Press_hPa",
            "DPS310_Alt_m", "DPS310_Temp_C", "DPS310_Press_hPa",
            "Ax", "Ay", "Az", "Gx", "Gy", "Gz",
            "Roll_deg", "Pitch_deg", "Yaw_deg",
            "Bus_V", "Current_mA", "Power_mW",
            "Accel_Mag", "Lat", "Lon", "Packet_Size_B",
        ])

        self.thread = None
        self.init_ui()

        # link watchdog
        from PyQt6.QtCore import QTimer
        self.watchdog = QTimer(self)
        self.watchdog.timeout.connect(self._check_link)
        self.watchdog.start(500)

    # ---------------------------------------------------------
    # UI
    # ---------------------------------------------------------
    def init_ui(self):
        central = QWidget()
        main = QVBoxLayout(central)

        # NEW: the original dashboard becomes tab 1 of a QTabWidget.
        # Nothing inside `central` / `main` has been altered.
        self.tabs = QTabWidget()
        self.tabs.setStyleSheet(
            "QTabWidget::pane { border: 1px solid #383838; }"
            "QTabBar::tab { background-color: #1B1B1B; border: 1px solid #383838;"
            " padding: 6px 18px; }"
            "QTabBar::tab:selected { background-color: #272727; color: #00FFCC; }"
            "QTabBar::tab:hover { background-color: #272727; }")
        self.tabs.addTab(central, "Telemetry Dashboard")
        self.setCentralWidget(self.tabs)

        # ---- top bar ----
        top = QHBoxLayout()
        self.port_selector = QComboBox()
        self.baud_selector = QComboBox()
        self.baud_selector.addItems(BAUD_CHOICES)
        self.baud_selector.setCurrentText(DEFAULT_BAUD)
        btn_refresh = QPushButton("Refresh Ports")
        btn_refresh.clicked.connect(self.refresh_ports)
        self.btn_connect = QPushButton("Connect Serial")
        self.btn_connect.clicked.connect(self.toggle_connection)
        self.status_label = QLabel("Status: DISCONNECTED")
        self.status_label.setStyleSheet("color: #FF3333; font-weight: bold;")
        self.link_label = QLabel("LoRa link: --")
        self.link_label.setStyleSheet("color: #888888; font-weight: bold;")

        top.addWidget(QLabel("COM Port:"))
        top.addWidget(self.port_selector)
        top.addWidget(QLabel("Baud:"))
        top.addWidget(self.baud_selector)
        top.addWidget(btn_refresh)
        top.addWidget(self.btn_connect)
        top.addWidget(self.status_label)
        top.addWidget(self.link_label)
        top.addStretch()
        main.addLayout(top)

        # ---- metrics row ----
        metrics = QHBoxLayout()

        def box(title, labels_grid):
            gb = QGroupBox(title)
            g = QGridLayout(gb)
            for lbl, r, c in labels_grid:
                g.addWidget(lbl, r, c)
            return gb

        self.lbl_bmp = QLabel("Alt 0.00 m | 0.00 °C | 0.00 hPa")
        self.lbl_bmp.setStyleSheet("color: #00FFCC; font-weight: bold;")
        self.lbl_dps = QLabel("Alt 0.00 m | 0.00 °C | 0.00 hPa")
        self.lbl_dps.setStyleSheet("color: #FF8A00; font-weight: bold;")
        self.lbl_alt_diff = QLabel("ΔAlt (BMP−DPS): 0.00 m")
        self.lbl_max_alt = QLabel("Max Alt: 0.00 m")
        metrics.addWidget(box("BAROMETERS  (BMP581 / DPS310)", [
            (QLabel("BMP581:"), 0, 0), (self.lbl_bmp, 0, 1),
            (QLabel("DPS310:"), 1, 0), (self.lbl_dps, 1, 1),
            (self.lbl_alt_diff, 2, 0), (self.lbl_max_alt, 2, 1)]))

        self.lbl_accel = QLabel("Accel: X 0.00 | Y 0.00 | Z 0.00 m/s²")
        self.lbl_gyro = QLabel("Gyro: X 0.00 | Y 0.00 | Z 0.00 °/s")
        self.lbl_euler = QLabel("Roll +0.0° | Pitch +0.0° | Yaw +0.0°")
        self.lbl_accel_mag = QLabel("|a|: 0.00 m/s² (max 0.00)")
        metrics.addWidget(box("BNO085 IMU", [
            (self.lbl_accel, 0, 0), (self.lbl_gyro, 1, 0),
            (self.lbl_euler, 2, 0), (self.lbl_accel_mag, 3, 0)]))

        self.lbl_vbus = QLabel("Bus: 0.00 V")
        self.lbl_vbus.setStyleSheet(
            "color: #B388FF; font-size: 13pt; font-weight: bold;")
        self.lbl_cur = QLabel("Current: 0.0 mA")
        self.lbl_pwr = QLabel("Power: 0.0 mW")
        metrics.addWidget(box("INA219 POWER", [
            (self.lbl_vbus, 0, 0), (self.lbl_cur, 1, 0),
            (self.lbl_pwr, 2, 0)]))

        self.lbl_packet_count = QLabel("Packets: 0")
        self.lbl_packet_rate = QLabel("Rate: 0.00 Hz")
        self.lbl_last_packet = QLabel("Last Packet: --")
        self.lbl_time = QLabel("Flight Time: 0.00 s")
        self.lbl_packet_size = QLabel("Last Size: 0 B")
        self.lbl_avg_size = QLabel("Average Size: 0.0 B")
        metrics.addWidget(box("LoRa TELEMETRY LINK", [
            (self.lbl_packet_count, 0, 0), (self.lbl_packet_rate, 0, 1),
            (self.lbl_last_packet, 1, 0), (self.lbl_time, 1, 1),
            (self.lbl_packet_size, 2, 0), (self.lbl_avg_size, 2, 1)]))
        main.addLayout(metrics)

        # ---- sidebar + graph grid ----
        pg.setConfigOption("background", "#121212")
        pg.setConfigOption("foreground", "#FFFFFF")

        row = QHBoxLayout()

        side = QWidget()
        side.setMaximumWidth(300)
        side_l = QVBoxLayout(side)
        side_l.setSpacing(6)

        gb_att = QGroupBox("FLIGHT ATTITUDE INDICATOR")
        l_att = QVBoxLayout(gb_att)
        self.attitude_indicator = AttitudeIndicator()
        self.attitude_indicator.setFixedHeight(190)
        l_att.addWidget(self.attitude_indicator)
        side_l.addWidget(gb_att)

        gb_map = QGroupBox("PAYLOAD LOCATION")
        l_map = QVBoxLayout(gb_map)
        map_widget = self._build_map_widget()
        map_widget.setFixedHeight(190)
        l_map.addWidget(map_widget)
        side_l.addWidget(gb_map)
        side_l.addStretch()
        row.addWidget(side)

        graphs = QWidget()
        grid = QGridLayout(graphs)
        grid.setSpacing(6)
        grid.setContentsMargins(0, 0, 0, 0)

        self.plots = {}
        self.curves = {}   # plot id -> [(key, curve)]
        for n, (pid, title, ylabel, units, series) in enumerate(PLOT_SPECS):
            pw = pg.PlotWidget(title=title)
            pw.setLabel("left", ylabel, units=units)
            pw.setLabel("bottom", "Flight Time", units="s")
            pw.showGrid(x=True, y=True, alpha=0.25)
            pw.setBackground("#121212")
            pw.setMinimumHeight(150)
            pw.getPlotItem().getAxis("left").setWidth(70)
            if len(series) > 1:
                pw.addLegend()
            pw.setXRange(0, 10, padding=0)
            pw.setYRange(0, 10, padding=0)
            self.plots[pid] = pw
            self.curves[pid] = []
            for key, color, name in series:
                curve = pw.plot(pen=pg.mkPen(color=color, width=2), name=name)
                self.curves[pid].append((key, curve))
            gb = QGroupBox(title.upper())
            QVBoxLayout(gb).addWidget(pw)
            grid.addWidget(gb, n // 4, n % 4)
        for i in range(4):
            grid.setColumnStretch(i, 1)
        grid.setRowStretch(0, 1)
        grid.setRowStretch(1, 1)
        row.addWidget(graphs, 1)
        main.addLayout(row, 1)

        # ---- console ----
        self.console = QTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumHeight(160)
        self.console.document().setMaximumBlockCount(1500)
        self.console.setStyleSheet(
            "background-color:#000000; color:#00FF00;"
            "font-family:Consolas; font-size:10pt;")
        main.addWidget(self.console)

        self.console.append(f"[SYSTEM] {VERSION}")
        self.console.append(
            "[SYSTEM] CAN,time,bmpAlt,bmpT,bmpP,dpsAlt,dpsT,dpsP,"
            "ax,ay,az,gx,gy,gz,roll,pitch,yaw,Vbus,mA,mW[,lat,lon]")
        self.console.append(f"[SYSTEM] Logging to {self.csv_filename}")
        self.console.append("[SYSTEM] Select COM port and press Connect Serial.")
        self._init_extra_tabs()   # NEW: adds tab 2 (Live Map) + tab 3 (Simulation)
        self.refresh_ports()

    # ---------------------------------------------------------
    # NEW — EXTRA TABS
    # ---------------------------------------------------------
    def _init_extra_tabs(self):
        self.map_tab = LiveMapTab()                 # Existing map tab
        self.sim_tab = RocketSimulationTab()        # Existing 2D simulation tab
        self.sim3d_tab = Rocket3DSimulationTab()     # NEW TAB 4: STL-based 3D view
        self.tabs.addTab(self.map_tab, "Live Map")
        self.tabs.addTab(self.sim_tab, "2D Flight Simulation")
        self.tabs.addTab(self.sim3d_tab, "3D Rocket Simulation")
        self.console.append(
            "[SYSTEM] Existing Live Map + 2D Flight Simulation tabs initialized; "
            "new STL-based 3D Rocket Simulation tab added as tab 4.")

    def _feed_new_tabs(self, pkt):
        """Forward the same parsed packet to the map and both simulations.
        Each tab is isolated so its errors cannot affect serial/telemetry logic."""
        try:
            self.map_tab.update_from_packet(pkt)
        except Exception as e:
            self.console.append(f"[MAP TAB ERROR] {type(e).__name__}: {e}")
        try:
            self.sim_tab.update_from_packet(pkt)
        except Exception as e:
            self.console.append(f"[SIM TAB ERROR] {type(e).__name__}: {e}")
        try:
            self.sim3d_tab.update_from_packet(pkt)
        except Exception as e:
            self.console.append(f"[3D SIM TAB ERROR] {type(e).__name__}: {e}")

    # ---------------------------------------------------------
    # MAP
    # ---------------------------------------------------------
    def _build_map_widget(self):
        container = QWidget()
        stack = QStackedLayout(container)
        stack.setStackingMode(QStackedLayout.StackingMode.StackAll)
        stack.setContentsMargins(0, 0, 0, 0)

        if MAP_SUPPORT_AVAILABLE:
            self.map_view = QWebEngineView()
            self._render_map(self.map_view)
            stack.addWidget(self.map_view)
        else:
            fb = QLabel("Map unavailable —\ninstall with:\n"
                        "pip install folium PyQt6-WebEngine")
            fb.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fb.setStyleSheet("color: #FF8A00;")
            stack.addWidget(fb)

        overlay = ClickableMapOverlay()
        overlay.clicked.connect(self._open_map_fullscreen)
        stack.addWidget(overlay)
        return container

    def _build_folium_html(self):
        lat = self.map_marker_lat
        lon = self.map_marker_lon
        fmap = folium.Map(
            location=[lat if lat is not None else MAP_DEFAULT_LAT,
                      lon if lon is not None else MAP_DEFAULT_LON],
            zoom_start=15, tiles="OpenStreetMap")
        if lat is not None:
            folium.Marker(
                [lat, lon], tooltip="Payload location",
                icon=folium.Icon(color="red", icon="rocket", prefix="fa")
            ).add_to(fmap)
        return fmap.get_root().render()

    def _render_map(self, view):
        view.setHtml(self._build_folium_html())

    def _open_map_fullscreen(self):
        if not MAP_SUPPORT_AVAILABLE:
            QMessageBox.information(
                self, "Map Unavailable",
                "pip install folium PyQt6-WebEngine")
            return
        if self.map_marker_lat is None:
            QMessageBox.information(
                self, "GPS Location Unavailable",
                "The locator appears once valid latitude/longitude "
                "arrive in the telemetry.")
            return
        d = self.map_fullscreen_dialog
        if d is not None and d.isVisible():
            d.raise_()
            d.activateWindow()
            return
        self.map_fullscreen_dialog = MapFullscreenDialog(
            self._build_folium_html(), parent=self)
        self.map_fullscreen_dialog.finished.connect(
            lambda _=None: setattr(self, "map_fullscreen_dialog", None))

    def _update_map_location(self, lat, lon):
        if not MAP_SUPPORT_AVAILABLE:
            return
        self.map_marker_lat, self.map_marker_lon = lat, lon
        now = time.time()
        if now - self.last_map_render < MAP_REFRESH_S:
            return
        self.last_map_render = now
        if self.map_view is not None:
            self._render_map(self.map_view)
        d = self.map_fullscreen_dialog
        if d is not None and d.isVisible():
            d.view.setHtml(self._build_folium_html())

    # ---------------------------------------------------------
    # PORTS / CONNECTION
    # ---------------------------------------------------------
    def refresh_ports(self):
        current = self.port_selector.currentText()
        self.port_selector.clear()
        ports = serial.tools.list_ports.comports()
        if not ports:
            self.port_selector.addItem("NO SERIAL PORTS FOUND")
            self.console.append("[PORT] No serial ports detected.")
            return
        for p in ports:
            self.port_selector.addItem(p.device)
        i = self.port_selector.findText(current)
        if i >= 0:
            self.port_selector.setCurrentIndex(i)
        self.console.append("[PORT] Available serial ports:")
        for p in ports:
            self.console.append(f"    {p.device} — {p.description or 'Unknown device'}")

    def toggle_connection(self):
        if self.thread is not None and self.thread.isRunning():
            self.console.append("[SERIAL] Disconnecting...")
            self.thread.stop()
            self.thread = None
            self._set_status("DISCONNECTED", "#FF3333")
            self.btn_connect.setText("Connect Serial")
            return

        port = self.port_selector.currentText()
        if not port or port == "NO SERIAL PORTS FOUND":
            QMessageBox.warning(self, "No Serial Port",
                                "No valid serial port is selected.")
            return
        baud = int(self.baud_selector.currentText())

        # reset stats + history
        self.packet_count = 0
        self.total_packet_bytes = 0
        self.last_packet_size = 0
        self.last_packet_wall = None
        self.start_wall = time.time()
        self.max_alt = float("-inf")
        self.max_accel = 0.0
        for d in self.hist.values():
            d.clear()
        for lst in self.curves.values():
            for _, c in lst:
                c.setData([], [])
        self.attitude_indicator.set_attitude(0, 0, 0)

        self.console.append(f"[SERIAL] Connecting to {port}...")
        self.thread = SerialReaderThread(port, baud)
        self.thread.telemetry_packet_signal.connect(self.handle_telemetry_packet)
        self.thread.raw_console_signal.connect(self.console.append)
        self.thread.connection_error_signal.connect(self.handle_connection_error)
        self.thread.connection_status_signal.connect(self.handle_connection_status)
        self.thread.start()

        self.btn_connect.setText("Disconnect")
        self._set_status(f"CONNECTING ({port})", "#FFFF00")

    def _set_status(self, text, color):
        self.status_label.setText(f"Status: {text}")
        self.status_label.setStyleSheet(f"color: {color}; font-weight: bold;")

    def handle_connection_status(self, status):
        self.console.append(f"[STATUS] {status}")
        if status.startswith("CONNECTED"):
            self._set_status(status, "#00FFCC")
        elif status == "DISCONNECTED":
            self._set_status("DISCONNECTED", "#FF3333")
            self.btn_connect.setText("Connect Serial")

    def handle_connection_error(self, message):
        self.console.append(f"[ERROR] {message}")
        self._set_status("SERIAL ERROR", "#FF3333")
        self.btn_connect.setText("Connect Serial")
        QMessageBox.critical(self, "Serial Connection Error", message)

    def _check_link(self):
        if self.last_packet_wall is None:
            return
        age = time.time() - self.last_packet_wall
        if age > LINK_LOST_TIMEOUT_S:
            self.link_label.setText(f"LoRa link: NO SIGNAL ({age:.0f}s)")
            self.link_label.setStyleSheet("color: #FF3333; font-weight: bold;")
        else:
            self.link_label.setText("LoRa link: OK")
            self.link_label.setStyleSheet("color: #00FFCC; font-weight: bold;")

    # ---------------------------------------------------------
    # PLOT HELPER
    # ---------------------------------------------------------
    @staticmethod
    def _autoscale(plot, x, series_list):
        if len(x) == 1:
            plot.setXRange(x[0] - 5, x[0] + 5, padding=0)
        elif len(x) >= 2:
            xmin, xmax = min(x), max(x)
            if xmax <= xmin:
                xmax = xmin + 1
            pad = max((xmax - xmin) * 0.05, 0.5)
            plot.setXRange(xmin - pad, xmax + pad, padding=0)

        ys = [v for s in series_list for v in s]
        if ys:
            lo, hi = min(ys), max(ys)
            if hi - lo < 1.0:
                c = (lo + hi) / 2
                lo, hi = c - 1.0, c + 1.0
            else:
                pad = (hi - lo) * 0.15
                lo, hi = lo - pad, hi + pad
            plot.setYRange(lo, hi, padding=0)

    # ---------------------------------------------------------
    # TELEMETRY HANDLER
    # ---------------------------------------------------------
    def handle_telemetry_packet(self, pkt, packet_size):
        try:
            time_ms = int(pkt["time_ms"])
            t = time_ms / 1000.0

            if "latitude" in pkt:
                self._update_map_location(pkt["latitude"], pkt["longitude"])

            # BNO085 already fuses orientation — use it directly
            self.attitude_indicator.set_attitude(
                pkt["pitch"], pkt["roll"], pkt["yaw"])

            # history
            self.hist["t"].append(t)
            for k in FIELD_KEYS[1:]:
                self.hist[k].append(pkt[k])

            x = list(self.hist["t"])
            for pid, lst in self.curves.items():
                for key, curve in lst:
                    curve.setData(x, list(self.hist[key]))
                self._autoscale(self.plots[pid], x,
                                [self.hist[k] for k, _ in lst])

            # stats
            self.packet_count += 1
            self.last_packet_wall = time.time()
            self.last_packet_size = packet_size
            self.total_packet_bytes += packet_size
            avg = self.total_packet_bytes / self.packet_count
            elapsed = time.time() - self.start_wall
            rate = self.packet_count / elapsed if elapsed > 0 else 0.0

            self.max_alt = max(self.max_alt, pkt["bmp_alt"], pkt["dps_alt"])
            self.max_accel = max(self.max_accel, pkt["accel_mag"])

            # labels
            self.lbl_bmp.setText(
                f"Alt {pkt['bmp_alt']:.2f} m | {pkt['bmp_temp']:.2f} °C | "
                f"{pkt['bmp_press']:.2f} hPa")
            self.lbl_dps.setText(
                f"Alt {pkt['dps_alt']:.2f} m | {pkt['dps_temp']:.2f} °C | "
                f"{pkt['dps_press']:.2f} hPa")
            self.lbl_alt_diff.setText(f"ΔAlt (BMP−DPS): {pkt['alt_diff']:+.2f} m")
            self.lbl_max_alt.setText(f"Max Alt: {self.max_alt:.2f} m")

            self.lbl_accel.setText(
                f"Accel: X {pkt['ax']:.2f} | Y {pkt['ay']:.2f} | "
                f"Z {pkt['az']:.2f} m/s²")
            self.lbl_gyro.setText(
                f"Gyro: X {pkt['gx']:.2f} | Y {pkt['gy']:.2f} | "
                f"Z {pkt['gz']:.2f} °/s")
            self.lbl_euler.setText(
                f"Roll {pkt['roll']:+.1f}° | Pitch {pkt['pitch']:+.1f}° | "
                f"Yaw {pkt['yaw']:+.1f}°")
            self.lbl_accel_mag.setText(
                f"|a|: {pkt['accel_mag']:.2f} m/s² (max {self.max_accel:.2f})")

            low = pkt["vbus"] < LOW_VOLTAGE_WARN
            self.lbl_vbus.setText(
                f"Bus: {pkt['vbus']:.2f} V" + ("  ⚠ LOW" if low else ""))
            self.lbl_vbus.setStyleSheet(
                f"color: {'#FF3333' if low else '#B388FF'};"
                "font-size: 13pt; font-weight: bold;")
            self.lbl_cur.setText(f"Current: {pkt['cur']:.1f} mA")
            self.lbl_pwr.setText(f"Power: {pkt['pwr']:.1f} mW")

            self.lbl_packet_count.setText(f"Packets: {self.packet_count}")
            self.lbl_packet_rate.setText(f"Rate: {rate:.2f} Hz")
            self.lbl_last_packet.setText(
                f"Last Packet: {time.strftime('%H:%M:%S')}")
            self.lbl_time.setText(f"Flight Time: {t:.2f} s")
            self.lbl_packet_size.setText(f"Last Size: {packet_size} B")
            self.lbl_avg_size.setText(f"Average Size: {avg:.1f} B")

            # CSV
            self.csv_writer.writerow(
                [time.strftime("%H:%M:%S"), time_ms]
                + [pkt[k] for k in FIELD_KEYS[1:]]
                + [pkt["accel_mag"],
                   pkt.get("latitude", ""), pkt.get("longitude", ""),
                   packet_size])
            self.csv_file.flush()

            self.console.append(
                f"[TELEMETRY] T={t:.2f}s | BMP={pkt['bmp_alt']:.1f}m | "
                f"DPS={pkt['dps_alt']:.1f}m | R={pkt['roll']:+.0f}° "
                f"P={pkt['pitch']:+.0f}° | {pkt['vbus']:.2f}V | {packet_size}B")

            # NEW: forward the same parsed packet to tab 2 (map) and tab 3 (sim)
            self._feed_new_tabs(pkt)

        except Exception as e:
            self.console.append(
                f"[TELEMETRY UI ERROR] {type(e).__name__}: {e}")

    # ---------------------------------------------------------
    def closeEvent(self, event):
        if self.thread is not None and self.thread.isRunning():
            self.thread.stop()
            self.thread = None
        try:
            self.csv_file.close()
        except Exception:
            pass
        event.accept()


# ============================================================
# ENTRY POINT
# ============================================================

if __name__ == "__main__":
    # Keep Qt Quick's RHI backend compatible with the OpenGL-based 3D view
    # (pyqtgraph.opengl). This must be set before QApplication is created.
    os.environ.setdefault("QSG_RHI_BACKEND", "opengl")

    print("=" * 70)
    print(VERSION)
    print("RUNNING FILE:", __file__)
    print("=" * 70)

    app = QApplication(sys.argv)
    app.setStyleSheet("""
        QWidget { background-color: #0B0B0B; color: #E6E6E6; }
        QMainWindow { background-color: #050505; }
        QGroupBox {
            background-color: #111111; border: 1px solid #383838;
            border-radius: 4px; margin-top: 10px;
            padding: 10px 6px 6px 6px; font-weight: bold;
        }
        QGroupBox::title {
            subcontrol-origin: margin; left: 10px;
            padding: 0 4px; color: #BDBDBD;
        }
        QPushButton, QComboBox {
            background-color: #1B1B1B; border: 1px solid #383838; padding: 5px;
        }
        QPushButton:hover { background-color: #272727; }
    """)
    window = GroundStationGUI()
    window.show()
    sys.exit(app.exec())