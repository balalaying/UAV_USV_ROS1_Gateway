"""Thread-safe latest-frame model and PyQtGraph view for SAN60 spectra."""

import math
import threading
import time

import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QGridLayout, QLabel, QVBoxLayout, QWidget
import pyqtgraph as pg


REQUIRED_FIELDS = (
    'start_hz',
    'stop_hz',
    'bin_hz',
    'rbw_hz',
    'temperature_c',
    'peak_hz',
    'peak_dbm',
    'powers_dbm',
    'sequence',
)


def validate_spectrum_frame(frame):
    """Validate one decoded SAN60 JSON object and normalize numeric values."""
    if not isinstance(frame, dict):
        raise ValueError('SAN60 payload must be a JSON object')
    if frame.get('type') != 'spectrum':
        raise ValueError('SAN60 payload type must be spectrum')
    missing = [name for name in REQUIRED_FIELDS if name not in frame]
    if missing:
        raise ValueError('SAN60 payload missing fields: %s' % ', '.join(missing))

    values = {
        name: float(frame[name])
        for name in (
            'start_hz',
            'stop_hz',
            'bin_hz',
            'rbw_hz',
            'temperature_c',
            'peak_hz',
            'peak_dbm',
        )
    }
    if not all(math.isfinite(value) for value in values.values()):
        raise ValueError('SAN60 scalar fields must be finite')
    if values['start_hz'] < 0.0:
        raise ValueError('SAN60 start_hz must not be negative')
    if values['stop_hz'] < values['start_hz']:
        raise ValueError('SAN60 stop_hz must not be lower than start_hz')
    if values['bin_hz'] <= 0.0:
        raise ValueError('SAN60 bin_hz must be positive')

    sequence = frame['sequence']
    if isinstance(sequence, bool) or not isinstance(sequence, int):
        raise ValueError('SAN60 sequence must be an integer')
    if sequence < 0:
        raise ValueError('SAN60 sequence must not be negative')

    powers = np.asarray(frame['powers_dbm'], dtype=np.float32)
    if powers.ndim != 1 or powers.size == 0:
        raise ValueError('SAN60 powers_dbm must be a non-empty array')
    if not np.isfinite(powers).all():
        raise ValueError('SAN60 powers_dbm contains a non-finite value')

    values['powers_dbm'] = powers
    values['sequence'] = sequence
    return values


class San60SpectrumModel:
    """Latest-only spectrum cache shared by rospy and the Qt GUI thread."""

    def __init__(self):
        self._lock = threading.Lock()
        self._generation = 0
        self._start_hz = 0.0
        self._stop_hz = 0.0
        self._bin_hz = 0.0
        self._rbw_hz = 0.0
        self._temperature_c = 0.0
        self._peak_hz = 0.0
        self._peak_dbm = 0.0
        self._powers_dbm = np.empty((0,), dtype=np.float32)
        self._sequence = None
        self._received_monotonic = 0.0
        self._last_sequence = None
        self._dropped_frames = 0
        self._out_of_order_frames = 0

    def update(self, frame):
        powers = np.asarray(frame['powers_dbm'], dtype=np.float32).reshape((-1,))
        powers = powers.copy()
        powers.setflags(write=False)
        sequence = int(frame['sequence'])
        received_monotonic = time.monotonic()

        with self._lock:
            if self._last_sequence is not None:
                if sequence > self._last_sequence + 1:
                    self._dropped_frames += sequence - self._last_sequence - 1
                elif sequence <= self._last_sequence:
                    self._out_of_order_frames += 1
            self._last_sequence = sequence
            self._start_hz = float(frame['start_hz'])
            self._stop_hz = float(frame['stop_hz'])
            self._bin_hz = float(frame['bin_hz'])
            self._rbw_hz = float(frame['rbw_hz'])
            self._temperature_c = float(frame['temperature_c'])
            self._peak_hz = float(frame['peak_hz'])
            self._peak_dbm = float(frame['peak_dbm'])
            self._powers_dbm = powers
            self._sequence = sequence
            self._received_monotonic = received_monotonic
            self._generation += 1

    def snapshot(self):
        with self._lock:
            return {
                'generation': self._generation,
                'start_hz': self._start_hz,
                'stop_hz': self._stop_hz,
                'bin_hz': self._bin_hz,
                'rbw_hz': self._rbw_hz,
                'temperature_c': self._temperature_c,
                'peak_hz': self._peak_hz,
                'peak_dbm': self._peak_dbm,
                'powers_dbm': self._powers_dbm,
                'sequence': self._sequence,
                'received_monotonic': self._received_monotonic,
                'last_sequence': self._last_sequence,
                'dropped_frames': self._dropped_frames,
                'out_of_order_frames': self._out_of_order_frames,
            }


class San60SpectrumWidget(QWidget):
    """Latest SAN60 trace with a 20 Hz GUI refresh rate."""

    ONLINE_THRESHOLD_SECONDS = 1.0
    OFFLINE_THRESHOLD_SECONDS = 1.5

    def __init__(self, model, parent=None):
        super().__init__(parent)
        self.model = model
        self._last_generation = -1
        self._frequency_key = None
        self._frequency_mhz = np.empty((0,), dtype=np.float64)
        self._online = None

        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(6)
        status_layout = QGridLayout()
        status_layout.setHorizontalSpacing(8)
        status_layout.setVerticalSpacing(4)

        self.value_labels = {}
        fields = (
            ('SAN60', 'status', 'OFFLINE'),
            ('温度', 'temperature', '--'),
            ('峰值频率', 'peak_frequency', '--'),
            ('峰值功率', 'peak_power', '--'),
            ('起始频率', 'start_frequency', '--'),
            ('终止频率', 'stop_frequency', '--'),
            ('RBW', 'rbw', '--'),
            ('Sequence', 'sequence', '--'),
            ('帧统计', 'frame_stats', '丢帧 0 / 乱序 0'),
        )
        for index, (title, key, initial) in enumerate(fields):
            row = index // 3
            column = (index % 3) * 2
            name_label = QLabel(title)
            name_label.setStyleSheet('color: #526873;')
            value_label = QLabel(initial)
            value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            value_label.setStyleSheet('font-weight: 600; color: #17242c;')
            status_layout.addWidget(name_label, row, column)
            status_layout.addWidget(value_label, row, column + 1)
            self.value_labels[key] = value_label
        status_layout.setColumnStretch(1, 1)
        status_layout.setColumnStretch(3, 1)
        status_layout.setColumnStretch(5, 1)
        layout.addLayout(status_layout)

        self.plot = pg.PlotWidget(background='#0a1118')
        self.plot.setLabel('bottom', 'Frequency', units='MHz')
        self.plot.setLabel('left', 'Power', units='dBm')
        self.plot.showGrid(x=True, y=True, alpha=0.25)
        self.plot.getPlotItem().setMenuEnabled(False)
        self.plot.getPlotItem().hideButtons()
        self.curve = self.plot.plot(
            pen=pg.mkPen('#22d3ee', width=1.5),
            connect='finite',
        )
        layout.addWidget(self.plot, 1)
        self.setMinimumHeight(280)
        self._set_online(False)

        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.refresh)
        self.timer.start()

    def _set_online(self, online):
        online = bool(online)
        if self._online == online and self.value_labels['status'].text() in (
            'ONLINE', 'OFFLINE'
        ):
            return
        self._online = online
        label = self.value_labels['status']
        label.setText('ONLINE' if online else 'OFFLINE')
        label.setStyleSheet(
            ('font-weight: 700; color: #176b47;' if online else
             'font-weight: 700; color: #a23b2a;')
        )

    @staticmethod
    def _frequency_text(value_hz):
        return '%.3f MHz' % (float(value_hz) / 1e6)

    @staticmethod
    def _rbw_text(value_hz):
        value_hz = float(value_hz)
        if abs(value_hz) >= 1e6:
            return '%.3f MHz' % (value_hz / 1e6)
        return '%.1f kHz' % (value_hz / 1e3)

    def refresh(self):
        snapshot = self.model.snapshot()
        received_at = snapshot['received_monotonic']
        age = (
            time.monotonic() - received_at
            if received_at > 0.0 else float('inf')
        )
        if self._online:
            self._set_online(age <= self.OFFLINE_THRESHOLD_SECONDS)
        else:
            self._set_online(age < self.ONLINE_THRESHOLD_SECONDS)

        generation = snapshot['generation']
        if generation == self._last_generation or not len(
            snapshot['powers_dbm']
        ):
            return

        powers = snapshot['powers_dbm']
        frequency_key = (
            snapshot['start_hz'], snapshot['bin_hz'], len(powers)
        )
        if frequency_key != self._frequency_key:
            self._frequency_mhz = (
                snapshot['start_hz']
                + np.arange(len(powers), dtype=np.float64)
                * snapshot['bin_hz']
            ) / 1e6
            self._frequency_key = frequency_key

        self.curve.setData(self._frequency_mhz, powers)
        self.value_labels['temperature'].setText(
            '%.1f °C' % snapshot['temperature_c']
        )
        self.value_labels['peak_frequency'].setText(
            self._frequency_text(snapshot['peak_hz'])
        )
        self.value_labels['peak_power'].setText(
            '%.1f dBm' % snapshot['peak_dbm']
        )
        self.value_labels['start_frequency'].setText(
            self._frequency_text(snapshot['start_hz'])
        )
        self.value_labels['stop_frequency'].setText(
            self._frequency_text(snapshot['stop_hz'])
        )
        self.value_labels['rbw'].setText(self._rbw_text(snapshot['rbw_hz']))
        self.value_labels['sequence'].setText(str(snapshot['sequence']))
        self.value_labels['frame_stats'].setText(
            '丢帧 %d / 乱序 %d'
            % (
                snapshot['dropped_frames'],
                snapshot['out_of_order_frames'],
            )
        )
        self._last_generation = generation
