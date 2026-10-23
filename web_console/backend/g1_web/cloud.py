"""Bounded XY projection of mapping PointCloud2 data; no ROS imports or navigation use."""
import math
import struct
import threading
import time


class MappingPreview:
    def __init__(self):
        self.lock = threading.Lock()
        self.reset()

    def reset(self):
        with self.lock:
            self.cells = {}
            self.frame = ''
            self.received_at = self.last_sample = 0.0
            self.scans = self.input_points = self.revision = 0
            self.cached = None

    def add(self, message):
        now = time.monotonic()
        if now - self.last_sample < 0.8:
            return
        fields = {field.name: field for field in message.fields}
        if not all(name in fields and fields[name].datatype in (7, 8) for name in ('x', 'y', 'z')):
            return
        endian = '>' if message.is_bigendian else '<'
        codecs = [(fields[name].offset, struct.Struct(endian + ('f' if fields[name].datatype == 7 else 'd'))) for name in ('x', 'y', 'z')]
        if message.point_step <= 0 or any(offset < 0 or offset + codec.size > message.point_step for offset, codec in codecs):
            return
        total = message.width * message.height
        if total <= 0 or message.row_step < message.width * message.point_step or len(message.data) < message.row_step * message.height:
            return
        stride = max(1, math.ceil(total / 2500))
        points = []
        for index in range(0, total, stride):
            row, column = divmod(index, message.width)
            base = row * message.row_step + column * message.point_step
            x, y, z = (codec.unpack_from(message.data, base + offset)[0] for offset, codec in codecs)
            if all(math.isfinite(value) and abs(value) < 5000 for value in (x, y, z)):
                points.append((round(x, 2), round(y, 2)))
        with self.lock:
            frame = message.header.frame_id
            if frame != self.frame:
                self.cells.clear()
                self.frame = frame
            for x, y in points:
                key = (round(x / 0.15), round(y / 0.15))
                self.cells[key] = (x, y)
            while len(self.cells) > 20000:
                del self.cells[next(iter(self.cells))]
            self.received_at = time.time()
            self.last_sample = now
            self.scans += 1
            self.input_points = total
            self.revision += 1
            self.cached = None

    def snapshot(self):
        with self.lock:
            if self.cached is None:
                values = list(self.cells.values())
                stride = max(1, math.ceil(len(values) / 5000))
                self.cached = values[::stride]
            return {'available': bool(self.cached), 'stale': time.time() - self.received_at > 3,
                    'frame_id': self.frame, 'points': self.cached, 'revision': self.revision,
                    'scans': self.scans, 'last_scan_points': self.input_points, 'received_at': self.received_at}
