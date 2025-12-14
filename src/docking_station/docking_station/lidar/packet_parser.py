"""
UDP packet parsing and scan assembly for lidar data.
"""

import socket
import struct
import threading
import time
import traceback
from collections import deque, defaultdict

def parse_lidar_packet(data):
    try:
        # Ignore heartbeat packets (prevent "Bad Header" spam)
        if len(data) == 128:
            return None

        if len(data) < 32:
            return None
        
        header = struct.unpack_from("<H", data, 0)[0]
        if header != 0xFAC7:
            return None

        # Extract Metadata
        num_points = struct.unpack_from("<H", data, 2)[0]
        start_angle = struct.unpack_from("<I", data, 8)[0] / 1000.0
        flags = struct.unpack_from("<I", data, 16)[0]

        distance_scale = 0.001 
        has_intensity = bool(flags & 0x02)

        # Parse Points
        pos = 28
        distances = []
        for _ in range(num_points):
            if pos + 2 > len(data): break
            d = struct.unpack_from("<H", data, pos)[0]
            distances.append(d * distance_scale)
            pos += 2

        rel_angles = []
        for _ in range(len(distances)):
            if pos + 2 > len(data): break
            a = struct.unpack_from("<H", data, pos)[0] / 1000.0
            rel_angles.append(a)
            pos += 2

        intensities = None
        if has_intensity:
            intensities = []
            for _ in range(len(distances)):
                if pos >= len(data): break
                intensities.append(data[pos])
                pos += 1

        abs_angles = [(start_angle + ra) % 360.0 for ra in rel_angles]
        return abs_angles, distances, intensities

    except Exception as e:
        print(f"[Parser] CRASH: {e}")
        traceback.print_exc()
        return None


class LidarReceiver(threading.Thread):
    def __init__(self, host, port, buffer_deque, stop_event, print_interval=2.0):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.buffer = buffer_deque
        self.stop_event = stop_event
        self.print_interval = print_interval
        self.last_print = 0
        self.sock = None

    def run(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind((self.host, self.port))
            self.sock.settimeout(1.0)
        except Exception as e:
            print(f"[Receiver] bind error: {e}")
            self.stop_event.set()
            return

        pkt_cnt = 0
        while not self.stop_event.is_set():
            try:
                data, addr = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            except Exception:
                continue

            pkt_cnt += 1
            parsed = parse_lidar_packet(data)
            if parsed is None:
                continue
            
            angles, dists, ints = parsed
            self.buffer.append((time.time(), addr, angles, dists, ints))

            now = time.time()
            if now - self.last_print > self.print_interval and dists:
                self.last_print = now
                avg = sum(dists) / len(dists)
                print(f"[{time.strftime('%H:%M:%S')}] pkt {pkt_cnt} {len(dists)} pts avg={avg:.2f}m")
        
        if self.sock: self.sock.close()


class ScanAssembler:
    def __init__(self, angle_tol):
        self.angle_tol = angle_tol
        self.reset()

    def reset(self):
        self.points = defaultdict(lambda: {"dist": [], "int": []})
        self.last_min_angle = None

    def add_packet(self, ts, addr, angles, distances, intensities):
        if not angles:
            return None

        curr_min = min(angles)
        
        # --- FIXED LOGIC ---
        # Instead of comparing to start_angle, we check if the angle 
        # dropped significantly (e.g. from 350 to 10) compared to the last packet.
        packet_completes_scan = False
        if self.last_min_angle is not None:
             # If angle dropped by more than 180 degrees, we wrapped around
             if self.last_min_angle - curr_min > 180.0:
                 packet_completes_scan = True
        
        result = None
        if packet_completes_scan:
            # print(f"[Assembler] Wrap detected {self.last_min_angle:.1f}->{curr_min:.1f}. Publishing.")
            result = self._finalize_scan(ts, addr)
        
        # Add current packet data
        for a, d, i in zip(angles, distances, intensities or [None] * len(distances)):
            key = round(a / self.angle_tol) * self.angle_tol
            self.points[key]["dist"].append(d)
            if i is not None:
                self.points[key]["int"].append(i)
        
        self.last_min_angle = curr_min
        return result

    def _finalize_scan(self, ts, addr):
        angles, distances, intensities = [], [], []
        for a in sorted(self.points.keys()):
            dists = self.points[a]["dist"]
            ints = self.points[a]["int"]
            if dists:
                distances.append(min(dists))
                angles.append(a)
                intensities.append(int(sum(ints) / len(ints)) if ints else 0)
        
        self.points = defaultdict(lambda: {"dist": [], "int": []})
        
        if not angles:
            return None
            
        return (ts, addr, angles, distances, intensities)