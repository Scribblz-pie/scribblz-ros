#!/usr/bin/env python3

"""
UDP packet parsing and scan assembly for lidar data.

This module handles:
- Parsing binary UDP packets from the lidar sensor
- Receiving packets via UDP socket
- Assembling packets into complete 360-degree scans
"""

import socket
import struct
import threading
import time
from collections import deque, defaultdict

# Throttling for error messages (print at most once per 10 seconds per error type)
_error_message_times = {}
_error_message_interval = 10.0  # seconds


def parse_lidar_packet(data):
    """
    Parse a binary UDP packet from the lidar sensor.
    
    Args:
        data: Raw bytes from UDP packet
        
    Returns:
        Tuple of (angles, distances, intensities) or None if parsing fails
    """
    try:
        if len(data) < 32:
            error_key = f"small_{len(data)}"
            now = time.time()
            if error_key not in _error_message_times or (now - _error_message_times[error_key]) > _error_message_interval:
                print(f"[Parser] Packet too small: {len(data)} bytes")
                _error_message_times[error_key] = now
            return None
        
        header = struct.unpack_from("<H", data, 0)[0]
        if header != 0xFAC7:
            error_key = f"bad_header_{header:#06x}"
            now = time.time()
            if error_key not in _error_message_times or (now - _error_message_times[error_key]) > _error_message_interval:
                print(f"[Parser] Bad header: {header:#06x}, expected 0xFAC7, size={len(data)} bytes (throttled)")
                _error_message_times[error_key] = now
            return None

        num_points = struct.unpack_from("<H", data, 2)[0]
        start_angle = struct.unpack_from("<I", data, 8)[0] / 1000.0
        flags = struct.unpack_from("<I", data, 16)[0]

        distance_scale = 0.001  # mm to m
        has_intensity = bool(flags & 0x02)

        pos = 28
        distances = []
        for _ in range(num_points):
            if pos + 2 > len(data):
                break
            d = struct.unpack_from("<H", data, pos)[0]
            distances.append(d * distance_scale)
            pos += 2

        rel_angles = []
        for _ in range(len(distances)):
            if pos + 2 > len(data):
                break
            a = struct.unpack_from("<H", data, pos)[0] / 1000.0
            rel_angles.append(a)
            pos += 2

        intensities = None
        if has_intensity:
            intensities = []
            for _ in range(len(distances)):
                if pos >= len(data):
                    break
                intensities.append(data[pos])
                pos += 1

        abs_angles = [(start_angle + ra) % 360.0 for ra in rel_angles]
        return abs_angles, distances, intensities
    except Exception as e:
        print(f"[Parser] Exception during parsing: {e}")
        import traceback
        traceback.print_exc()
        return None


class LidarReceiver(threading.Thread):
    """
    Thread that receives UDP packets from the lidar sensor.
    
    Parses incoming packets and adds them to a buffer for processing.
    """
    
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
        """Main receiver loop - runs in separate thread."""
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
            except Exception as e:
                print(f"[Receiver] error: {e}")
                break

            pkt_cnt += 1
            # Only print every 100 packets to reduce spam
            if pkt_cnt % 100 == 0:
                print(f"[Receiver] Received {pkt_cnt} packets from {addr}, latest size={len(data)} bytes")
            
            parsed = parse_lidar_packet(data)
            if parsed is None:
                continue
            angles, dists, ints = parsed
            self.buffer.append((time.time(), addr, angles, dists, ints))

            now = time.time()
            if now - self.last_print > self.print_interval and dists:
                self.last_print = now
                avg = sum(dists) / len(dists)
                print(f"[{time.strftime('%H:%M:%S')}] pkt {pkt_cnt} {len(dists)} pts avg={avg:.2f}m range={min(dists):.2f}-{max(dists):.2f}m")
        if self.sock:
            self.sock.close()
        print("[Receiver] stopped")


class ScanAssembler:
    """
    Assembles individual lidar packets into partial scans.
    
    Accumulates points from multiple packets and publishes at 180-degree intervals
    for faster update rates (~2x improvement over full 360-degree scans).
    """
    
    def __init__(self, angle_tol):
        """
        Initialize scan assembler.
        
        Args:
            angle_tol: Angle tolerance for binning points (degrees)
        """
        self.angle_tol = angle_tol
        self.reset()

    def reset(self):
        """Reset assembler state for a new scan."""
        self.points = defaultdict(lambda: {"dist": [], "int": []})
        self.start_angle = None
        self.min_angle = None
        self.max_angle = None

    def add_packet(self, ts, addr, angles, distances, intensities):
        """
        Add a packet to the current scan.
        
        Args:
            ts: Timestamp
            addr: Source address
            angles: List of angles (degrees)
            distances: List of distances (meters)
            intensities: List of intensities (or None)
            
        Returns:
            Completed scan tuple (ts, addr, angles, distances, intensities) or None
        """
        if not angles:
            return None
            
        # Track angle range for this scan
        packet_min = min(angles)
        packet_max = max(angles)
        
        if self.min_angle is None:
            self.min_angle = packet_min
            self.max_angle = packet_max
        else:
            self.min_angle = min(self.min_angle, packet_min)
            self.max_angle = max(self.max_angle, packet_max)

        # Add points to scan
        for a, d, i in zip(angles, distances, intensities or [None] * len(distances)):
            key = round(a / self.angle_tol) * self.angle_tol
            self.points[key]["dist"].append(d)
            if i is not None:
                self.points[key]["int"].append(i)

        # Publish when we have 180+ degrees of coverage
        # Handle angle wrapping around 0/360
        coverage = self.max_angle - self.min_angle
        if coverage < 0:  # Wrapped around 0/360
            coverage = 360 + coverage
            
        if coverage >= 180.0 and len(self.points) > 10:  # Minimum 10 points
            return self._finalize_scan(ts, addr)
        return None

    def _finalize_scan(self, ts, addr):
        """
        Finalize the current scan and return complete scan data.
        
        Returns:
            Tuple of (ts, addr, angles, distances, intensities)
        """
        angles, distances, intensities = [], [], []
        for a in sorted(self.points.keys()):
            dists = self.points[a]["dist"]
            ints = self.points[a]["int"]
            if dists:
                distances.append(min(dists))
                angles.append(a)
                intensities.append(int(sum(ints) / len(ints)) if ints else None)
        self.reset()
        return (ts, addr, angles, distances, intensities)

