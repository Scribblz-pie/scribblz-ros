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
            return None
        print(f"[Parser] Packet too small: {len(data)} bytes")
        
        header = struct.unpack_from("<H", data, 0)[0]
        if header != 0xFAC7:
            return None
        
        # after: if header != 0xFAC7:
        print(f"[Parser] Bad header: {header:#06x}, size={len(data)}")

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
    except Exception:
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
            print(f"[Receiver] Got UDP packet #{pkt_cnt} from {addr}, size={len(data)} bytes")
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
    Assembles individual lidar packets into complete 360-degree scans.
    
    Accumulates points from multiple packets until a full rotation is detected.
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
        if self.start_angle is None and angles:
            self.start_angle = min(angles)

        for a, d, i in zip(angles, distances, intensities or [None] * len(distances)):
            key = round(a / self.angle_tol) * self.angle_tol
            self.points[key]["dist"].append(d)
            if i is not None:
                self.points[key]["int"].append(i)

        if angles and min(angles) < (self.start_angle or 0) * 0.5:
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

