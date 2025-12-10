#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
import socket, struct, threading, time, math
from collections import deque, defaultdict
import numpy as np



def parse_lidar_packet(data):
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
    def __init__(self, angle_tol):
        self.angle_tol = angle_tol
        self.reset()

    def reset(self):
        self.points = defaultdict(lambda: {"dist": [], "int": []})
        self.start_angle = None

    def add_packet(self, ts, addr, angles, distances, intensities):
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


def find_small_clusters(points, min_pts, max_pts, dist_thresh):
    if len(points) < min_pts:
        return []

    unvisited = set(range(len(points)))
    clusters = []

    while unvisited:
        seed = unvisited.pop()
        cluster_indices = [seed]
        to_check = [seed]

        while to_check:
            current = to_check.pop()
            current_point = points[current]

            unvisited_list = list(unvisited)
            if len(unvisited_list) > 0:
                unvisited_points = points[unvisited_list]
                distances = np.linalg.norm(unvisited_points - current_point, axis=1)
                nearby_mask = distances < dist_thresh

                for i, is_nearby in enumerate(nearby_mask):
                    if is_nearby:
                        idx = unvisited_list[i]
                        unvisited.discard(idx)
                        cluster_indices.append(idx)
                        to_check.append(idx)

        if min_pts <= len(cluster_indices) <= max_pts:
            clusters.append(points[cluster_indices])

    clusters.sort(key=len)
    return clusters


def filter_valid_points(points, min_dist, max_dist):
    distances = np.sqrt(points[:, 0] ** 2 + points[:, 1] ** 2)
    valid_mask = (distances > min_dist) & (distances < max_dist)
    return points[valid_mask]


def _fit_circle_algebraic(points):
    if len(points) < 3:
        return None
    try:
        x = points[:, 0]
        y = points[:, 1]
        A = np.column_stack([2 * x, 2 * y, np.ones(len(points))])
        b = x ** 2 + y ** 2
        params, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        cx, cy, D = params
        r_squared = cx ** 2 + cy ** 2 + D
        if r_squared <= 0:
            return None
        r = math.sqrt(r_squared)
        return (cx, cy, r)
    except Exception:
        return None


def fit_circle_ransac(angles, distances, cfg):
    if len(angles) < cfg["min_circle_points"]:
        return None

    points = np.array([(dist * 100.0 * math.cos(math.radians(angle)),
                        dist * 100.0 * math.sin(math.radians(angle)))
                       for angle, dist in zip(angles, distances)])

    valid_points = filter_valid_points(points, cfg["min_detection_distance_cm"], cfg["max_detection_distance_cm"])
    if len(valid_points) < cfg["min_circle_points"]:
        return None

    clusters = find_small_clusters(valid_points, cfg["min_circle_points"], 30, 10.0)
    if len(clusters) == 0:
        return None

    best_fit = None
    best_score = float('inf')

    for cluster_points in clusters:
        if len(cluster_points) < cfg["min_circle_points"]:
            continue

        for _ in range(cfg["ransac_iterations"]):
            if len(cluster_points) < 3:
                break

            sample_indices = np.random.choice(len(cluster_points), 3, replace=False)
            fit_result = _fit_circle_algebraic(cluster_points[sample_indices])
            if fit_result is None:
                continue

            cx, cy, r = fit_result

            if r < max(0.5, cfg["expected_radius_cm"] - cfg["radius_tolerance_cm"]) or r > cfg["expected_radius_cm"] + cfg["radius_tolerance_cm"]:
                continue

            distances_to_circle = np.abs(np.sqrt((cluster_points[:, 0] - cx) ** 2 + (cluster_points[:, 1] - cy) ** 2) - r)
            inliers = distances_to_circle < cfg["inlier_threshold_cm"]
            inlier_count = np.sum(inliers)

            if inlier_count < cfg["min_circle_points"]:
                continue

            score = np.mean(distances_to_circle[inliers]) + abs(r - cfg["expected_radius_cm"]) * 0.2

            if score < best_score:
                best_score = score
                inlier_points = cluster_points[inliers]
                refined_fit = _fit_circle_algebraic(inlier_points)
                if refined_fit is not None:
                    best_fit = (*refined_fit, inlier_count)

    return best_fit


class LidarPoseNode(Node):
    def __init__(self):
        super().__init__('lidar_pose')

        # params
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 6794)
        self.declare_parameter('angle_tolerance', 0.05)
        self.declare_parameter('start_filter_angle', 0.0)
        self.declare_parameter('end_filter_angle', 360.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('scan_frame_id', 'lidar')
        self.declare_parameter('print_interval', 2.0)

        # circle fit config
        self.declare_parameter('expected_radius_cm', 3.0)
        self.declare_parameter('radius_tolerance_cm', 5.0)
        self.declare_parameter('min_circle_points', 3)
        self.declare_parameter('ransac_iterations', 50)
        self.declare_parameter('inlier_threshold_cm', 2.0)
        self.declare_parameter('min_detection_distance_cm', 5.0)
        self.declare_parameter('max_detection_distance_cm', 500.0)

        self.host = self.get_parameter('host').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.angle_tol = self.get_parameter('angle_tolerance').get_parameter_value().double_value
        self.start_filter = self.get_parameter('start_filter_angle').get_parameter_value().double_value
        self.end_filter = self.get_parameter('end_filter_angle').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.scan_frame_id = self.get_parameter('scan_frame_id').get_parameter_value().string_value
        self.print_interval = self.get_parameter('print_interval').get_parameter_value().double_value

        self.cfg = {
            "expected_radius_cm": self.get_parameter('expected_radius_cm').get_parameter_value().double_value,
            "radius_tolerance_cm": self.get_parameter('radius_tolerance_cm').get_parameter_value().double_value,
            "min_circle_points": self.get_parameter('min_circle_points').get_parameter_value().integer_value,
            "ransac_iterations": self.get_parameter('ransac_iterations').get_parameter_value().integer_value,
            "inlier_threshold_cm": self.get_parameter('inlier_threshold_cm').get_parameter_value().double_value,
            "min_detection_distance_cm": self.get_parameter('min_detection_distance_cm').get_parameter_value().double_value,
            "max_detection_distance_cm": self.get_parameter('max_detection_distance_cm').get_parameter_value().double_value,
        }

        qos = QoSProfile(depth=10)
        self.scan_pub = self.create_publisher(LaserScan, '/lidar/scan', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', qos)

        self.buffer = deque(maxlen=64)
        self.stop_event = threading.Event()
        self.receiver = LidarReceiver(self.host, self.port, self.buffer, self.stop_event, self.print_interval)
        self.assembler = ScanAssembler(self.angle_tol)

        self.receiver.start()
        self.timer = self.create_timer(0.02, self.process_buffer)

        self.get_logger().info(f'lidar pose node listening on {self.host}:{self.port}')

    def destroy_node(self):
        self.stop_event.set()
        if self.receiver.is_alive():
            self.receiver.join(timeout=2.0)
        super().destroy_node()

    def angle_in_filter(self, a):
        if self.start_filter <= self.end_filter:
            return self.start_filter <= a <= self.end_filter
        return a >= self.start_filter or a <= self.end_filter

    def process_buffer(self):
        updated = False
        scan = None
        while self.buffer:
            pkt = self.buffer.popleft()
            res = self.assembler.add_packet(*pkt)
            if res:
                scan = res
                updated = True

        if not updated or scan is None:
            return

        ts, addr, angles, distances, intensities = scan

        filt_angles = []
        filt_distances = []
        for a, d in zip(angles, distances):
            if self.angle_in_filter(a):
                filt_angles.append(a)
                filt_distances.append(d)

        if len(filt_angles) == 0:
            return

        self.publish_scan(filt_angles, filt_distances, ts)
        circle_fit = fit_circle_ransac(filt_angles, filt_distances, self.cfg)
        if circle_fit:
            self.publish_pose(circle_fit, ts)

    def publish_scan(self, angles_deg, distances_m, ts):
        sorted_pairs = sorted(zip(angles_deg, distances_m), key=lambda x: x[0])
        angles_deg_sorted = [a for a, _ in sorted_pairs]
        ranges = [d for _, d in sorted_pairs]

        if not angles_deg_sorted:
            return

        angle_min = math.radians(angles_deg_sorted[0])
        angle_max = math.radians(angles_deg_sorted[-1])
        if len(angles_deg_sorted) > 1:
            angle_increment = (angle_max - angle_min) / (len(angles_deg_sorted) - 1)
        else:
            angle_increment = 0.0

        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.scan_frame_id
        msg.angle_min = angle_min
        msg.angle_max = angle_max
        msg.angle_increment = angle_increment
        msg.time_increment = 0.0
        msg.scan_time = 0.0
        msg.range_min = 0.0
        msg.range_max = max(ranges) if ranges else 0.0
        msg.ranges = ranges
        msg.intensities = []
        self.scan_pub.publish(msg)

    def publish_pose(self, circle_fit, ts):
        cx_cm, cy_cm, r, inliers = circle_fit
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = cx_cm / 100.0
        pose.pose.position.y = cy_cm / 100.0
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

