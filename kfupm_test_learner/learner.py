"""Practice: drive reactively and record one lap of the track (from the car's own
odometry) into F1TENTH_DATA_DIR. Official run: load that lap, turn it into a racing
line with a speed profile, and follow it with pure pursuit. No lap saved -> reactive."""
import math
import os

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String

from .gap_follower import MAX_STEER, gap_command

WHEELBASE = 0.33
LAP_FILE = "lap.csv"


def racing_line(points, v_max=5.5, a_lat=3.2, a_brake=3.5):
    """Resample the recorded lap every 0.1 m, smooth it, and give each point a speed."""
    p = np.asarray(points, dtype=float)
    seg = np.linalg.norm(np.diff(p, axis=0, append=p[:1]), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    closed = np.vstack([p, p[:1]])
    t = np.arange(0.0, s[-1], 0.1)
    line = np.column_stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])])
    k = 9                                    # circular moving average, ~0.9 m each side
    pad = np.vstack([line[-k:], line, line[:k]])
    kernel = np.ones(2 * k + 1) / (2 * k + 1)
    line = np.column_stack([np.convolve(pad[:, i], kernel, mode="valid") for i in range(2)])
    a, b, c = np.roll(line, 8, 0), line, np.roll(line, -8, 0)
    ab, bc, ca = (np.linalg.norm(a - b, axis=1), np.linalg.norm(b - c, axis=1), np.linalg.norm(c - a, axis=1))
    cross = np.abs((b - a)[:, 0] * (c - a)[:, 1] - (b - a)[:, 1] * (c - a)[:, 0])
    radius = np.where(cross > 1e-9, ab * bc * ca / (2 * np.maximum(cross, 1e-9)), 1e9)
    v = np.minimum(v_max, np.sqrt(a_lat * radius))
    for _ in range(2):                       # brake in time for corners (wraps around the lap)
        for i in range(len(v) - 1, -1, -1):
            v[i] = min(v[i], math.sqrt(v[(i + 1) % len(v)] ** 2 + 2 * a_brake * 0.1))
    return line, v


class Learner(Node):
    def __init__(self, data_dir_override=None, crash_without_lap=False):
        super().__init__("stage3_test_learner")
        self.mode = os.environ.get("F1TENTH_MODE", "race")
        self.data = data_dir_override or os.environ.get("F1TENTH_DATA_DIR", "")
        self.pose, self.yaw, self.idx = None, 0.0, None
        self.racing, self.laps, self.rec, self.saved = False, 0, [], False
        self.line = self.speed = None
        if self.mode == "practice" and self.data:
            os.makedirs(self.data, exist_ok=True)
        lap_path = os.path.join(self.data, LAP_FILE) if self.data else ""
        if self.mode == "race" and (crash_without_lap or (lap_path and os.path.isfile(lap_path))):
            pts = np.loadtxt(lap_path, delimiter=",", ndmin=2)
            self.line, self.speed = racing_line(pts[:, :2])
        self.get_logger().info(f"mode={self.mode} data_dir={self.data or '-'} "
                               f"{'racing line ' + str(len(self.line)) + ' points, top ' + format(self.speed.max(), '.1f') + ' m/s' if self.line is not None else 'reactive driving'}")
        self.pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self.create_subscription(LaserScan, "/scan", self.on_scan, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.on_odom, 10)
        self.create_subscription(String, "/competition/race_status", self.on_status, 10)
        self.create_subscription(Int32, "/competition/lap_count", self.on_laps, 10)

    def on_status(self, msg):
        self.racing = msg.data == "RACING"

    def on_laps(self, msg):
        if msg.data > self.laps and self.mode == "practice" and not self.saved and len(self.rec) > 50:
            # first timed lap done: save it now (practice can end at any moment)
            tmp = os.path.join(self.data, LAP_FILE + ".tmp")
            np.savetxt(tmp, np.asarray(self.rec), delimiter=",", fmt="%.3f")
            os.replace(tmp, os.path.join(self.data, LAP_FILE))
            self.saved = True
            self.get_logger().info(f"saved one lap: {len(self.rec)} points")
        self.laps = msg.data

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y)
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        if self.mode == "practice" and self.racing and self.laps == 0:
            if not self.rec or math.dist(self.rec[-1], self.pose) > 0.2:
                self.rec.append(self.pose)
        if self.line is not None:
            self.follow()

    def follow(self):
        pos, n = np.asarray(self.pose), len(self.line)
        if self.idx is None:
            self.idx = int(np.argmin(np.linalg.norm(self.line - pos, axis=1)))
        else:
            w = (self.idx + np.arange(0, 80)) % n
            self.idx = int(w[np.argmin(np.linalg.norm(self.line[w] - pos, axis=1))])
        v = float(self.speed[self.idx])
        target = self.line[(self.idx + int((0.8 + 0.18 * v) / 0.1)) % n]
        dx, dy = target - pos
        ly = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy
        steer = math.atan(2 * WHEELBASE * ly / max(dx * dx + dy * dy, 1e-6))
        self.send(float(np.clip(steer, -MAX_STEER, MAX_STEER)), v)

    def on_scan(self, scan):
        steer, speed, ahead = gap_command(scan, v_max=4.0 if self.mode == "practice" else 4.5)
        if self.line is None:
            self.send(steer, speed)

    def send(self, steer, speed):
        m = AckermannDriveStamped()
        m.drive.steering_angle, m.drive.speed = steer, speed
        self.pub.publish(m)


def run(**kw):
    rclpy.init()
    node = Learner(**kw)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
