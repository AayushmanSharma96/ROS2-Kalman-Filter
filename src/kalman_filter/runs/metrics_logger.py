#!/usr/bin/env python3
import rclpy, csv, math, numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Float64

def yaw_from_q(q):
    # Z-only yaw (assumes flat sim)
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))

class MetricsLogger(Node):
    def __init__(self):
        super().__init__('metrics_logger')
        
        self.path_topic   = self.declare_parameter('path_topic', '/path').get_parameter_value().string_value
        self.est_topic    = self.declare_parameter('est_pose_topic', '/kf/pose').get_parameter_value().string_value
        self.gt_topic     = self.declare_parameter('gt_pose_topic',  '/sim/true_pose').get_parameter_value().string_value
        self.cmd_topic    = self.declare_parameter('cmd_topic', '/cmd_vel').get_parameter_value().string_value
        self.enc_topic    = self.declare_parameter('encoder_topic', '/encoder/speed').get_parameter_value().string_value
        self.csv_path     = self.declare_parameter('csv', 'metrics.csv').get_parameter_value().string_value

        # Data
        self.path_xy = []            # [(x,y), ...]
        self.path_s  = []            # cumulative arclength
        self.last_est = None         # (t, x, y)
        self.last_cmd = None         # (t, a, omega)
        self.last_a   = None         # (t, a) for jerk
        self.v_enc    = 0.0

        # CSV
        self.fh = open(self.csv_path, 'w', newline='')
        self.wr = csv.writer(self.fh)
        self.wr.writerow([
            't','who',                 # who: est or gt
            'x','y','theta','v_enc',
            'cte','heading_err','s_along',
            'a_cmd','omega_cmd','jerk'
        ])

        # Subs
        self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.create_subscription(PoseStamped, self.est_topic, lambda m: self.pose_cb(m,'est'), 50)
        self.create_subscription(PoseStamped, self.gt_topic,  lambda m: self.pose_cb(m,'gt'),  50)
        self.create_subscription(Twist, self.cmd_topic, self.cmd_cb, 50)
        self.create_subscription(Float64, self.enc_topic, self.enc_cb, 50)

        self.get_logger().info(f"Logging metrics to {self.csv_path}")

    def destroy_node(self):
        try:
            self.fh.flush(); self.fh.close()
        except Exception:
            pass
        super().destroy_node()

    def enc_cb(self, msg: Float64):
        self.v_enc = float(msg.data)

    def cmd_cb(self, msg: Twist):
        t = self.get_clock().now().nanoseconds * 1e-9
        a = float(msg.linear.x)
        if self.last_a is not None:
            dt = max(t - self.last_a[0], 1e-6)
            jerk = (a - self.last_a[1]) / dt
        else:
            jerk = 0.0
        self.last_a = (t, a)
        self.last_cmd = (t, a, float(msg.angular.z), jerk)

    def path_cb(self, msg: Path):
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.path_xy = pts
        self.path_s  = self._cumulative_s(pts)

    def pose_cb(self, msg: PoseStamped, who: str):
        if not self.path_xy:
            return
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.position.x
        y = msg.pose.position.y
        th = yaw_from_q(msg.pose.orientation)

        # metrics vs path
        cte, hdg_err, s_along = self._errors_vs_path(x, y, th)

        a_cmd, om_cmd, jerk = (self.last_cmd[1], self.last_cmd[2], self.last_cmd[3]) if self.last_cmd else (0.0,0.0,0.0)

        self.wr.writerow([f"{t:.6f}", who, f"{x:.6f}", f"{y:.6f}", f"{th:.6f}",
                          f"{self.v_enc:.6f}", f"{cte:.6f}", f"{hdg_err:.6f}", f"{s_along:.6f}",
                          f"{a_cmd:.6f}", f"{om_cmd:.6f}", f"{jerk:.6f}"])

    def _cumulative_s(self, pts):
        s = [0.0]
        for i in range(1, len(pts)):
            dx = pts[i][0]-pts[i-1][0]; dy = pts[i][1]-pts[i-1][1]
            s.append(s[-1] + math.hypot(dx,dy))
        return s

    def _errors_vs_path(self, x, y, theta):
        
        # nearest segment projection with wraparound
        best = (1e18, 0, (0.0,0.0), (0.0,0.0), 0)  # (dist2, i, A, B, j)
        n = len(self.path_xy)
        for i in range(n):
            A = self.path_xy[i]
            B = self.path_xy[(i+1) % n]
            d2, j = self._point_seg_dist2_and_t((x,y), A, B)
            if d2 < best[0]:
                best = (d2, i, A, B, j)
        _, i, A, B, t = best

        # closest point C = A + t*(B-A)
        Cx = A[0] + t*(B[0]-A[0]); Cy = A[1] + t*(B[1]-A[1])

        # signed CTE (left of AB positive)
        ABx = B[0]-A[0]; ABy = B[1]-A[1]
        APx = x-A[0];  APy = y-A[1]
        cross = ABx*APy - ABy*APx
        cte = math.copysign(math.sqrt(best[0]), cross)

        # path tangent heading at segment
        seg_yaw = math.atan2(ABy, ABx)
        hdg_err = self._wrap(theta - seg_yaw)

        # arclength at projection
        seg_len = math.hypot(ABx, ABy)
        s_base  = self.path_s[i]
        s_here  = s_base + t*seg_len

        return cte, hdg_err, s_here

    @staticmethod
    def _point_seg_dist2_and_t(P, A, B):
        # returns (min distance^2, t in [0,1])
        APx = P[0]-A[0]; APy = P[1]-A[1]
        ABx = B[0]-A[0]; ABy = B[1]-A[1]
        denom = ABx*ABx + ABy*ABy
        if denom <= 1e-12:
            return ((P[0]-A[0])**2 + (P[1]-A[1])**2, 0.0)
        t = (APx*ABx + APy*ABy) / denom
        t = max(0.0, min(1.0, t))
        Cx = A[0] + t*ABx; Cy = A[1] + t*ABy
        d2 = (P[0]-Cx)**2 + (P[1]-Cy)**2
        return d2, t

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

def main():
    rclpy.init()
    node = MetricsLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
