#!/usr/bin/env python3
import math
from statistics import median
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from std_srvs.srv import SetBool
from std_msgs.msg import String, Float32

from rcl_interfaces.msg import SetParametersResult


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        # I/O
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        # Enable via service
        self.declare_parameter("enable_service", "/wall_follower/enable")

        # Runtime commands via topics
        self.declare_parameter("set_side_topic", "/wall_follower/set_side")           # std_msgs/String: "left"/"right"
        self.declare_parameter("set_distance_topic", "/wall_follower/set_distance")   # std_msgs/Float32: meters

        # Runtime-configurable behavior (parameters)
        self.declare_parameter("enabled", False)
        self.declare_parameter("side", "right")          # "left" | "right"
        self.declare_parameter("desired_distance", 0.05) # [m]

        self.declare_parameter("forward_speed", 0.08)    # [m/s]
        self.declare_parameter("forward_angle_deg", 0.0) # [deg] (180 pokud je lidar otočený)

        # ===== Target X (drive along wall only in X) =====
        # Pokud use_target_x=True, follower se zastaví, jakmile odom.x dosáhne target_x v toleranci.
        self.declare_parameter("use_target_x", False)
        self.declare_parameter("target_x", 0.0)          # [m] v odom frame
        self.declare_parameter("x_tolerance", 0.02)      # [m]

        # Geometry
        self.declare_parameter("theta_deg", 30.0)
        self.declare_parameter("lookahead", 0.20)

        # PID lateral -> vy
        self.declare_parameter("kp_lat", 1.2)
        self.declare_parameter("ki_lat", 0.0)
        self.declare_parameter("kd_lat", 0.05)
        self.declare_parameter("integral_limit", 0.30)

        # Angle alignment -> wz
        self.declare_parameter("k_angle", 1.5)

        # Limits
        self.declare_parameter("max_vx", 0.20)
        self.declare_parameter("max_vy", 0.20)
        self.declare_parameter("max_wz", 1.2)

        # Safety
        self.declare_parameter("min_front_dist", 0.25)
        self.declare_parameter("front_window_deg", 20.0)

        # Robustness + loop
        self.declare_parameter("sample_window", 2)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("reset_pid_on_disable", True)
        self.declare_parameter("reset_pid_on_reconfigure", True)

        # Load names
        self.scan_topic = self.get_parameter("scan_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.enable_srv_name = self.get_parameter("enable_service").value
        self.set_side_topic = self.get_parameter("set_side_topic").value
        self.set_distance_topic = self.get_parameter("set_distance_topic").value

        # Pub/Sub
        self.sub_scan = self.create_subscription(
            LaserScan, self.scan_topic, self.on_scan, qos_profile_sensor_data
        )
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.on_odom, 20
        )
        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)

        # Service enable/disable
        self.srv_enable = self.create_service(SetBool, self.enable_srv_name, self.handle_enable)

        # Topics for side/distance
        self.sub_side = self.create_subscription(String, self.set_side_topic, self.on_side_cmd, 10)
        self.sub_dist = self.create_subscription(Float32, self.set_distance_topic, self.on_distance_cmd, 10)

        # Parameter validation callback
        self.add_on_set_parameters_callback(self.on_params)

        # Control loop
        rate = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / max(rate, 1.0), self.on_timer)

        # State
        self.latest_scan: Optional[LaserScan] = None

        # Odom X tracking
        self.current_x: Optional[float] = None
        self._target_dir: Optional[int] = None  # +1 if target ahead in +X, -1 if in -X, None not set

        # PID state
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

        # Cached runtime params
        self.enabled = bool(self.get_parameter("enabled").value)
        self.side = str(self.get_parameter("side").value).lower().strip()
        self.desired_distance = float(self.get_parameter("desired_distance").value)

        self.use_target_x = bool(self.get_parameter("use_target_x").value)
        self.target_x = float(self.get_parameter("target_x").value)
        self.x_tolerance = float(self.get_parameter("x_tolerance").value)

        self.get_logger().info(
            f"WallFollower ready. enable service: {self.enable_srv_name} | side topic: {self.set_side_topic} | dist topic: {self.set_distance_topic} | odom={self.odom_topic}"
        )

    # ============================================================
    #  VEŘEJNÉ FUNKCE (přímo v třídě WallFollower)
    # ============================================================
    def start_jizdy(self) -> bool:
        """Zapne wall-following (začne publikovat cmd_vel)."""
        self._target_dir = None  # přepočítá se podle aktuálního X při běhu
        return self._set_enabled(True)

    def zastav_jizdu(self) -> bool:
        """Vypne wall-following a ihned pošle stop cmd_vel."""
        ok = self._set_enabled(False)
        self._publish_stop()
        return ok

    def nastav_vzdalenost(self, distance_m: float) -> bool:
        """Nastaví požadovanou vzdálenost od zdi (v metrech)."""
        results = self.set_parameters([
            rclpy.parameter.Parameter("desired_distance", rclpy.Parameter.Type.DOUBLE, float(distance_m))
        ])
        ok = all(r.successful for r in results)
        if not ok:
            reason = next((r.reason for r in results if not r.successful), "set desired_distance failed")
            self.get_logger().warn(f"nastav_vzdalenost failed: {reason}")
        return ok

    def nastav_stranu(self, side: str) -> bool:
        """Nastaví stranu ('left' nebo 'right')."""
        s = str(side).lower().strip()
        results = self.set_parameters([
            rclpy.parameter.Parameter("side", rclpy.Parameter.Type.STRING, s)
        ])
        ok = all(r.successful for r in results)
        if not ok:
            reason = next((r.reason for r in results if not r.successful), "set side failed")
            self.get_logger().warn(f"nastav_stranu failed: {reason}")
        return ok

    def nastav_cilove_x(self, target_x: float, tolerance: Optional[float] = None) -> bool:
        """
        Nastaví cílovou pozici na ose X (v odom frame) a zapne režim use_target_x.
        tolerance (volitelně) upraví x_tolerance.
        """
        params = [
            rclpy.parameter.Parameter("target_x", rclpy.Parameter.Type.DOUBLE, float(target_x)),
            rclpy.parameter.Parameter("use_target_x", rclpy.Parameter.Type.BOOL, True),
        ]
        if tolerance is not None:
            params.append(rclpy.parameter.Parameter("x_tolerance", rclpy.Parameter.Type.DOUBLE, float(tolerance)))

        results = self.set_parameters(params)
        ok = all(r.successful for r in results)
        if ok:
            self._target_dir = None  # přepočítat směr k cíli
        else:
            reason = next((r.reason for r in results if not r.successful), "set target_x failed")
            self.get_logger().warn(f"nastav_cilove_x failed: {reason}")
        return ok

    def zrus_cilove_x(self) -> bool:
        """Vypne target-X režim (jede neomezeně)."""
        results = self.set_parameters([
            rclpy.parameter.Parameter("use_target_x", rclpy.Parameter.Type.BOOL, False)
        ])
        ok = all(r.successful for r in results)
        if ok:
            self._target_dir = None
        return ok

    def _set_enabled(self, value: bool) -> bool:
        results = self.set_parameters([
            rclpy.parameter.Parameter("enabled", rclpy.Parameter.Type.BOOL, bool(value))
        ])
        ok = all(r.successful for r in results)
        if not ok:
            reason = next((r.reason for r in results if not r.successful), "set enabled failed")
            self.get_logger().warn(f"set_enabled failed: {reason}")
        return ok

    # ---------- Parameter handling (validation + cache) ----------
    def on_params(self, params):
        reset_on_reconf = bool(self.get_parameter("reset_pid_on_reconfigure").value)

        for p in params:
            if p.name == "side":
                v = str(p.value).lower().strip()
                if v not in ("left", "right"):
                    return SetParametersResult(successful=False, reason="side must be 'left' or 'right'")
                self.side = v
                if reset_on_reconf:
                    self._reset_pid()

            elif p.name == "desired_distance":
                v = float(p.value)
                if not (0.01 <= v <= 1.0):
                    return SetParametersResult(successful=False, reason="desired_distance out of range (0.01..1.0)")
                self.desired_distance = v
                if reset_on_reconf:
                    self._reset_pid()

            elif p.name == "enabled":
                self.enabled = bool(p.value)
                if (not self.enabled) and bool(self.get_parameter("reset_pid_on_disable").value):
                    self._reset_pid()

            elif p.name == "use_target_x":
                self.use_target_x = bool(p.value)
                self._target_dir = None

            elif p.name == "target_x":
                self.target_x = float(p.value)
                self._target_dir = None

            elif p.name == "x_tolerance":
                v = float(p.value)
                if not (0.0 < v <= 1.0):
                    return SetParametersResult(successful=False, reason="x_tolerance out of range (0.0..1.0)")
                self.x_tolerance = v

        return SetParametersResult(successful=True)

    # ---------- Service enable/disable ----------
    def handle_enable(self, request: SetBool.Request, response: SetBool.Response):
        ok = self._set_enabled(bool(request.data))
        response.success = ok
        if ok:
            if request.data:
                response.message = "wall follower enabled"
                self._target_dir = None
            else:
                response.message = "wall follower disabled (cmd_vel stopped)"
                self._publish_stop()
        else:
            response.message = "failed to change enabled state"
        return response

    # ---------- Topic commands (side/distance) ----------
    def on_side_cmd(self, msg: String):
        v = msg.data.lower().strip()
        if v not in ("left", "right"):
            self.get_logger().warn(f"Ignoring side='{msg.data}'. Use 'left' or 'right'.")
            return
        self.nastav_stranu(v)

    def on_distance_cmd(self, msg: Float32):
        self.nastav_vzdalenost(float(msg.data))

    # ---------- Odom ----------
    def on_odom(self, msg: Odometry):
        self.current_x = float(msg.pose.pose.position.x)

    # ---------- Scan + control ----------
    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def _reset_pid(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = self.get_clock().now()

    def _range_at_angle(self, scan: LaserScan, angle_rad: float, window: int) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None

        # bounds / wrap for ~360 scanners
        if angle_rad < scan.angle_min or angle_rad > scan.angle_max:
            span = scan.angle_max - scan.angle_min
            if span > 6.0:
                while angle_rad < scan.angle_min:
                    angle_rad += 2.0 * math.pi
                while angle_rad > scan.angle_max:
                    angle_rad -= 2.0 * math.pi
            else:
                return None

        idx = int(round((angle_rad - scan.angle_min) / scan.angle_increment))
        idx = int(clamp(idx, 0, len(scan.ranges) - 1))

        vals = []
        w = int(max(0, window))
        for j in range(idx - w, idx + w + 1):
            if 0 <= j < len(scan.ranges):
                r = scan.ranges[j]
                if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                    vals.append(r)

        if not vals:
            return None
        return float(median(vals))

    def _min_front_distance(self, scan: LaserScan, forward_angle_rad: float, window_rad: float) -> Optional[float]:
        if scan.angle_increment == 0.0:
            return None

        start = forward_angle_rad - window_rad
        end = forward_angle_rad + window_rad

        steps = max(5, int((2.0 * window_rad) / abs(scan.angle_increment)))
        vals = []
        for k in range(steps + 1):
            a = start + (end - start) * (k / steps)
            r = self._range_at_angle(scan, a, window=0)
            if r is not None:
                vals.append(r)

        if not vals:
            return None
        return float(min(vals))

    def _target_x_reached(self) -> bool:
        """Vrací True, pokud je v target-X režimu a dosažen cíl (s tolerancí)."""
        if not self.use_target_x:
            return False
        if self.current_x is None:
            # bez odometrie nemůžeme vyhodnotit cíl
            return False

        x = self.current_x
        tx = self.target_x
        tol = self.x_tolerance

        # Směr k cíli určíme jednou při prvních ticích po enable
        if self._target_dir is None:
            dx = tx - x
            if abs(dx) <= tol:
                self._target_dir = 0
            else:
                self._target_dir = 1 if dx > 0.0 else -1

        if self._target_dir == 0:
            return True

        # pokud jedeme k vyšším X: zastav jakmile x >= target - tol
        if self._target_dir > 0:
            return x >= (tx - tol)
        # pokud jedeme k nižším X: zastav jakmile x <= target + tol
        return x <= (tx + tol)

    def on_timer(self):
        if not self.enabled:
            return

        # Auto-stop at target X
        if self._target_x_reached():
            self.get_logger().info(
                f"Reached target_x={self.target_x:.3f} (x={self.current_x:.3f}, tol={self.x_tolerance:.3f}). Stopping."
            )
            self.zastav_jizdu()
            return

        scan = self.latest_scan
        if scan is None:
            return

        if self.side not in ("left", "right"):
            self._publish_stop()
            return

        side_sign = +1.0 if self.side == "left" else -1.0
        desired = self.desired_distance

        forward_speed = float(self.get_parameter("forward_speed").value)
        forward_angle_deg = float(self.get_parameter("forward_angle_deg").value)

        theta_deg = float(self.get_parameter("theta_deg").value)
        lookahead = float(self.get_parameter("lookahead").value)

        kp = float(self.get_parameter("kp_lat").value)
        ki = float(self.get_parameter("ki_lat").value)
        kd = float(self.get_parameter("kd_lat").value)
        i_lim = float(self.get_parameter("integral_limit").value)

        k_angle = float(self.get_parameter("k_angle").value)

        max_vx = float(self.get_parameter("max_vx").value)
        max_vy = float(self.get_parameter("max_vy").value)
        max_wz = float(self.get_parameter("max_wz").value)

        min_front = float(self.get_parameter("min_front_dist").value)
        front_window_deg = float(self.get_parameter("front_window_deg").value)

        sample_window = int(self.get_parameter("sample_window").value)

        forward_angle = math.radians(forward_angle_deg)
        theta = math.radians(abs(theta_deg))

        a_angle = forward_angle + side_sign * (math.pi / 2.0)
        b_angle = a_angle - side_sign * theta

        a = self._range_at_angle(scan, a_angle, window=sample_window)
        b = self._range_at_angle(scan, b_angle, window=sample_window)

        front_min = self._min_front_distance(scan, forward_angle, math.radians(front_window_deg))
        if front_min is not None and front_min < min_front:
            self._publish_stop()
            return

        if a is None or b is None:
            self._publish_stop()
            return

        denom = a * math.sin(theta)
        if abs(denom) < 1e-6:
            self._publish_stop()
            return

        alpha = math.atan2((a * math.cos(theta) - b), denom)
        dist = a * math.cos(alpha)
        dist_future = dist + lookahead * math.sin(alpha)

        e = dist_future - desired

        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        if dt <= 0.0:
            dt = 1e-3

        self.integral = clamp(self.integral + e * dt, -abs(i_lim), +abs(i_lim))
        de = (e - self.prev_error) / dt

        u = kp * e + ki * self.integral + kd * de

        vy = clamp(side_sign * u, -abs(max_vy), +abs(max_vy))
        wz = clamp(-k_angle * alpha, -abs(max_wz), +abs(max_wz))
        vx = clamp(forward_speed, -abs(max_vx), +abs(max_vx))

        cmd = Twist()
        cmd.linear.x = float(vx)
        cmd.linear.y = float(vy)
        cmd.angular.z = float(wz)
        self.pub_cmd.publish(cmd)

        self.prev_error = e
        self.prev_time = now

    def _publish_stop(self):
        self.pub_cmd.publish(Twist())


def main():
    rclpy.init()
    node = WallFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
