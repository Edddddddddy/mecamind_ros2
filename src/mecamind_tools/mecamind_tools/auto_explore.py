#!/usr/bin/env python3
'''
auto_explore v4.1 -- Wall-follow + Boustrophedon coverage (vacuum-robot style)
Deterministic exploration: trace walls first, then zigzag-cover the interior.
Optimized for Mecanum chassis (strafing for wall distance + stuck escape).
Anti-spin: never rotate in place for more than 2s, always combine with lateral.
Physical-stuck detection: if wheels spin but robot doesn't move, lateral escape.
'''
import math, time, os, subprocess, rclpy
from pathlib import Path
import numpy as np
from collections import deque
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def angle_diff(a, b):
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


def quat_to_yaw(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def astar_grid(grid, start_rc, goal_rc, inflation=2):
    '''BFS on occupancy grid with inflated obstacles. Returns [(row,col),...] or None.'''
    h, w = grid.shape
    sr, sc = int(np.clip(start_rc[0], 0, h-1)), int(np.clip(start_rc[1], 0, w-1))
    gr, gc = int(np.clip(goal_rc[0], 0, h-1)), int(np.clip(goal_rc[1], 0, w-1))

    obstacles = (grid > 50) | (grid == -1)
    if inflation > 0:
        from scipy.ndimage import binary_dilation
        obstacles = binary_dilation(obstacles, iterations=inflation)

    if obstacles[sr, sc] or obstacles[gr, gc]:
        obstacles = (grid > 50)
        if obstacles[sr, sc] or obstacles[gr, gc]:
            return None

    visited = np.zeros((h, w), dtype=bool)
    visited[sr, sc] = True
    parent = {}
    queue = deque([(sr, sc)])
    found = False
    dirs = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while queue:
        r, c = queue.popleft()
        if abs(r - gr) <= 1 and abs(c - gc) <= 1:
            parent[(gr, gc)] = (r, c)
            found = True
            break
        for dr, dc in dirs:
            nr, nc = r + dr, c + dc
            if 0 <= nr < h and 0 <= nc < w and not visited[nr, nc] and not obstacles[nr, nc]:
                visited[nr, nc] = True
                parent[(nr, nc)] = (r, c)
                queue.append((nr, nc))

    if not found:
        return None

    path = []
    cur = (gr, gc)
    while cur != (sr, sc):
        path.append(cur)
        cur = parent.get(cur)
        if cur is None:
            break
    path.reverse()
    return path


# ─── Phases ───
PHASE_INIT = 'INIT'
PHASE_WALL_FOLLOW = 'WALL_FOLLOW'
PHASE_CROSS_EXPLORE = 'CROSS_EXPLORE'
PHASE_PLAN_COVERAGE = 'PLAN_COVERAGE'
PHASE_BOUSTROPHEDON = 'BOUSTROPHEDON'
PHASE_GAP_FILL = 'GAP_FILL'
PHASE_COMPLETE = 'COMPLETE'


class AutoExploreNode(Node):
    def __init__(self, name):
        super().__init__(name, allow_undeclared_parameters=True)
        params = [
            ('linear_speed', 0.10), ('angular_speed', 0.6),
            ('linear_speed_slow', 0.05), ('angular_speed_slow', 0.35),
            ('lateral_speed', 0.08),
            ('obstacle_stop_dist', 0.07), ('obstacle_slow_dist', 0.15),
            ('obstacle_side_dist', 0.05),
            ('goal_tolerance', 0.20),
            ('wall_follow_dist', 0.25),
            ('wall_follow_kp', 1.0),
            ('wall_follow_kd', 0.3),
            ('wall_follow_side', 'right'),
            ('wall_loop_close_dist', 0.4),
            ('wall_min_duration', 20.0),
            ('row_spacing', 0.30),
            ('waypoint_tolerance', 0.15),
            ('auto_save_map', True),
            ('map_save_path', str(Path('~/.ros/mecamind_three_room_map').expanduser())),
            ('startup_wait_sec', 8.0),
            ('progress_report_interval', 10.0),
            ('frontier_cluster_dist', 0.3),
            ('min_frontier_size', 3),
            ('min_frontier_distance', 0.5),
            ('frontier_blacklist_dist', 0.55),
            ('max_gap_fill_failures', 18),
            ('gap_fill_timeout_sec', 240.0),
            ('min_unknown_reduction', 8),
            ('cross_goal_timeout_sec', 150.0),
            ('cross_goal_stall_sec', 35.0),
            ('cross_goal_min_progress', 0.25),
            ('gap_goal_timeout_sec', 120.0),
            ('gap_goal_stall_sec', 40.0),
            ('boustrophedon_stall_sec', 35.0),
            ('boustrophedon_min_coverage_gain', 0.2),
            ('boustrophedon_max_skips', 6),
            ('boundary_revisit_enabled', True),
            ('boundary_revisit_margin', 0.45),
            ('max_boundary_revisits', 4),
            ('cmd_topic', '/controller/cmd_vel'),
            ('checkpoint_save_interval_sec', 90.0),
        ]
        for n, v in params:
            self.declare_parameter(n, v)

        self.map_data = None
        self.map_info = None
        self.laser_ranges = None
        self.scan_angle_min = -math.pi
        self.scan_angle_increment = 0.0
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.running = False
        self._map_received = False
        self._map_saved = False
        self._start_time = None
        self._last_progress_report = 0.0
        self._last_checkpoint_save = 0.0
        self._node_start_time = time.time()

        self.phase = PHASE_INIT

        # Wall-follow state
        self._wf_start_pos = None
        self._wf_start_time = 0.0
        self._wf_prev_error = 0.0
        self._wf_corner_count = 0
        self._wf_last_coverage = 0.0
        self._wf_coverage_stall_time = 0.0

        # Boustrophedon state
        self._coverage_path = []
        self._coverage_idx = 0
        self._wp_start_time = 0.0
        self._wp_start_dist = 999.0
        self._boustro_last_coverage = 0.0
        self._boustro_last_gain_time = 0.0
        self._boustro_skip_count = 0

        # A* local path
        self._local_path = []
        self._local_path_idx = 0

        # Cross-explore state (navigate to far side)
        self._cross_goal = None
        self._cross_attempts = 0
        self._cross_round = 0
        self._cross_goal_started = 0.0
        self._cross_goal_last_progress = 0.0
        self._cross_goal_best_dist = float('inf')

        # Gap-fill state
        self._gap_goal = None
        self._gap_attempts = 0
        self._gap_failed_goals = []
        self._gap_started_time = 0.0
        self._gap_unknown_at_goal = None
        self._gap_goal_started = 0.0
        self._gap_goal_last_progress = 0.0
        self._gap_goal_best_dist = float('inf')
        self._completion_reason = ''
        self._boundary_revisit_goals = []

        # Anti-spin & physical-stuck tracking
        self._cmd_time = 0.0
        self._cmd_pos = (0.0, 0.0)
        self._spin_start = 0.0
        self._spinning = False
        self._phys_stuck_time = 0.0
        self._phys_stuck_pos = (0.0, 0.0)
        self._phys_stuck_escape = False
        self._phys_escape_start = 0.0
        self._phys_escape_dir = 1.0
        self._phys_stuck_count = 0
        self._phys_last_stuck_time = 0.0
        # General navigation stuck
        self._nav_stuck_time = 0.0
        self._nav_stuck_pos = (0.0, 0.0)

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, map_qos)
        self.scan_sub = self.create_subscription(LaserScan, '/scan_raw', self.scan_callback, 1)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 1)
        self.cmd_pub = self.create_publisher(Twist, str(self._p('cmd_topic')), 1)
        self.create_service(Trigger, '~/start', self.start_cb)
        self.create_service(Trigger, '~/stop', self.stop_cb)
        self.create_service(Trigger, '~/save_map', self.save_map_cb)
        self.create_service(Trigger, '~/init_finish', self.state_cb)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.auto_start_timer = self.create_timer(1.0, self._check_auto_start)
        self.get_logger().info(
            '\033[1;36mAutoExplore v4.1 ready (wall-follow + boustrophedon + anti-spin)\033[0m')

    def _p(self, n):
        return self.get_parameter(n).value

    def _coverage_percent(self):
        if self.map_data is None:
            return 0.0
        g = self.map_data
        return (np.sum(g == 0) + np.sum(g > 50)) / g.size * 100.0

    # ─── ROS Callbacks ───

    def _check_auto_start(self):
        if self.running:
            return

        now = time.time()
        if self._map_received:
            self.get_logger().info('\033[1;32mMap received - auto-starting!\033[0m')
        elif now - self._node_start_time < self._p('startup_wait_sec'):
            return
        else:
            self.get_logger().warn(
                '\033[1;33mMap not received yet - starting exploration to warm up SLAM.\033[0m')

        self.running = True
        self._start_time = now
        self._last_checkpoint_save = now
        self.phase = PHASE_WALL_FOLLOW
        self._wf_start_pos = (self.robot_x, self.robot_y)
        self._wf_start_time = now
        self._phys_stuck_time = now
        self._phys_stuck_pos = (self.robot_x, self.robot_y)
        self.auto_start_timer.cancel()

    def start_cb(self, req, resp):
        self.running = True
        self._map_saved = False
        self._start_time = time.time()
        self._last_checkpoint_save = self._start_time
        self.phase = PHASE_WALL_FOLLOW
        self._wf_start_pos = (self.robot_x, self.robot_y)
        self._wf_start_time = time.time()
        resp.success = True
        return resp

    def stop_cb(self, req, resp):
        self.running = False
        self.cmd_pub.publish(Twist())
        self._try_save_map('stop_requested')
        resp.success = True
        return resp

    def save_map_cb(self, req, resp):
        ok = self._try_save_map('manual')
        resp.success = ok
        resp.message = 'map saved' if ok else 'save failed'
        return resp

    def state_cb(self, req, resp):
        resp.success = True
        resp.message = self.phase
        return resp

    def map_callback(self, msg):
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)
        self.map_info = msg.info
        if not self._map_received:
            self.get_logger().info('\033[1;32m[Map] First map received.\033[0m')
        self._map_received = True

    def scan_callback(self, msg):
        self.laser_ranges = np.array(msg.ranges, dtype=np.float32)
        self.scan_angle_min = float(msg.angle_min)
        self.scan_angle_increment = float(msg.angle_increment)

    def _scan_sector(self, center_angle, half_width):
        """Return scan values in a robot-relative angular sector."""
        if self.laser_ranges is None or len(self.laser_ranges) == 0:
            return np.array([], dtype=np.float32)

        increment = self.scan_angle_increment
        if abs(increment) < 1e-9:
            increment = (2.0 * math.pi) / len(self.laser_ranges)

        indices = np.arange(len(self.laser_ranges), dtype=np.float32)
        angles = self.scan_angle_min + indices * increment
        delta = np.arctan2(
            np.sin(angles - center_angle),
            np.cos(angles - center_angle),
        )
        return self.laser_ranges[np.abs(delta) <= half_width]

    def _clean_scan_sector(self, center_angle, half_width):
        values = self._scan_sector(center_angle, half_width)
        return values[(values > 0.02) & (values < 12.0)]

    def odom_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)

    # ─── Main Control Loop ───

    def control_loop(self):
        if not self.running or self.laser_ranges is None:
            return
        now = time.time()
        self._report_progress(now)

        if self.phase == PHASE_COMPLETE:
            self.cmd_pub.publish(Twist())
            return

        # Physical stuck escape takes priority over everything
        if self._phys_stuck_escape:
            self._do_physical_escape(now)
            return

        # Check if physically stuck (sending commands but not moving)
        if self._check_physical_stuck(now):
            return

        if self.phase == PHASE_WALL_FOLLOW:
            self._wall_follow_step(now)
        elif self.phase == PHASE_CROSS_EXPLORE:
            self._cross_explore_step(now)
        elif self.phase == PHASE_PLAN_COVERAGE:
            self._plan_coverage()
        elif self.phase == PHASE_BOUSTROPHEDON:
            self._boustrophedon_step(now)
        elif self.phase == PHASE_GAP_FILL:
            self._gap_fill_step(now)
        elif self.phase == PHASE_COMPLETE:
            self.cmd_pub.publish(Twist())

    # ─── Physical Stuck Detection ───

    def _check_physical_stuck(self, now):
        '''Detect: wheels spinning but robot not moving (odom unchanged).'''
        moved = math.hypot(
            self.robot_x - self._phys_stuck_pos[0],
            self.robot_y - self._phys_stuck_pos[1])

        if moved > 0.08:
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            self._phys_stuck_time = now
            if now - self._phys_last_stuck_time > 20.0:
                self._phys_stuck_count = 0
            return False

        # If not moved 8cm in 5 seconds → physically stuck
        if now - self._phys_stuck_time > 5.0:
            self._phys_stuck_count += 1
            self._phys_last_stuck_time = now
            self.get_logger().warn(
                f'\033[1;31m[STUCK] Wheels spinning but not moving! '
                f'Escape #{self._phys_stuck_count}...\033[0m')
            self._phys_stuck_escape = True
            self._phys_escape_start = now
            # Choose escape direction: alternate if repeated stuck
            side_half_width = math.pi / 4.0
            left_values = self._clean_scan_sector(math.pi / 2.0, side_half_width)
            right_values = self._clean_scan_sector(-math.pi / 2.0, side_half_width)
            left_clear = float(np.median(left_values)) if len(left_values) else 0.0
            right_clear = float(np.median(right_values)) if len(right_values) else 0.0
            if self._phys_stuck_count % 2 == 1:
                self._phys_escape_dir = 1.0 if left_clear > right_clear else -1.0
            else:
                self._phys_escape_dir = -1.0 if left_clear > right_clear else 1.0
            self._phys_stuck_time = now
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            return True
        return False

    def _do_physical_escape(self, now):
        '''Escalating escape: longer and more aggressive with repeated stucks.'''
        twist = Twist()
        elapsed = now - self._phys_escape_start
        lat_speed = self._p('lateral_speed')
        lin_speed = self._p('linear_speed')

        # Escalate duration based on consecutive stuck count
        esc_duration = min(2.0 + self._phys_stuck_count * 1.0, 6.0)
        phase1_end = esc_duration * 0.5
        phase2_end = esc_duration

        if elapsed < phase1_end:
            # Phase 1: strong backward + lateral
            twist.linear.y = self._phys_escape_dir * lat_speed * 2.0
            twist.linear.x = -lin_speed * 0.8
            twist.angular.z = self._phys_escape_dir * 0.3
        elif elapsed < phase2_end:
            # Phase 2: forward at an angle (diagonal escape)
            twist.linear.x = lin_speed * 1.2
            twist.linear.y = self._phys_escape_dir * lat_speed * 1.0
            twist.angular.z = -self._phys_escape_dir * 0.2
        else:
            # Done escaping
            self._phys_stuck_escape = False
            self._phys_stuck_time = now
            self._phys_stuck_pos = (self.robot_x, self.robot_y)
            self._local_path = []
            self.get_logger().info(
                f'[STUCK] Escape #{self._phys_stuck_count} done '
                f'({esc_duration:.1f}s), resuming...')
            self.cmd_pub.publish(Twist())
            return

        self.cmd_pub.publish(twist)

    # ─── Phase 1: Wall Follow ───

    def _wall_follow_step(self, now):
        twist = Twist()
        ranges = self.laser_ranges
        n = len(ranges)

        target_d = self._p('wall_follow_dist')
        kp = self._p('wall_follow_kp')
        kd = self._p('wall_follow_kd')
        follow_right = (self._p('wall_follow_side') == 'right')

        if n == 0:
            return

        increment = abs(self.scan_angle_increment)
        if increment < 1e-9:
            increment = (2.0 * math.pi) / n
        sector_half_width = max(0.12, min(math.pi / 3.0, (n // 8) * increment / 2.0))
        front_values = self._clean_scan_sector(0.0, sector_half_width)
        front_min = float(np.min(front_values)) if len(front_values) else np.inf

        if follow_right:
            side_range = self._clean_scan_sector(-math.pi / 2.0, math.pi / 4.0)
            fr_range = self._clean_scan_sector(-math.pi / 4.0, math.pi / 8.0)
        else:
            side_range = self._clean_scan_sector(math.pi / 2.0, math.pi / 4.0)
            fr_range = self._clean_scan_sector(math.pi / 4.0, math.pi / 8.0)

        side_min = float(np.min(side_range)) if len(side_range) else np.inf
        fr_min = float(np.min(fr_range)) if len(fr_range) else np.inf

        stop_d = self._p('obstacle_stop_dist')
        speed = self._p('linear_speed')
        ang_speed = self._p('angular_speed')

        # Check loop closure
        elapsed = now - self._wf_start_time
        if elapsed > self._p('wall_min_duration') and self._wf_start_pos:
            dist_to_start = math.hypot(
                self.robot_x - self._wf_start_pos[0],
                self.robot_y - self._wf_start_pos[1])
            if dist_to_start < self._p('wall_loop_close_dist'):
                self.get_logger().info(
                    f'\033[1;32m[WallFollow] Loop closed! '
                    f'({elapsed:.0f}s, corners={self._wf_corner_count})\033[0m')
                self.phase = PHASE_CROSS_EXPLORE
                self._cross_goal = None
                self._cross_attempts = 0
                self.cmd_pub.publish(Twist())
                return

        # Coverage stall: if coverage hasn't grown 1% in 60s, end wall-follow
        if self.map_data is not None:
            g = self.map_data
            coverage = (np.sum(g == 0) + np.sum(g > 50)) / g.size * 100.0
            if coverage - self._wf_last_coverage > 1.0:
                self._wf_last_coverage = coverage
                self._wf_coverage_stall_time = now
            elif now - self._wf_coverage_stall_time > 40.0 and elapsed > 30.0:
                self.get_logger().info(
                    f'[WallFollow] Coverage stalled at {coverage:.1f}%, '
                    f'navigating to far side...')
                self.phase = PHASE_CROSS_EXPLORE
                self._cross_goal = None
                self._cross_attempts = 0
                self.cmd_pub.publish(Twist())
                return

        # Hard timeout
        if elapsed > 300.0:
            self.get_logger().warn('[WallFollow] Timeout (5min), navigating to far side...')
            self.phase = PHASE_CROSS_EXPLORE
            self._cross_goal = None
            self._cross_attempts = 0
            self.cmd_pub.publish(Twist())
            return

        # ─── Wall Follow Control ───
        # Front blocked → turn + LATERAL (never pure spin!)
        if front_min < stop_d * 2.5:
            turn_dir = 1.0 if follow_right else -1.0
            twist.angular.z = turn_dir * ang_speed * 0.7
            # Always add lateral to avoid pure spinning
            lat_dir = -1.0 if follow_right else 1.0
            twist.linear.y = lat_dir * self._p('lateral_speed')
            twist.linear.x = -0.02  # tiny reverse to unstick
            self._wf_corner_count += 1
            self.cmd_pub.publish(twist)
            return

        # Front-right close → slight turn + forward
        if fr_min < target_d * 0.8:
            turn_dir = 1.0 if follow_right else -1.0
            twist.angular.z = turn_dir * self._p('angular_speed_slow')
            twist.linear.x = speed * 0.7
            self.cmd_pub.publish(twist)
            return

        # PD control on side wall distance
        error = target_d - side_min
        d_error = error - self._wf_prev_error
        self._wf_prev_error = error
        correction = kp * error + kd * d_error

        twist.linear.x = speed

        if side_min > target_d * 3.0:
            # Lost wall → turn toward it + forward (NOT pure rotation)
            turn_dir = -1.0 if follow_right else 1.0
            twist.angular.z = turn_dir * self._p('angular_speed_slow')
            twist.linear.x = speed * 0.6
        else:
            lat_speed = self._p('lateral_speed')
            if follow_right:
                twist.linear.y = -np.clip(correction, -lat_speed, lat_speed)
            else:
                twist.linear.y = np.clip(correction, -lat_speed, lat_speed)
            twist.angular.z = -correction * 0.5 if follow_right else correction * 0.5
            twist.angular.z = np.clip(twist.angular.z, -0.3, 0.3)

        # Slow down near front obstacles
        if front_min < self._p('obstacle_slow_dist'):
            twist.linear.x *= max(0.3, (front_min - stop_d) /
                                   (self._p('obstacle_slow_dist') - stop_d + 0.001))

        self.cmd_pub.publish(twist)

    # ─── Phase 1.5: Cross-Explore (go to far side) ───

    def _cross_explore_step(self, now):
        '''Navigate to the farthest frontier cluster to ensure both sides explored.'''
        if self._cross_goal is not None:
            dist = math.hypot(
                self._cross_goal[0] - self.robot_x,
                self._cross_goal[1] - self.robot_y)
            if dist < self._p('goal_tolerance') * 2.0:
                self.get_logger().info(
                    f'[CrossExplore] Reached far side, round {self._cross_round+1} done!')
                self._cross_goal = None
                self._cross_round += 1
                self._local_path = []
                self.cmd_pub.publish(Twist())
                # Do a short wall-follow on the new side if first round
                if self._cross_round <= 2:
                    next_goal = self._find_farthest_frontier()
                    if next_goal is not None:
                        dist_to_next = math.hypot(
                            next_goal[0] - self.robot_x,
                            next_goal[1] - self.robot_y)
                        if dist_to_next > 0.8:
                            self._cross_goal = next_goal
                            self._cross_attempts = 0
                            self._cross_goal_started = now
                            self._cross_goal_last_progress = now
                            self._cross_goal_best_dist = dist_to_next
                            self._nav_stuck_time = now
                            self._nav_stuck_pos = (self.robot_x, self.robot_y)
                            self.get_logger().info(
                                f'[CrossExplore] Round {self._cross_round+1}: '
                                f'next frontier at ({next_goal[0]:.1f},{next_goal[1]:.1f}), '
                                f'd={dist_to_next:.1f}m')
                            return
                self.phase = PHASE_PLAN_COVERAGE
                return

            # Check if stuck trying to reach cross-goal
            if (
                self._cross_goal_started > 0.0
                and now - self._cross_goal_started > self._p('cross_goal_timeout_sec')
            ):
                self.get_logger().warn(
                    '[CrossExplore] Goal time budget exhausted, '
                    'planning coverage from the current map')
                self._cross_goal = None
                self._local_path = []
                self.phase = PHASE_PLAN_COVERAGE
                self.cmd_pub.publish(Twist())
                return

            if dist < self._cross_goal_best_dist - self._p('cross_goal_min_progress'):
                self._cross_goal_best_dist = dist
                self._cross_goal_last_progress = now
            elif (
                self._cross_goal_last_progress > 0.0
                and now - self._cross_goal_last_progress > self._p('cross_goal_stall_sec')
            ):
                self.get_logger().warn(
                    '[CrossExplore] Goal distance is not improving, '
                    'planning coverage from the current map')
                self._cross_goal = None
                self._local_path = []
                self.phase = PHASE_PLAN_COVERAGE
                self.cmd_pub.publish(Twist())
                return

            moved = math.hypot(
                self.robot_x - self._nav_stuck_pos[0],
                self.robot_y - self._nav_stuck_pos[1])
            if now - self._nav_stuck_time > 12.0:
                if moved < 0.08:
                    self._cross_attempts += 1
                    if self._cross_attempts >= 3:
                        self.get_logger().warn(
                            '[CrossExplore] Cannot reach far side, '
                            'planning coverage from here...')
                        self._cross_goal = None
                        self._local_path = []
                        self.phase = PHASE_PLAN_COVERAGE
                        self.cmd_pub.publish(Twist())
                        return
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)

            nav_target = self._get_next_local_waypoint(self._cross_goal)
            twist = self._navigate_to_point(nav_target)
            self.cmd_pub.publish(twist)
            return

        # Find farthest frontier cluster
        goal = self._find_farthest_frontier()
        if goal is None:
            self.get_logger().info('[CrossExplore] No far frontiers, planning coverage...')
            self.phase = PHASE_PLAN_COVERAGE
            return

        self._cross_goal = goal
        self._cross_attempts = 0
        self._cross_goal_started = now
        self._local_path = []
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        dist = math.hypot(goal[0] - self.robot_x, goal[1] - self.robot_y)
        self._cross_goal_last_progress = now
        self._cross_goal_best_dist = dist
        self.get_logger().info(
            f'\033[1;35m[CrossExplore] Navigating to far frontier '
            f'({goal[0]:.1f},{goal[1]:.1f}), d={dist:.1f}m\033[0m')

    def _find_farthest_frontier(self):
        '''Find the farthest large frontier cluster from current position.'''
        if self.map_data is None:
            return None
        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        unk = (g == -1)
        unknown_neighbors = np.zeros_like(g, dtype=np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                unknown_neighbors += (
                    np.roll(np.roll(unk, dy, axis=0), dx, axis=1)
                ).astype(np.int16)

        # Navigate to a known free cell next to unknown space. Driving
        # directly into an unknown cell makes A* reject the goal and causes
        # repeated attempts against the same wall.
        fm = (g == 0) & (unknown_neighbors > 0)
        fm[:2, :] = fm[-2:, :] = fm[:, :2] = fm[:, -2:] = False

        if np.sum(fm) < 5:
            return None

        fy, fx = np.where(fm)
        wx = ox + (fx + 0.5) * res
        wy = oy + (fy + 0.5) * res
        pts = np.column_stack([wx, wy])

        clusters = self._grid_cluster(pts, 0.4)
        if not clusters:
            return None

        rx, ry = self.robot_x, self.robot_y
        # Pick the farthest cluster that's large enough
        scored = []
        for cx, cy in clusters:
            dd = math.hypot(cx - rx, cy - ry)
            if dd > 0.5 and self._is_reachable_goal(cx, cy):
                scored.append((dd, cx, cy))

        if not scored:
            return None

        scored.sort(key=lambda s: -s[0])  # farthest first
        return (scored[0][1], scored[0][2])

    # ─── Phase 2: Plan Coverage Path ───

    def _plan_coverage(self):
        if self.map_data is None:
            return

        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        free_mask = (g == 0)
        if np.sum(free_mask) < 10:
            self.phase = PHASE_GAP_FILL
            return

        # Include frontier cells (unknown adjacent to free) in coverage bounds
        unk_mask = (g == -1)
        frontier_mask = np.zeros_like(g, dtype=bool)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                shifted_free = np.roll(np.roll(free_mask, dy, axis=0), dx, axis=1)
                frontier_mask |= (unk_mask & shifted_free)
        coverage_mask = free_mask | frontier_mask

        fy, fx = np.where(coverage_mask)
        row_spacing = self._p('row_spacing')

        x_min_w = ox + float(np.min(fx)) * res + row_spacing
        x_max_w = ox + float(np.max(fx)) * res - row_spacing
        y_min_w = oy + float(np.min(fy)) * res + row_spacing
        y_max_w = oy + float(np.max(fy)) * res - row_spacing

        if x_max_w <= x_min_w or y_max_w <= y_min_w:
            self.phase = PHASE_GAP_FILL
            return

        # Sort waypoints: start from nearest corner
        corners = [
            (x_min_w, y_min_w), (x_max_w, y_min_w),
            (x_min_w, y_max_w), (x_max_w, y_max_w)]
        corners.sort(key=lambda c: math.hypot(c[0]-self.robot_x, c[1]-self.robot_y))
        start_x, start_y = corners[0]
        going_right = (start_x == x_min_w)
        start_from_bottom = (start_y == y_min_w)

        path = []
        ys = np.arange(y_min_w, y_max_w + 0.01, row_spacing)
        if not start_from_bottom:
            ys = ys[::-1]

        for y_val in ys:
            if going_right:
                path.append((x_min_w, float(y_val)))
                path.append((x_max_w, float(y_val)))
            else:
                path.append((x_max_w, float(y_val)))
                path.append((x_min_w, float(y_val)))
            going_right = not going_right

        # Filter waypoints inside walls
        filtered_path = []
        for wx, wy in path:
            mx = int(np.clip((wx - ox) / res, 0, w - 1))
            my = int(np.clip((wy - oy) / res, 0, h - 1))
            y_lo = max(0, my - 2); y_hi = min(h, my + 3)
            x_lo = max(0, mx - 2); x_hi = min(w, mx + 3)
            region = g[y_lo:y_hi, x_lo:x_hi]
            if np.any(region == 0) or np.any(region == -1):
                filtered_path.append((wx, wy))

        self._coverage_path = filtered_path
        self._coverage_idx = 0
        self._local_path = []
        self._nav_stuck_time = time.time()
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._boustro_last_coverage = self._coverage_percent()
        self._boustro_last_gain_time = time.time()
        self._boustro_skip_count = 0

        self.get_logger().info(
            f'\033[1;34m[Coverage] Planned {len(filtered_path)} waypoints '
            f'({len(ys)} rows)\033[0m')

        self.phase = PHASE_BOUSTROPHEDON if filtered_path else PHASE_GAP_FILL

    # ─── Phase 3: Boustrophedon Execution ───

    def _boustrophedon_step(self, now):
        if self._coverage_idx >= len(self._coverage_path):
            self.get_logger().info(
                '\033[1;32m[Boustrophedon] Coverage path complete!\033[0m')
            self.phase = PHASE_GAP_FILL
            self._local_path = []
            self.cmd_pub.publish(Twist())
            return

        coverage = self._coverage_percent()
        if coverage - self._boustro_last_coverage >= self._p('boustrophedon_min_coverage_gain'):
            self._boustro_last_coverage = coverage
            self._boustro_last_gain_time = now
            self._boustro_skip_count = 0
        elif (
            now - self._boustro_last_gain_time > self._p('boustrophedon_stall_sec')
            or self._boustro_skip_count >= int(self._p('boustrophedon_max_skips'))
        ):
            self.get_logger().warn(
                '[Boustrophedon] Coverage stalled, switching to gap fill...')
            self.phase = PHASE_GAP_FILL
            self._gap_goal = None
            self._local_path = []
            self._nav_stuck_time = now
            self._nav_stuck_pos = (self.robot_x, self.robot_y)
            self.cmd_pub.publish(Twist())
            return

        goal = self._coverage_path[self._coverage_idx]
        dist = math.hypot(goal[0] - self.robot_x, goal[1] - self.robot_y)

        if dist < self._p('waypoint_tolerance'):
            self._advance_waypoint(now)
            return

        # Track time spent on this waypoint
        if self._wp_start_time == 0.0 or self._wp_start_dist == 999.0:
            self._wp_start_time = now
            self._wp_start_dist = dist

        # Not approaching goal: if after 15s we haven't cut distance by 30%, skip
        wp_elapsed = now - self._wp_start_time
        if wp_elapsed > 15.0:
            if dist > self._wp_start_dist * 0.7:
                self.get_logger().warn(
                    f'[Boustrophedon] No progress toward wp {self._coverage_idx} '
                    f'(d={dist:.2f}m, was {self._wp_start_dist:.2f}m), skipping...')
                self._boustro_skip_count += 1
                self._advance_waypoint(now)
                self.cmd_pub.publish(Twist())
                return
            # Reset for next check window
            self._wp_start_time = now
            self._wp_start_dist = dist

        # Absolute stuck: not moved at all for 6s
        moved = math.hypot(
            self.robot_x - self._nav_stuck_pos[0],
            self.robot_y - self._nav_stuck_pos[1])
        if now - self._nav_stuck_time > 6.0:
            if moved < 0.05:
                self.get_logger().warn(
                    f'[Boustrophedon] Stuck at wp {self._coverage_idx}, skipping...')
                self._boustro_skip_count += 1
                self._advance_waypoint(now)
                self.cmd_pub.publish(Twist())
                return
            self._nav_stuck_time = now
            self._nav_stuck_pos = (self.robot_x, self.robot_y)

        nav_target = self._get_next_local_waypoint(goal)
        twist = self._navigate_to_point(nav_target)
        self.cmd_pub.publish(twist)

    def _advance_waypoint(self, now):
        self._coverage_idx += 1
        self._local_path = []
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._wp_start_time = 0.0
        self._wp_start_dist = 999.0

    def _get_next_local_waypoint(self, final_goal):
        '''A* path planning to goal, returns next intermediate waypoint.'''
        if self._local_path and self._local_path_idx < len(self._local_path):
            wp = self._local_path[self._local_path_idx]
            dist = math.hypot(wp[0] - self.robot_x, wp[1] - self.robot_y)
            if dist < self._p('waypoint_tolerance') * 1.5:
                self._local_path_idx += 1
                if self._local_path_idx < len(self._local_path):
                    return self._local_path[self._local_path_idx]
                return final_goal
            return wp

        if self.map_data is None or self.map_info is None:
            return final_goal

        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape

        sc = int((self.robot_x - ox) / res)
        sr = int((self.robot_y - oy) / res)
        gc = int((final_goal[0] - ox) / res)
        gr = int((final_goal[1] - oy) / res)

        path_cells = astar_grid(self.map_data, (sr, sc), (gr, gc), inflation=2)
        if path_cells is None or len(path_cells) < 2:
            return final_goal

        step = max(1, int(0.25 / res))
        self._local_path = []
        for i in range(0, len(path_cells), step):
            r, c = path_cells[i]
            wx = ox + (c + 0.5) * res
            wy = oy + (r + 0.5) * res
            self._local_path.append((wx, wy))
        self._local_path.append(final_goal)
        self._local_path_idx = 0

        return self._local_path[0] if self._local_path else final_goal

    # ─── Phase 4: Gap Fill ───

    def _gap_fill_step(self, now):
        if self._gap_started_time == 0.0:
            self._gap_started_time = now

        if (
            now - self._gap_started_time > self._p('gap_fill_timeout_sec')
            or len(self._gap_failed_goals) >= self._p('max_gap_fill_failures')
        ):
            self._finish_exploration('gap_fill_budget_exhausted')
            return

        if self._gap_goal is not None:
            dist = math.hypot(
                self._gap_goal[0] - self.robot_x,
                self._gap_goal[1] - self.robot_y)
            if dist < self._p('goal_tolerance'):
                unknown_now = int(np.sum(self.map_data == -1)) if self.map_data is not None else 0
                unknown_delta = (
                    self._gap_unknown_at_goal - unknown_now
                    if self._gap_unknown_at_goal is not None
                    else 0
                )
                if (
                    self._gap_unknown_at_goal is not None
                    and unknown_delta >= self._p('min_unknown_reduction')
                ):
                    self.get_logger().info(
                        f'[GapFill] Frontier visit reduced unknown cells by '
                        f'{unknown_delta}')
                else:
                    self._gap_failed_goals.append(self._gap_goal)
                    self.get_logger().warn(
                        f'[GapFill] Frontier yielded limited gain '
                        f'({unknown_delta} unknown cells), blacklisting it '
                        f'({len(self._gap_failed_goals)}/'
                        f'{int(self._p("max_gap_fill_failures"))})')
                self._gap_goal = None
                self._gap_attempts = 0
                self._local_path = []
                return

            if dist < self._gap_goal_best_dist - 0.15:
                self._gap_goal_best_dist = dist
                self._gap_goal_last_progress = now
            elif (
                now - self._gap_goal_started > self._p('gap_goal_timeout_sec')
                or now - self._gap_goal_last_progress > self._p('gap_goal_stall_sec')
            ):
                self._gap_failed_goals.append(self._gap_goal)
                self.get_logger().warn(
                    f'[GapFill] Goal made insufficient progress, switching '
                    f'frontier ({len(self._gap_failed_goals)}/'
                    f'{int(self._p("max_gap_fill_failures"))})')
                self._gap_goal = None
                self._gap_attempts = 0
                self._local_path = []
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)
                self.cmd_pub.publish(Twist())
                return

            moved = math.hypot(
                self.robot_x - self._nav_stuck_pos[0],
                self.robot_y - self._nav_stuck_pos[1])
            if now - self._nav_stuck_time > 10.0:
                if moved < 0.08:
                    self._gap_attempts += 1
                    if self._gap_attempts >= 3:
                        self._gap_failed_goals.append(self._gap_goal)
                        self.get_logger().warn(
                            f'[GapFill] Goal unreachable, skipping '
                            f'({len(self._gap_failed_goals)}/'
                            f'{int(self._p("max_gap_fill_failures"))})')
                        self._gap_goal = None
                        self._gap_attempts = 0
                        self._local_path = []
                        self._nav_stuck_time = now
                        self._nav_stuck_pos = (self.robot_x, self.robot_y)
                        self.cmd_pub.publish(Twist())
                        return
                self._nav_stuck_time = now
                self._nav_stuck_pos = (self.robot_x, self.robot_y)

            nav_target = self._get_next_local_waypoint(self._gap_goal)
            twist = self._navigate_to_point(nav_target)
            self.cmd_pub.publish(twist)
            return

        goal = self._find_frontier_goal()
        if goal is None:
            goal = self._find_boundary_revisit_goal()
            if goal is None:
                self._finish_exploration('no_reachable_frontiers')
                return
            self._boundary_revisit_goals.append(goal)
            self.get_logger().warn(
                f'[BoundaryRevisit] No frontier selected; revisiting map edge '
                f'at ({goal[0]:.2f},{goal[1]:.2f}) before accepting map.')

        self._gap_goal = goal
        self._gap_attempts = 0
        self._local_path = []
        self._nav_stuck_time = now
        self._nav_stuck_pos = (self.robot_x, self.robot_y)
        self._gap_unknown_at_goal = (
            int(np.sum(self.map_data == -1)) if self.map_data is not None else None
        )
        self._gap_goal_started = now
        self._gap_goal_last_progress = now
        self._gap_goal_best_dist = math.hypot(
            goal[0] - self.robot_x, goal[1] - self.robot_y)

    def _finish_exploration(self, reason):
        """Stop exploration once the frontier budget is exhausted or closed."""
        self._completion_reason = reason
        unknown = int(np.sum(self.map_data == -1)) if self.map_data is not None else -1
        if reason == 'no_reachable_frontiers':
            self.get_logger().info(
                f'\033[1;32m=== Exploration COMPLETE! === '
                f'(reason={reason}, unknown={unknown})\033[0m')
        else:
            self.get_logger().warn(
                f'\033[1;33m=== Exploration STOPPED WITH BEST-EFFORT MAP === '
                f'(reason={reason}, unknown={unknown})\033[0m')
        self.phase = PHASE_COMPLETE
        self._try_save_map(f'exploration_{reason}')
        self.cmd_pub.publish(Twist())

    # ─── Navigation ───

    def _is_reachable_goal(self, goal_x, goal_y):
        """Check reachability using only known free cells in the current map."""
        if self.map_data is None or self.map_info is None:
            return True
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        h, w = self.map_data.shape
        start = (
            int((self.robot_y - oy) / res),
            int((self.robot_x - ox) / res),
        )
        goal = (
            int((goal_y - oy) / res),
            int((goal_x - ox) / res),
        )
        path = astar_grid(self.map_data, start, goal, inflation=2)
        return path is not None

    def _navigate_to_point(self, goal):
        '''Navigate to point. NEVER pure rotation > 2s; always add lateral.'''
        twist = Twist()
        dx = goal[0] - self.robot_x
        dy = goal[1] - self.robot_y
        dist = math.hypot(dx, dy)
        goal_angle = math.atan2(dy, dx)
        heading_error = angle_diff(goal_angle, self.robot_yaw)

        n = len(self.laser_ranges)
        front_half_width = max(
            0.10,
            min(math.pi / 6.0, (n // 16) * max(abs(self.scan_angle_increment), 1e-6)),
        )
        front_values = self._clean_scan_sector(0.0, front_half_width)
        front_min = float(np.min(front_values)) if len(front_values) else np.inf

        stop_d = self._p('obstacle_stop_dist')
        slow_d = self._p('obstacle_slow_dist')
        speed = self._p('linear_speed')
        ang_speed = self._p('angular_speed')
        lat_speed = self._p('lateral_speed')

        # Front blocked → lateral dodge (NOT spin)
        if front_min < stop_d * 2.0:
            left_range = self._clean_scan_sector(math.pi / 2.0, math.pi / 4.0)
            right_range = self._clean_scan_sector(-math.pi / 2.0, math.pi / 4.0)
            left_clear = float(np.median(left_range)) if len(left_range) else 0.0
            right_clear = float(np.median(right_range)) if len(right_range) else 0.0

            # Lateral strafe + slight turn (never pure spin)
            if left_clear > right_clear:
                twist.linear.y = lat_speed
                twist.angular.z = self._p('angular_speed_slow') * 0.5
            else:
                twist.linear.y = -lat_speed
                twist.angular.z = -self._p('angular_speed_slow') * 0.5
            twist.linear.x = -0.02
            self._spinning = False
            return twist

        # Heading error handling — limit pure rotation time
        if abs(heading_error) > 0.5:
            now = time.time()
            if not self._spinning:
                self._spinning = True
                self._spin_start = now

            spin_duration = now - self._spin_start

            if spin_duration > 2.0:
                # Spinning too long! Add forward + lateral to break out
                self.get_logger().debug('Anti-spin: adding forward+lateral')
                twist.linear.x = speed * 0.5
                twist.linear.y = lat_speed * (1.0 if heading_error > 0 else -1.0)
                twist.angular.z = ang_speed * 0.3 * (1.0 if heading_error > 0 else -1.0)
                if spin_duration > 4.0:
                    self._spinning = False
                    self._spin_start = now
            else:
                # Normal short rotation (max 2s)
                twist.angular.z = ang_speed * (1.0 if heading_error > 0 else -1.0)
                twist.linear.x = speed * 0.2  # always creep forward
            return twist

        self._spinning = False

        # Drive toward goal with proportional steering
        twist.linear.x = speed
        twist.angular.z = heading_error * 2.0
        twist.angular.z = np.clip(twist.angular.z, -ang_speed, ang_speed)

        if dist < 0.5:
            twist.linear.x *= max(0.4, dist / 0.5)

        if front_min < slow_d:
            ratio = max(0.2, (front_min - stop_d) / (slow_d - stop_d + 0.001))
            twist.linear.x *= ratio

        return twist

    # ─── Frontier Detection ───

    def _find_boundary_revisit_goal(self):
        """Pick a reachable free cell near under-observed map edges."""
        if not self._p('boundary_revisit_enabled'):
            return None
        if self.map_data is None or self.map_info is None:
            return None
        if len(self._boundary_revisit_goals) >= int(self._p('max_boundary_revisits')):
            return None

        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y
        margin_cells = max(3, int(float(self._p('boundary_revisit_margin')) / max(res, 1.0e-6)))
        margin_cells = min(margin_cells, max(3, min(h, w) // 3))

        sides = {
            'north': (max(0, h - margin_cells), h, 0, w),
            'south': (0, min(h, margin_cells), 0, w),
            'east': (0, h, max(0, w - margin_cells), w),
            'west': (0, h, 0, min(w, margin_cells)),
        }

        candidates = []
        for side, (r0, r1, c0, c1) in sides.items():
            region = g[r0:r1, c0:c1]
            if region.size == 0:
                continue
            occupied_ratio = float(np.sum(region > 50)) / float(region.size)
            unknown_ratio = float(np.sum(region == -1)) / float(region.size)
            if occupied_ratio >= 0.04 and unknown_ratio < 0.04:
                continue

            free_rows, free_cols = np.where(region == 0)
            if len(free_rows) == 0:
                continue
            step = max(1, len(free_rows) // 100)
            urgency = max(0.0, 0.06 - occupied_ratio) * 8.0 + min(0.6, unknown_ratio)
            for local_r, local_c in zip(free_rows[::step], free_cols[::step]):
                row = int(r0 + local_r)
                col = int(c0 + local_c)
                wx = ox + (col + 0.5) * res
                wy = oy + (row + 0.5) * res
                if math.hypot(wx - self.robot_x, wy - self.robot_y) < 0.45:
                    continue
                if any(
                    math.hypot(wx - old_x, wy - old_y) < 0.65
                    for old_x, old_y in self._boundary_revisit_goals + self._gap_failed_goals
                ):
                    continue
                if not self._is_reachable_goal(wx, wy):
                    continue

                if side == 'north':
                    edge_distance = h - 1 - row
                elif side == 'south':
                    edge_distance = row
                elif side == 'east':
                    edge_distance = w - 1 - col
                else:
                    edge_distance = col

                distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
                score = urgency + 1.0 / (1.0 + edge_distance) + 0.04 * distance
                candidates.append((score, side, wx, wy, occupied_ratio, unknown_ratio))

        if not candidates:
            return None
        candidates.sort(key=lambda item: -item[0])
        score, side, wx, wy, occupied_ratio, unknown_ratio = candidates[0]
        self.get_logger().info(
            f'[BoundaryRevisit] {side} edge target ({wx:.2f},{wy:.2f}) '
            f'score={score:.2f} occ={occupied_ratio:.3f} unk={unknown_ratio:.3f}')
        return (wx, wy)

    def _find_frontier_goal(self):
        if self.map_data is None:
            return None
        g = self.map_data
        h, w = g.shape
        res = self.map_info.resolution
        ox = self.map_info.origin.position.x
        oy = self.map_info.origin.position.y

        unk = (g == -1)
        unknown_neighbors = np.zeros_like(g, dtype=np.int16)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dy == 0 and dx == 0:
                    continue
                unknown_neighbors += (
                    np.roll(np.roll(unk, dy, axis=0), dx, axis=1)
                ).astype(np.int16)
        # Use a known free approach cell instead of an unknown goal cell.
        fm = (g == 0) & (unknown_neighbors > 0)
        fm[:2, :] = fm[-2:, :] = fm[:, :2] = fm[:, -2:] = False

        if np.sum(fm) < self._p('min_frontier_size'):
            return None

        fy, fx = np.where(fm)
        wx = ox + (fx + 0.5) * res
        wy = oy + (fy + 0.5) * res
        strengths = unknown_neighbors[fy, fx]
        cs = self._p('frontier_cluster_dist')

        buckets = {}
        for px, py, strength in zip(wx, wy, strengths):
            key = (int(px / cs), int(py / cs))
            buckets.setdefault(key, []).append((px, py, int(strength)))

        clusters = []
        for members in buckets.values():
            if len(members) < self._p('min_frontier_size'):
                continue
            cx = sum(p[0] for p in members) / len(members)
            cy = sum(p[1] for p in members) / len(members)
            total_strength = sum(p[2] for p in members)
            clusters.append((cx, cy, len(members), total_strength))

        if not clusters:
            return None

        rx, ry = self.robot_x, self.robot_y
        scored = []
        for cx, cy, cluster_size, total_strength in clusters:
            dd = math.hypot(cx - rx, cy - ry)
            if dd < self._p('min_frontier_distance'):
                continue
            if any(
                math.hypot(cx - fx, cy - fy) < self._p('frontier_blacklist_dist')
                for fx, fy in self._gap_failed_goals
            ):
                continue
            if not self._is_reachable_goal(cx, cy):
                continue
            info_score = total_strength + 0.5 * cluster_size - 0.25 * dd
            scored.append((info_score, dd, cx, cy, cluster_size, total_strength))

        if not scored:
            return None

        scored.sort(key=lambda item: (-item[0], item[1]))
        _, dist, bx, by, cluster_size, total_strength = scored[0]
        self.get_logger().info(
            f'[GapFill] Free approach: ({bx:.2f},{by:.2f}) '
            f'dist={dist:.2f}m size={cluster_size} strength={total_strength}')
        return (bx, by)

    def _grid_cluster(self, pts, cs):
        if len(pts) == 0:
            return []
        grid = {}
        for px, py in pts:
            k = (int(px / cs), int(py / cs))
            grid.setdefault(k, []).append((px, py))
        return [(sum(p[0] for p in v) / len(v), sum(p[1] for p in v) / len(v))
                for v in grid.values() if len(v) >= self._p('min_frontier_size')]

    # ─── Progress Reporting ───

    def _report_progress(self, now):
        if now - self._last_progress_report < self._p('progress_report_interval'):
            return
        self._last_progress_report = now
        if self.map_data is None:
            return
        g = self.map_data
        total = g.size
        free = int(np.sum(g == 0))
        occupied = int(np.sum(g > 50))
        unknown = int(np.sum(g == -1))
        coverage = (free + occupied) / total * 100.0
        elapsed = now - self._start_time if self._start_time else 0
        mins = int(elapsed // 60)
        secs = int(elapsed % 60)

        phase_info = self.phase
        if self.phase == PHASE_BOUSTROPHEDON:
            phase_info += f' ({self._coverage_idx}/{len(self._coverage_path)})'
        elif self.phase == PHASE_CROSS_EXPLORE and self._cross_goal:
            d = math.hypot(self._cross_goal[0] - self.robot_x,
                           self._cross_goal[1] - self.robot_y)
            phase_info += f' (d={d:.1f}m, r={self._cross_round})'
        elif self.phase == PHASE_GAP_FILL:
            phase_info += f' (failed={len(self._gap_failed_goals)})'

        self.get_logger().info(
            f'\033[1;33m[{phase_info}] {mins:02d}:{secs:02d} | '
            f'coverage={coverage:.1f}% | free={free} occ={occupied} unk={unknown} | '
            f'pos=({self.robot_x:.2f},{self.robot_y:.2f})\033[0m')
        self._try_checkpoint_save(now)

    # ─── Map Saving ───

    def _try_checkpoint_save(self, now):
        interval = float(self._p('checkpoint_save_interval_sec'))
        if interval <= 0.0 or not self._p('auto_save_map'):
            return False
        if self.map_data is None:
            return False
        if now - self._last_checkpoint_save < interval:
            return False
        self._last_checkpoint_save = now
        return self._save_map_to_disk('checkpoint', mark_saved=False)

    def _try_save_map(self, reason='unknown'):
        if self._map_saved:
            return True
        return self._save_map_to_disk(reason, mark_saved=True)

    def _save_map_to_disk(self, reason='unknown', mark_saved=True):
        if not self._p('auto_save_map'):
            return False
        save_path = Path(os.path.expanduser(str(self._p('map_save_path')))).expanduser()
        save_dir = save_path.parent
        save_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(
            f'\033[1;35m[MapSave] Saving map ({reason}) -> {save_path}\033[0m')
        try:
            env = os.environ.copy()
            env['ROS_DOMAIN_ID'] = env.get('ROS_DOMAIN_ID', '0')
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
                 '-f', str(save_path),
                 '--ros-args', '-p', 'map_subscribe_transient_local:=true'],
                timeout=30, capture_output=True, text=True, env=env)
            if result.returncode == 0:
                if mark_saved:
                    self._map_saved = True
                self.get_logger().info(
                    f'\033[1;32m[MapSave] SUCCESS: {save_path}.pgm + .yaml\033[0m')
                return True
            else:
                self.get_logger().error(f'[MapSave] FAILED: {result.stderr}')
                return False
        except subprocess.TimeoutExpired:
            self.get_logger().error('[MapSave] Timeout (30s)')
            return False
        except Exception as e:
            self.get_logger().error(f'[MapSave] Error: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = AutoExploreNode('auto_explore')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Ctrl+C received, saving map before exit...')
    finally:
        if rclpy.ok():
            try:
                node.cmd_pub.publish(Twist())
                node._try_save_map('shutdown')
            except Exception as exc:
                node.get_logger().warn(f'Shutdown save skipped: {exc}')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
