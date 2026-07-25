"""Gazebo 红色跟随目标：跨房间 L 形折线驱动。

【轨迹】客厅 → 向下进入门厅/走廊 → 再横向穿到另一侧（L 形），
覆盖不同房间，而不是在墙边小椭圆里蹭。

默认折点（避开桌柜柱与隔断）：
  (-2.0,  1.0) 客厅
  (-2.0, -2.15) 南下到门厅
  ( 2.2, -2.15) 沿南侧通道横穿（隔断墙底边以南、柱子以北）

启动时会把小车摆到起点后方并朝南，便于一开跟随就能看见目标。
"""

from __future__ import annotations

import ast
import math
import shutil
import subprocess
from typing import List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

Point = Tuple[float, float]


def _parse_waypoints(raw: object) -> List[Point]:
    """解析 waypoints 参数：支持 JSON/Python 列表或扁平 [x0,y0,x1,y1,...]。"""
    if isinstance(raw, str):
        data = ast.literal_eval(raw)
    else:
        data = raw
    if not isinstance(data, (list, tuple)) or len(data) < 4:
        raise ValueError(f"waypoints 至少需要 2 个点，收到: {raw!r}")
    # [[x,y], ...] 
    if isinstance(data[0], (list, tuple)):
        pts = [(float(p[0]), float(p[1])) for p in data]
    else:
        if len(data) % 2 != 0:
            raise ValueError(f"扁平 waypoints 长度必须为偶数: {raw!r}")
        pts = [(float(data[i]), float(data[i + 1])) for i in range(0, len(data), 2)]
    if len(pts) < 2:
        raise ValueError("waypoints 至少 2 个点")
    return pts


def _polyline_length(pts: Sequence[Point]) -> float:
    total = 0.0
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def _point_on_polyline(pts: Sequence[Point], dist: float) -> Point:
    """沿折线行走 dist 米（从起点算）得到插值坐标；超出端点则钳制。"""
    if dist <= 0.0:
        return pts[0]
    traveled = 0.0
    for i in range(1, len(pts)):
        x0, y0 = pts[i - 1]
        x1, y1 = pts[i]
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-6:
            continue
        if traveled + seg >= dist:
            t = (dist - traveled) / seg
            return (x0 + t * (x1 - x0), y0 + t * (y1 - y0))
        traveled += seg
    return pts[-1]


class FollowTargetMover(Node):
    def __init__(self) -> None:
        super().__init__("mecamind_follow_target_mover")
        self.declare_parameter("world_name", "three_room_house")
        self.declare_parameter("model_name", "follow_target")
        # 客厅 → 南下 → 横穿南侧通道（L 形，跨房间）
        self.declare_parameter(
            "waypoints_xy",
            "[-2.0, 1.0, -2.0, -2.15, 2.2, -2.15]",
        )
        self.declare_parameter("z", 0.38)
        self.declare_parameter("speed_mps", 0.16)
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("enabled", True)
        self.declare_parameter("ping_pong", True)
        # 等仿真传感器起来后再摆车，避免冷启动 set_pose 把服务打爆
        self.declare_parameter("startup_delay_sec", 8.0)
        # 起点后方朝南，对准 L 的第一段（向下）
        self.declare_parameter("reposition_robot", True)
        self.declare_parameter("robot_name", "mecamind_mecanum")
        self.declare_parameter("robot_x", -2.0)
        self.declare_parameter("robot_y", 1.55)
        self.declare_parameter("robot_z", 0.10)
        self.declare_parameter("robot_yaw", -math.pi / 2.0)

        self._gz_bin = shutil.which("gz")
        if not self._gz_bin:
            self.get_logger().error("未找到 gz 可执行文件，无法驱动跟随目标")

        try:
            self._waypoints = _parse_waypoints(self.get_parameter("waypoints_xy").value)
        except ValueError as exc:
            self.get_logger().error(f"waypoints 无效，回退默认 L: {exc}")
            self._waypoints = [(-2.0, 1.0), (-2.0, -2.15), (2.2, -2.15)]
        self._path_len = max(0.1, _polyline_length(self._waypoints))
        self._s = 0.0
        self._direction = 1.0
        self._busy = False
        self._robot_ready = False
        self._boot_time = self.get_clock().now()
        self._last_tick = self._boot_time

        rate = max(1.0, float(self.get_parameter("rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._on_timer)
        pts = " -> ".join(f"({x:.1f},{y:.1f})" for x, y in self._waypoints)
        self.get_logger().info(
            f"follow_target_mover L 轨迹: {pts} | "
            f"len={self._path_len:.2f}m speed={float(self.get_parameter('speed_mps').value):.2f}m/s"
        )

    def _on_timer(self) -> None:
        if (
            not bool(self.get_parameter("enabled").value)
            or not self._gz_bin
            or self._busy
        ):
            return
        boot_wait = max(0.0, float(self.get_parameter("startup_delay_sec").value))
        if (self.get_clock().now() - self._boot_time).nanoseconds * 1e-9 < boot_wait:
            return
        if bool(self.get_parameter("reposition_robot").value) and not self._robot_ready:
            ok = self._set_pose(
                str(self.get_parameter("robot_name").value),
                float(self.get_parameter("robot_x").value),
                float(self.get_parameter("robot_y").value),
                float(self.get_parameter("robot_z").value),
                yaw=float(self.get_parameter("robot_yaw").value),
            )
            if ok:
                # 目标放到 L 起点
                x0, y0 = self._waypoints[0]
                self._set_pose(
                    str(self.get_parameter("model_name").value),
                    x0,
                    y0,
                    float(self.get_parameter("z").value),
                )
                self._robot_ready = True
                self._s = 0.0
                self._direction = 1.0
                self._last_tick = self.get_clock().now()
                self.get_logger().info("已将小车摆到客厅起点后方（朝南），开始 L 形跨房间轨迹")
            return

        now = self.get_clock().now()
        dt = (now - self._last_tick).nanoseconds * 1e-9
        self._last_tick = now
        # sim 刚启动时 dt 可能异常大，钳制避免瞬移
        dt = max(0.0, min(dt, 0.5))
        speed = max(0.02, float(self.get_parameter("speed_mps").value))
        self._s += self._direction * speed * dt

        if bool(self.get_parameter("ping_pong").value):
            if self._s >= self._path_len:
                self._s = self._path_len
                self._direction = -1.0
            elif self._s <= 0.0:
                self._s = 0.0
                self._direction = 1.0
        else:
            if self._s >= self._path_len:
                self._s = 0.0  # 循环

        x, y = _point_on_polyline(self._waypoints, self._s)
        self._set_pose(
            str(self.get_parameter("model_name").value),
            x,
            y,
            float(self.get_parameter("z").value),
        )

    def _set_pose(
        self,
        name: str,
        x: float,
        y: float,
        z: float,
        yaw: Optional[float] = None,
    ) -> bool:
        world = str(self.get_parameter("world_name").value)
        req = f'name: "{name}", position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}}'
        if yaw is not None:
            half = 0.5 * yaw
            req += (
                f", orientation: {{x: 0, y: 0, "
                f"z: {math.sin(half):.6f}, w: {math.cos(half):.6f}}}"
            )
        cmd = [
            self._gz_bin,
            "service",
            "-s",
            f"/world/{world}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "2000",
            "--req",
            req,
        ]
        self._busy = True
        try:
            result = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=3.5,
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            self.get_logger().warning(f"set_pose 调用失败: {exc}", throttle_duration_sec=5.0)
            return False
        finally:
            self._busy = False
        ok = result.returncode == 0 and "data: true" in (result.stdout or "")
        if not ok:
            detail = (result.stderr or result.stdout or "").strip()
            self.get_logger().warning(
                f"set_pose 未成功 (code={result.returncode}): {detail[:200]}",
                throttle_duration_sec=5.0,
            )
        return ok


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = FollowTargetMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
