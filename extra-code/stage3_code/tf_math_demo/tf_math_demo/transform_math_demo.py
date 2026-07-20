# 文件说明：用纯 numpy 演示 TF 背后的数学——齐次变换矩阵。
# 演示概念：RPY 旋转矩阵、4x4 齐次变换、逆变换、变换链乘。
# 不依赖 ROS 2 运行时，可直接运行：
#   ros2 run tf_math_demo transform_math_demo
#   或 python3 transform_math_demo.py
import math

import numpy as np


# 按 ROS 约定合成旋转矩阵：先绕 x 转 roll，再绕 y 转
# pitch，最后绕 z 转 yaw，即 R = Rz @ Ry @ Rx
def rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Return ROS-style RPY rotation matrix: Rz(yaw) * Ry(pitch) * Rx(roll)."""
    rx = np.array(
        [
            [1.0, 0.0, 0.0],
            [0.0, math.cos(roll), -math.sin(roll)],
            [0.0, math.sin(roll), math.cos(roll)],
        ]
    )
    ry = np.array(
        [
            [math.cos(pitch), 0.0, math.sin(pitch)],
            [0.0, 1.0, 0.0],
            [-math.sin(pitch), 0.0, math.cos(pitch)],
        ]
    )
    rz = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    return rz @ ry @ rx


# 组装 4x4 齐次变换矩阵：左上 3x3 放旋转 R，右上 3x1 放
# 平移 t。用它左乘齐次点 [x,y,z,1] 即可一次完成旋转+平移，
# 这正是 TF 中一个 frame 到另一个 frame 的变换表示
def make_transform(translation, rpy) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix_rpy(*rpy)
    transform[:3, 3] = np.array(translation, dtype=float)
    return transform


# 齐次变换的解析逆：旋转矩阵是正交阵，逆等于转置 R^T，
# 平移部分变为 -R^T @ t。比直接调用通用矩阵求逆更快更稳定
def invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ translation
    return inverse


def print_matrix(name: str, matrix: np.ndarray) -> None:
    print(f"{name} =")
    print(np.array2string(matrix, precision=3, suppress_small=True))


def main() -> None:
    np.set_printoptions(precision=3, suppress=True)

    # 示例 1：TurtleBot3 上雷达(base_scan)相对底盘(base_link)
    # 的真实安装位置——后移 0.032 m、抬高 0.172 m、无旋转
    print("=== 1. TurtleBot3 base_link -> base_scan transform ===")
    t_base_scan = make_transform([-0.032, 0.0, 0.172], [0.0, 0.0, 0.0])
    print_matrix("T_base_scan", t_base_scan)

    # 把雷达坐标系下的点变换到底盘坐标系：
    # 齐次坐标末位补 1，矩阵左乘即完成变换
    point_scan = np.array([2.0, 0.5, 0.0, 1.0])
    point_base = t_base_scan @ point_scan
    print(f"point_scan: {point_scan[:3]}")
    print(f"point_base: {point_base[:3]}")
    assert np.allclose(point_base[:3], [1.968, 0.5, 0.172])

    # 示例 2：逆变换——反方向 base_scan -> base_link，
    # 与原变换相乘应得到单位矩阵（互为逆）
    print("\n=== 2. Transform inverse ===")
    t_scan_base = invert_transform(t_base_scan)
    print_matrix("T_scan_base", t_scan_base)
    assert np.allclose(t_base_scan @ t_scan_base, np.eye(4))
    print("T_base_scan * T_scan_base == identity: True")

    # 示例 3：变换链——把多段变换依次矩阵相乘，就得到
    # map 直接到 base_scan 的变换。TF 树在后台做的正是
    # 沿树找路径然后连乘这些矩阵
    print("\n=== 3. Transform chain: map -> odom -> base_link -> base_scan ===")
    t_map_odom = make_transform([1.0, 2.0, 0.0], [0.0, 0.0, 0.0])
    t_odom_base = make_transform([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    t_map_scan = t_map_odom @ t_odom_base @ t_base_scan
    print_matrix("T_map_scan", t_map_scan)
    print(f"base_scan origin in map: {t_map_scan[:3, 3]}")
    assert np.allclose(t_map_scan[:3, 3], [0.968, 2.0, 0.172])

    obstacle_scan = np.array([3.0, 1.0, 0.0, 1.0])
    obstacle_map = t_map_scan @ obstacle_scan
    print(f"obstacle_scan: {obstacle_scan[:3]}")
    print(f"obstacle_map: {obstacle_map[:3]}")
    assert np.allclose(obstacle_map[:3], [3.968, 3.0, 0.172])

    # 示例 4：课件练习题验证——带 yaw 90° 旋转的变换，
    # x 轴方向的点会被转到 y 轴方向
    print("\n=== 4. Exercise check: yaw 90 degrees ===")
    t_base_laser = make_transform([0.1, 0.0, 0.2], [0.0, 0.0, math.pi / 2.0])
    exercise_point = np.array([1.0, 0.0, 0.0, 1.0])
    exercise_result = t_base_laser @ exercise_point
    print_matrix("T_base_laser", t_base_laser)
    print(f"exercise result: {exercise_result[:3]}")
    assert np.allclose(exercise_result[:3], [0.1, 1.0, 0.2])

    print("\nAll transform math checks passed.")


if __name__ == "__main__":
    main()
