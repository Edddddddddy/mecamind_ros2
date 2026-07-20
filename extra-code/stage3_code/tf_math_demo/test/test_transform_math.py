# 文件说明：transform_math_demo 中变换数学函数的单元测试。
# 覆盖三点：yaw 旋转把 x 轴转到 y 轴、齐次变换同时应用
# 旋转和平移、逆变换与原变换相乘为单位矩阵。
# 运行：colcon test 或在包目录下执行 pytest。
import math

import numpy as np

from tf_math_demo.transform_math_demo import invert_transform
from tf_math_demo.transform_math_demo import make_transform
from tf_math_demo.transform_math_demo import rotation_matrix_rpy


def test_yaw_rotation_maps_x_axis_to_y_axis():
    rotation = rotation_matrix_rpy(0.0, 0.0, math.pi / 2.0)

    assert np.allclose(rotation @ np.array([1.0, 0.0, 0.0]), [0.0, 1.0, 0.0])


def test_make_transform_applies_rotation_and_translation():
    transform = make_transform([0.1, 0.0, 0.2], [0.0, 0.0, math.pi / 2.0])
    point = np.array([1.0, 0.0, 0.0, 1.0])

    assert np.allclose(transform @ point, [0.1, 1.0, 0.2, 1.0])


def test_inverse_transform_round_trip_is_identity():
    transform = make_transform([1.2, -0.4, 0.3], [0.2, -0.1, 0.7])
    inverse = invert_transform(transform)

    assert np.allclose(transform @ inverse, np.eye(4))
    assert np.allclose(inverse @ transform, np.eye(4))
