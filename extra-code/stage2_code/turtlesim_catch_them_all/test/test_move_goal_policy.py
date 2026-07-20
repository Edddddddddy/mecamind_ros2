# 单元测试：验证 move_to_position_server 的 Goal 接受策略——
# 合法坐标的 Goal 被 ACCEPT 并占用机器人（goal_reserved_）；
# 已有 Goal 执行中或坐标越界时返回 REJECT 且不占用机器人。
from threading import Lock
from types import SimpleNamespace
from unittest.mock import Mock

from rclpy.action import GoalResponse

from turtlesim_catch_them_all.move_to_position_server import MoveToPositionServer


def make_server_double(reserved=False):
    return SimpleNamespace(
        get_logger=Mock(return_value=Mock()),
        goal_lock_=Lock(),
        goal_reserved_=reserved,
    )


def test_valid_goal_reserves_single_robot():
    server = make_server_double()
    goal = SimpleNamespace(target_x=2.0, target_y=8.0)

    response = MoveToPositionServer.goal_callback(server, goal)

    assert response == GoalResponse.ACCEPT
    assert server.goal_reserved_ is True


def test_second_goal_is_rejected_while_move_is_active():
    server = make_server_double(reserved=True)
    goal = SimpleNamespace(target_x=2.0, target_y=8.0)

    response = MoveToPositionServer.goal_callback(server, goal)

    assert response == GoalResponse.REJECT


def test_out_of_bounds_goal_does_not_reserve_robot():
    server = make_server_double()
    goal = SimpleNamespace(target_x=-1.0, target_y=8.0)

    response = MoveToPositionServer.goal_callback(server, goal)

    assert response == GoalResponse.REJECT
    assert server.goal_reserved_ is False
