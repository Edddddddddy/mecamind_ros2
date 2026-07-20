# 单元测试：验证 spawner 的 catch_turtle 服务处理策略——
# kill 无法安排时返回 success=False 且海龟保留在存活列表；
# kill 安排成功时返回 success=True、移除海龟并重新发布列表。
from types import SimpleNamespace
from unittest.mock import Mock

from turtlesim_catch_them_all.turtle_spawner import TurtleSpawnerNode


def make_spawner_double(kill_scheduled):
    return SimpleNamespace(
        alive_turtles_=[SimpleNamespace(name='turtle2')],
        call_kill_service=Mock(return_value=kill_scheduled),
        publish_alive_turtles=Mock(),
    )


def test_target_remains_alive_when_kill_cannot_be_scheduled():
    spawner = make_spawner_double(kill_scheduled=False)
    response = SimpleNamespace(success=None)

    TurtleSpawnerNode.callback_catch_turtle(
        spawner, SimpleNamespace(name='turtle2'), response
    )

    assert response.success is False
    assert [turtle.name for turtle in spawner.alive_turtles_] == ['turtle2']
    spawner.publish_alive_turtles.assert_not_called()


def test_target_is_removed_after_kill_is_scheduled():
    spawner = make_spawner_double(kill_scheduled=True)
    response = SimpleNamespace(success=None)

    TurtleSpawnerNode.callback_catch_turtle(
        spawner, SimpleNamespace(name='turtle2'), response
    )

    assert response.success is True
    assert spawner.alive_turtles_ == []
    spawner.publish_alive_turtles.assert_called_once_with()
