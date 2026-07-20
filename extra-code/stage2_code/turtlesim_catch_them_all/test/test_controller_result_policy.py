# 单元测试：验证 controller 处理 Action Result 的策略——
# 移动未成功（CANCELED/ABORTED）时绝不调用抓捕服务并复位状态；
# 移动成功（SUCCEEDED）时对选中的目标海龟发起抓捕。
# 用 SimpleNamespace + Mock 构造假节点，不依赖真实 ROS 环境。
from types import SimpleNamespace
from unittest.mock import Mock

from action_msgs.msg import GoalStatus
import pytest

from turtlesim_catch_them_all.turtle_controller import TurtleControllerNode


class CompletedFuture:
    def __init__(self, value):
        self.value = value

    def result(self):
        return self.value


def make_controller_double():
    return SimpleNamespace(
        call_catch_turtle_service=Mock(),
        get_logger=Mock(return_value=Mock()),
        goal_in_progress_=True,
        turtle_to_catch_=SimpleNamespace(name='turtle2'),
    )


@pytest.mark.parametrize(
    'status',
    [GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED],
)
def test_unsuccessful_move_never_catches_target(status):
    controller = make_controller_double()
    future = CompletedFuture(SimpleNamespace(status=status, result=None))

    TurtleControllerNode.result_callback(controller, future)

    controller.call_catch_turtle_service.assert_not_called()
    assert controller.turtle_to_catch_ is None
    assert controller.goal_in_progress_ is False


def test_successful_move_catches_selected_target():
    controller = make_controller_double()
    result = SimpleNamespace(final_x=2.0, final_y=3.0, distance_error=0.01)
    future = CompletedFuture(
        SimpleNamespace(status=GoalStatus.STATUS_SUCCEEDED, result=result)
    )

    TurtleControllerNode.result_callback(controller, future)

    controller.call_catch_turtle_service.assert_called_once_with('turtle2')
