# 文件说明：talker_node 的单元测试。
# 只测试纯函数 build_greeting_message 的消息内容，
# 不需要启动 ROS 2。运行：colcon test 或 pytest。
from hello_ros2.talker_node import build_greeting_message


def test_build_greeting_message():
    message = build_greeting_message()

    assert message.data == 'Hello ROS 2!'
