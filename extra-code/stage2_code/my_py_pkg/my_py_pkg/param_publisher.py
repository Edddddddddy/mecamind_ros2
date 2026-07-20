# ============================================================
# 参数（Parameter）系统示例：可配置的 Publisher
# 演示概念：declare_parameter 声明参数、运行中动态改参数、
# 参数校验回调（on_set）与生效回调（post_set）
# 运行方式：ros2 run my_py_pkg param_publisher
# 运行中修改参数示例：
#   ros2 param set /param_publisher frequency 2.0
# ============================================================
import rclpy
# SetParametersResult：参数校验回调必须返回的结果类型
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


class ParamPublisher(Node):
    """Publish a configurable String message and react to parameter updates."""

    def __init__(self):
        super().__init__("param_publisher")
        # 声明参数并给默认值；只有声明过的参数才能被
        # ros2 param 命令读写，也可在启动时用 -p 覆盖
        self.declare_parameter("frequency", 1.0)
        self.declare_parameter("message", "Hello ROS 2")

        self.publisher_ = self.create_publisher(String, "chatter", 10)
        # 读取参数当前值，缓存为成员变量供回调使用
        self.frequency_ = float(self.get_parameter("frequency").value)
        self.message_ = str(self.get_parameter("message").value)
        if self.frequency_ <= 0.0:
            raise ValueError("frequency must be positive")
        # 周期 = 1/频率，例如 frequency=2.0 时每 0.5 秒发一次
        self.timer_ = self.create_timer(1.0 / self.frequency_, self.timer_callback)
        # 注册两个参数回调：
        # on_set 在参数生效前触发，用于校验（可拒绝非法值）；
        # post_set 在参数生效后触发，用于应用新值
        self.on_set_callback_handle_ = self.add_on_set_parameters_callback(
            self.on_set_parameters)
        self.post_set_callback_handle_ = self.add_post_set_parameters_callback(
            self.post_set_parameters)
        self.get_logger().info(
            f"Param publisher started: frequency={self.frequency_}, "
            f'message="{self.message_}"')

    def timer_callback(self):
        # 定时发布参数指定的消息内容
        msg = String()
        msg.data = self.message_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

    def on_set_parameters(self, params: list[Parameter]):
        # 校验回调：参数生效前调用。返回 successful=False
        # 就会拒绝这次修改（ros2 param set 会报失败）
        for param in params:
            if param.name == "frequency":
                if float(param.value) <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason="frequency must be positive",
                    )
        return SetParametersResult(successful=True)

    def post_set_parameters(self, params: list[Parameter]):
        # 生效回调：参数已通过校验并写入后调用，
        # 在这里把新值应用到节点的实际行为上
        frequency_changed = False
        for param in params:
            if param.name == "frequency":
                self.frequency_ = float(param.value)
                frequency_changed = True
            elif param.name == "message":
                self.message_ = str(param.value)

        # Timer 周期创建后不能直接改，
        # 频率变了就取消旧 Timer、按新周期重建一个
        if frequency_changed:
            self.timer_.cancel()
            self.timer_ = self.create_timer(
                1.0 / self.frequency_, self.timer_callback)
        self.get_logger().info(
            f"Parameters updated: frequency={self.frequency_}, "
            f'message="{self.message_}"')


def main(args=None):
    rclpy.init(args=args)
    node = ParamPublisher()
    # spin 阻塞运行；参数修改请求也是在事件循环中被处理的
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
