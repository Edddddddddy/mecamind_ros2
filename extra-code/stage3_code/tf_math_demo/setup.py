# 文件说明：tf_math_demo 包的 Python 打包配置。
# 声明安装的资源文件（含 RViz 配置）和两个可执行入口。
from setuptools import find_packages, setup

package_name = "tf_math_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    # data_files：ament 索引标记、包清单，以及随包安装的
    # RViz 配置文件（课上可视化 TurtleBot3 TF 树时加载）
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/rviz", ["rviz/turtlebot3_tf.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ROS2 AI Course",
    maintainer_email="course@example.com",
    description="Runnable TF and transform math demos for Stage 3 lesson 3.1.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # entry_points：注册两个 ros2 run 可执行程序，
    # 名字 = 模块路径:函数，运行时调用对应的 main()
    entry_points={
        "console_scripts": [
            "transform_math_demo = tf_math_demo.transform_math_demo:main",
            "tf_point_transform_demo = tf_math_demo.tf_point_transform_demo:main",
        ],
    },
)
