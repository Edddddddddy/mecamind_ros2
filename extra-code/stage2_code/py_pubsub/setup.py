from setuptools import find_packages, setup

package_name = "py_pubsub"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    # data_files：安装包注册文件和 package.xml，
    # 使 ros2 run / ros2 pkg 等命令能识别本包
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Python topic publisher/subscriber examples.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # console_scripts：可执行程序注册表，格式为
    # "可执行名 = 包名.模块名:函数名"，
    # 对应 ros2 run py_pubsub publisher_node 等命令
    entry_points={
        "console_scripts": [
            "publisher_node = py_pubsub.publisher_node:main",
            "subscriber_node = py_pubsub.subscriber_node:main",
        ],
    },
)

