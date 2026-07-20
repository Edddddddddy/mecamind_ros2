from setuptools import find_packages, setup

package_name = "py_action_tutorial"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="student",
    maintainer_email="student@example.com",
    description="Python action tutorial nodes.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # console_scripts：把 Python 函数注册为可执行命令，
    # 之后即可用 ros2 run py_action_tutorial <命令名> 启动节点
    # 格式："命令名 = 包名.模块名:入口函数"
    entry_points={
        "console_scripts": [
            "count_until_server = py_action_tutorial.count_until_server:main",
            "count_until_client = py_action_tutorial.count_until_client:main",
            "move_to_position_server = py_action_tutorial.move_to_position_server:main",
            "move_to_position_client = py_action_tutorial.move_to_position_client:main",
        ],
    },
)

