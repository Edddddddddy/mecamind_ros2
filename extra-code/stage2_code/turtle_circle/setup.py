from setuptools import find_packages, setup

package_name = "turtle_circle"

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
    description="Publish Twist commands to make turtle1 move in a circle.",
    license="Apache-2.0",
    tests_require=["pytest"],
    # console_scripts：注册可执行命令，
    # 使 ros2 run turtle_circle circle_node 能找到入口函数
    entry_points={
        "console_scripts": [
            "circle_node = turtle_circle.circle_node:main",
        ],
    },
)

