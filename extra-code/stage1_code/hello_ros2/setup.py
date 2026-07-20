# 文件说明：hello_ros2 包的 Python 打包配置（colcon build 时使用）。
# ROS 2 的 Python 包基于 setuptools，本文件声明包名、
# 安装的资源文件以及可执行入口（entry_points）。
from setuptools import find_packages, setup

package_name = 'hello_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    # data_files：把非代码文件装进 install 目录
    # - resource/ 下的标记文件让 ament 索引找到这个包
    # - package.xml 是 ROS 包清单，必须一起安装
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ROS2 AI Course',
    maintainer_email='course@example.com',
    description='Stage 1 reference package with a tested ROS 2 talker node.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # entry_points：定义 ros2 run 可执行程序
    # 'talker = hello_ros2.talker_node:main' 表示
    # `ros2 run hello_ros2 talker` 会调用
    # hello_ros2/talker_node.py 里的 main() 函数
    entry_points={
        'console_scripts': [
            'talker = hello_ros2.talker_node:main',
        ],
    },
)
