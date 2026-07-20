from setuptools import find_packages, setup

package_name = 'turtlesim_catch_them_all'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Stage 2 final Turtlesim catch-them-all project.',
    license='Apache-2.0',
    tests_require=['pytest'],
    # console_scripts：把 Python 函数注册为可执行命令，
    # 之后即可用 ros2 run turtlesim_catch_them_all <命令名> 启动
    # 格式："命令名 = 包名.模块名:入口函数"
    entry_points={
        'console_scripts': [
            "controller = turtlesim_catch_them_all.turtle_controller:main",
            "spawner = turtlesim_catch_them_all.turtle_spawner:main",
            "move_to_position_server = turtlesim_catch_them_all.move_to_position_server:main",
        ],
    },
)
