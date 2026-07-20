from setuptools import find_packages, setup

package_name = 'py_first_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # data_files：把非代码文件安装到 install 目录，
    # 让 ros2 命令能找到这个包（包注册信息和 package.xml）
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='My first Python ROS 2 package',
    license='Apache-2.0',
    tests_require=['pytest'],
    # console_scripts：注册可执行程序，格式为
    # "可执行名 = 包名.模块名:函数名"
    # 之后就能用 ros2 run py_first_pkg py_minimal 运行
    entry_points={
        'console_scripts': [
            'py_minimal = py_first_pkg.minimal_node:main',
            'py_oop = py_first_pkg.oop_node:main',
        ],
    },
)
