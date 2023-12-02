from setuptools import setup

package_name = 'ros2bag_convert'

setup(
    name=package_name,
    version='0.1.0',
    packages=["ros2bag_convert.ros2bag_convert","ros2bag_convert"],
    # data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']), ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fishros',
    maintainer_email='sangxin2014@sina.com',
    description='Convert ROS2 bag files to CSV, JSON, etc. ',
    license='MIT License (MIT)',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'ros2bag-tool' = "wq"
            "ros2bag-convert = ros2bag_convert.main:main",
        ],
    },
)
