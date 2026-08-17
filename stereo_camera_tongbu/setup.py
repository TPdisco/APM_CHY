from setuptools import find_packages, setup

package_name = 'stereo_camera_tongbu'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'message_filters',
        'tf2_ros',
        'geometry_msgs',
        'numpy',
        'transforms3d',  # 替代 tf_transformations
        ],
    zip_safe=True,
    maintainer='disco',
    maintainer_email='disco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'stereo_tongbu_node = stereo_camera_tongbu.stereo_tongbu_node:main'
        ],
    },
)
