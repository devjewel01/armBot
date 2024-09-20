from setuptools import setup

package_name = 'armbot_py_examples'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jewelnath.me@gmail.com',
    description='ROS 2 Code Examples',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = armbot_py_examples.simple_publisher:main',
            'simple_subscriber = armbot_py_examples.simple_subscriber:main',
            'simple_parameter = armbot_py_examples.simple_parameter:main',
            'simple_moveit = armbot_py_examples.simple_moveit_interface:main',
            'simple_service_server = armbot_py_examples.simple_service_server:main',
            'simple_service_client = armbot_py_examples.simple_service_client:main',
            'simple_action_server = armbot_py_examples.simple_action_server:main',
            'simple_action_client = armbot_py_examples.simple_action_client:main',
        ],
    },
)
