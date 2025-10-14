import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'air_router'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fclad',
    maintainer_email='fclad@seas.upenn.edu',
    description='air_router routes your quad to satisfy all your communication needs',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = air_router.test_node:main',
            'navigator = air_router.navigator:main',
            'goal_finder = air_router.goal_finder:main',
            'robot_finder = air_router.robot_finder:main',
            'test_goal_finder = air_router.test_goal_finder:main',
            'test_navigator = air_router.test_navigator:main',
            'test_robot_finder = air_router.test_robot_finder:main'
        ],
    },
)
