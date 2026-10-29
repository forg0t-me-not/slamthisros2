import os
from glob import glob

from setuptools import setup


package_name = 'slamthisros2'

simulation_dir = os.path.join(
    'turtlebot3_simulations',
    'turtlebot3_gazebo',
)

stadium_model_dir = os.path.join(
    simulation_dir,
    'models',
    'Stadium',
)


setup(
    name=package_name,
    version='0.1.0',

    packages=[package_name],

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),

        (
            'share/' + package_name,
            ['package.xml'],
        ),

        (
            'share/' + package_name + '/launch',
            [
                os.path.join(
                    simulation_dir,
                    'launch',
                    'course_final.launch.py',
                )
            ],
        ),

        (
            'share/' + package_name + '/worlds',
            [
                os.path.join(
                    simulation_dir,
                    'worlds',
                    'Stadium.world',
                )
            ],
        ),

        # Install only model.config at this level.
        (
            'share/' + package_name + '/models/Stadium',
            [
                os.path.join(
                    stadium_model_dir,
                    'model.config',
                )
            ],
        ),

        # Install the contents of meshes separately.
        (
            'share/' + package_name + '/models/Stadium/meshes',
            glob(
                os.path.join(
                    stadium_model_dir,
                    'meshes',
                    '*',
                )
            ),
        ),

        (
            'share/' + package_name + '/maps',
            [
                'final_map.pgm',
                'final_map.yaml',
            ],
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=False,

    maintainer='waiyanlynnmyint',
    maintainer_email='waiyanlynnmyint@outlook.com',
    description='TurtleBot3 stadium Tasks A, B, and C',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'main = slamthisros2.main:main',
            'task_a = slamthisros2.Task_A:main',
            'task_b = slamthisros2.Task_B:main',
            'task_c = slamthisros2.Task_C:main',
        ],
    },
)
