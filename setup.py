from setuptools import find_packages, setup
import os
import glob

package_name = 'my_behavior_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pw',
    maintainer_email='pw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calc_pos_error = my_behavior_tree.calc_pos_error:main',
            'bt_basic = my_behavior_tree.bt_basic:main',
            'bt_basic_blackboard = my_behavior_tree.bt_basic_blackboard:main',
            'bt_turltesim_nav = my_behavior_tree.bt_turltesim_nav:tutorial_main',
            'bt_turltesim_nav_with_controller = my_behavior_tree.bt_turtlesim_nav_with_controller:tutorial_main',
        ],
    },
)
