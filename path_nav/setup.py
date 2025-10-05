from setuptools import find_packages, setup
import glob

package_name = 'path_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')), # launch files
        ('share/' + package_name + '/rviz',glob.glob('rviz/*.rviz')), # rviz save files
        ('share/' + package_name + '/test', glob.glob('test/*.py')), # test files
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shamganesh',
    maintainer_email='shamganesh@todo.todo',
    description='Robot path smoothing',
    license='self',
    entry_points={
        'console_scripts': [
            'trajectory_publisher = path_nav.trajectory_publisher:main',
            'pure_pursuit_controller = path_nav.pure_pursuit_controller:main',
        ],
    },
)
