from setuptools import find_packages, setup
from glob import glob

package_name = 'puzzlebot_gazebo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/gazebo', glob('gazebo/*.sdf')),
        ('share/' + package_name + '/gazebo/plugins/', glob('gazebo/plugins/*.so')),
        ('share/' + package_name + '/models', glob('models/*.stl')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alfredo',
    maintainer_email='josfrend17@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
