from setuptools import find_packages, setup

package_name = 'px4_indoor_navigation'

setup(
    name=package_name,
    version='0.0.0',
    keywords=['ROS'],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/start_alt_vxy_controller.launch.py']),
        ('share/' + package_name, ['config/alt_vxy_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='invdro',
    maintainer_email='invdro@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'altitude_vxy_controller = px4_indoor_navigation.altitude_vxy_controller:main',
            
        
        ],
    },
)

