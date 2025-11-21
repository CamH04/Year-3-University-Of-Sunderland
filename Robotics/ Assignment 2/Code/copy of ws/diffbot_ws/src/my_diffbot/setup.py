from setuptools import setup

package_name = 'my_diffbot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['my_diffbot/launch/bringup.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/diffbot.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/diff_controllers.yaml']),
        ('share/' + package_name + '/urdf', ['urdf/diffbot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Differential drive robot simulation for Gazebo Harmonic',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmdvel_to_wheels = my_diffbot.cmdvel_to_wheels:main'
        ],
    },
)

