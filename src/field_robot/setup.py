from setuptools import setup

package_name = 'field_robot'
nodes = ['web_ui_node', 'map_node', 'control_node', 'motor_node',
         'fusion_node', 'mpu_node', 'spray_node', 'trigger_node']

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: ['web/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    entry_points={'console_scripts': [f'{n} = {package_name}.{n}:main' for n in nodes]},
)
