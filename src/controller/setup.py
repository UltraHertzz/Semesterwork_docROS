from setuptools import find_packages, setup

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/control_launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tumi6',
    maintainer_email='shixin.li@tum.de',
    description='Controller Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'game_pad_node = controller.game_pad:main',
            'key_board_node = controller.key_board:main'
        ],
    },
)
