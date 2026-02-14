from setuptools import find_packages, setup

package_name = 'cobot1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='abbeyroad1027@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        	'move_basic = cobot1.move_basic:main',
        	'simple_move = cobot1.simple_move:main',
            	'move_periodic = cobot1.move_periodic:main',
            	'force_test = cobot1.force_test:main',
            	'pour_test = cobot1.pouring_simulation_260214_v2:main',
        ],
    },
)
