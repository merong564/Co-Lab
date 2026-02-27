from setuptools import find_packages, setup

package_name = 'colab_pkg'

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
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ui = colab_pkg.user_interface:main',
            'ui2 = colab_pkg.user_interface_virtual:main',
            'test = colab_pkg.test_msg_ui:main',
            'pour = colab_pkg.task_pouring_ctrl_ver2:main',
            'scale = colab_pkg.scale_driver_ver2:main',
            'noise = colab_pkg.noise_analyzer:main',
            'mock = colab_pkg.mock_controller:main',
            'mix = colab_pkg.task_mixing_copy:main',
            'scale_lpf = colab_pkg.scale_driver_ver2_lpf:main',
            'scale_alpf = colab_pkg.scale_driver_ver2_alpf:main',
            'scale_kalman = colab_pkg.scale_driver_ver2_kalman:main',
        ],
    },
)
