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
    maintainer='monn',
    maintainer_email='monn@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lc_pub = colab_pkg.loadcell_pub:main',
            'lc_sub = colab_pkg.loadcell_sub:main',
            'tilting = colab_pkg.tilting:main',
            'test_tilting = colab_pkg.tilting_test:main',
            'test_ui = colab_pkg.test_ui:main',
            'test_lc = colab_pkg.test_lc:main',
            'test_scale = colab_pkg.scale_driver_test:main',
            'test_cli = colab_pkg.test_cli_servers:main',
            'ctrl_nm = colab_pkg.system_controller_no_mixing:main',     # 혼합 단계 없는 컨트롤러 (테스트용)
            'pour_ctrl = colab_pkg.task_pouring_ctrl:main',              # Pouring 단계 제어 노드 (테스트용)
            'mix_master = colab_pkg.mix_master:main',              # Mixing 단계 제어 노드 (테스트용)
            # 최종 테스트용 (모든 단계)
            'ui = colab_pkg.user_interface:main',
            'ctrl = colab_pkg.system_controller:main',
            'pour = colab_pkg.task_pouring:main',
            'scale = colab_pkg.scale_driver:main',
            'transfer = colab_pkg.task_transfer:main',
            'mixing = colab_pkg.task_mixing:main',

            
            'gripper_test = colab_pkg.gripper_test:main',
            
        ],
    },
)
