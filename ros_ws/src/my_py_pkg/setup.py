from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
    maintainer='dpma',
    maintainer_email='dter.mell@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mpc_diffdrive_controller_node = my_py_pkg.mpc_diffdrive_control:main',
            'mpcc_diffdrive_controller_node = my_py_pkg.mpcc_diffdrive_control:main',
            'keyboard_control_node = my_py_pkg.keyboard_control_node:main'
        ],
    },
)
