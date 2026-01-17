import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isac_libs_sensing_example'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join(package_name, "launch", "*launch.[pxy][yma]*")),
        )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fede3751',
    maintainer_email='trombetti@di.uniroma1.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "agent_controller="+package_name+".isac_devices.agent_controller:main",
            "isac_signal_display="+package_name+".isac_signal_display:main",
        ],
    },
)
