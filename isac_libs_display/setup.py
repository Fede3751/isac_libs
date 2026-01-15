import os
from glob import glob
from setuptools import find_packages, setup


package_name = 'isac_libs_display'
main_pkg_name = 'isac_libs_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join(package_name, "launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("lib", package_name, "utils"),
            glob(main_pkg_name + "/utils/*.py")
        ),
        (
            os.path.join("lib", package_name, "cache_types"),
            glob(main_pkg_name + "/cache_types/*.py")
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
            'display_controller = '+package_name+'.display_controller:main',
            
            'edge_renderer = '+package_name+'.edge_renderer:main',
            'sensor_renderer = '+package_name+'.sensor_renderer:main',
            'agent_renderer = '+package_name+'.agent_renderer:main'
        ],
    },
)
