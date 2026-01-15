import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'isac_libs_toy_example'
main_package = 'isac_libs_main'

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join(package_name, "launch", "*launch.[pxy][yma]*")),
        ),
        (
            os.path.join("lib", package_name, "utils"),
            glob(package_name + "/utils/*.py")
        ),
        (
            os.path.join("lib", package_name, "utils"),
            glob(main_package + "/utils/*.py")
        ),
        (
            os.path.join("lib", package_name, "utils"),
            glob(main_package + "/*.py")
        ),
        (
            os.path.join("lib", package_name, "cache_types"),
            glob(package_name + "/cache_types/*.py")
        )
            
    ],
    install_requires=["setuptools", "isac_libs_main"],
    zip_safe=True,
    maintainer="fede3751",
    maintainer_email="trombetti@di.uniroma1.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            
            "application_controller="+package_name+".orchestrators.application_controller:main",
            "agent_network_manager="+package_name+".orchestrators.agent_network_manager:main",


            "edge_device_controller="+package_name+".isac_devices.edge_device_controller:main",
            "relay_device_controller="+package_name+".isac_devices.relay_device_controller:main",
            "agent_controller="+package_name+".isac_devices.agent_controller:main"
            
        ],
    },
)
