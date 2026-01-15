import os
import warnings
from glob import glob
from setuptools import find_packages, setup

warnings.filterwarnings("ignore")

package_name = "isac_libs_main"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("lib", package_name, "utils"),
            glob(package_name + "/utils/*.py")
        ),
            
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fede3751",
    maintainer_email="trombetti@di.uniroma1.it",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        # This package does not offer any entry points by itself, but only classes to extend,
        # and utils files to use, which are compiled above in the "data_files".
        "console_scripts": [
        ],
    },
)
