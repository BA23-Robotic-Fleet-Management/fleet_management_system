import os
from glob import glob
from setuptools import setup, find_packages

package_name = "free_fleet_rmf_adapter"

setup(
    name=package_name,
    version="1.4.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, ["config.yaml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.xml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Hussein Kabbout, Navid Sassan",
    maintainer_email="hussein.kabbout@hotmail.com,sassanav@students.zhaw.ch",
    description="TODO_DESCRIPTION",
    # TODO: Is this allowed?
    license="TODO_LICENSE",
    entry_points={
        "console_scripts": [
            "adapter=src.adapter:main",
        ],
    },
)
