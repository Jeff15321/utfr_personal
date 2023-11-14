import os
from glob import glob
from setuptools import setup

package_name = "visualization"
submodules = "visualization/submodules"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name, submodules],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alfred",
    maintainer_email="alfred.xue@mail.utoronto.ca",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["visualization = visualization.visualization_node:main"],
    },
)
