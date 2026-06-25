import os
from glob import glob
from setuptools import find_packages, setup

package_name = "kimm_control"


def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue

        files = [os.path.join(path, filename) for filename in filenames]
        install_path = os.path.join("share", package_name, path)
        data_files.append((install_path, files))

    return data_files


data_files = [
    ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
    (f"share/{package_name}", ["package.xml"]),
    (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    (f"share/{package_name}/config", glob("config/*.yaml")),
]

data_files += package_files("lib/kimm_6rss")

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mariana Bernardes",
    maintainer_email="mcostabernardesmatias@bwh.harvard.edu",
    description="ROS 2 control bridge for the KIMM Stewart platform robot.",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "kimm_robot_node = kimm_control.kimm_robot_node:main",
            "simulated_bridge_node = kimm_control.simulated_bridge:main",
            "hardware_bridge_node = kimm_control.hardware_bridge:main",
        ],
    },
)