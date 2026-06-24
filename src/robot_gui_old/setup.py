from glob import glob
from setuptools import find_packages, setup

package_name = "robot_gui"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/ui", glob("ui/*.ui")),
        ("share/" + package_name + "/ui", glob("ui/*.png")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Minh Quang Tran",
    maintainer_email="tranminhquang617@gmail.com",
    description="Qt GUI for the 6DOF robot using a Qt Designer .ui file.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "robot_gui = robot_gui.main:main",
        ],
    },
)
