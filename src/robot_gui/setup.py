from setuptools import setup, find_packages

package_name = "robot_gui"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/robot_gui.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Robot GUI: RS485 hardware + TF + MoveIt planning (QtPy)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "robot_gui = robot_gui.app:main",
        ],
    },
)
