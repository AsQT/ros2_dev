import os
from setuptools import setup

package_name = "robot_drl"

# Collect all files under the package-level models/ directory.
# "models/" is a sibling of setup.py (src/robot_drl/models/).
# Do NOT use package_name + "/models" — that searches inside the Python package
# subdirectory (src/robot_drl/robot_drl/models/) which is wrong.
model_files = []
for root, dirs, files in os.walk("models"):
    for f in files:
        src = os.path.join(root, f)
        if not os.path.isfile(src):
            continue
        # e.g. "models/run/model/best_model.zip"
        # Strip the "models/" prefix so install goes to share/robot_drl/models/...
        rel_path = os.path.relpath(src, "models")          # "run/model/best_model.zip"
        install_dir = os.path.join("share", package_name, "models",
                                   os.path.dirname(rel_path))  # "share/robot_drl/models/run/model"
        model_files.append((install_dir, [src]))

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/" + f for f in os.listdir("launch") if f.endswith(".launch.py")]),
        ("share/" + package_name + "/rviz", ["rviz/" + f for f in os.listdir("rviz") if f.endswith(".rviz")]),
    ] + model_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Norman",
    maintainer_email="norman@example.com",
    description="DRL unified planner node (DDPG/SAC/TD3) for robot",
    license="MIT",
    extras_require={"dev": ["pytest"]},
    entry_points={
        "console_scripts": [
            "drl_unified_planner_node = robot_drl.drl_unified_planner_node:main",
            "mock_environment_node = robot_drl.mock_environment_node:main",
            "mock_hw_obstacle_test = robot_drl.mock_hw_obstacle_test:main",
            "gazebo_obstacle_test = robot_drl.gazebo_obstacle_test:main",
        ],
    },
)
