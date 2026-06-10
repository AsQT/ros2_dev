import os
from setuptools import setup

package_name = "gp7_drl_inference"

# Collect all files under the package-level models/ directory.
# "models/" is a sibling of setup.py (src/gp7_drl_inference/models/).
# Do NOT use package_name + "/models" — that searches inside the Python package
# subdirectory (src/gp7_drl_inference/gp7_drl_inference/models/) which is wrong.
model_files = []
for root, dirs, files in os.walk("models"):
    for f in files:
        src = os.path.join(root, f)
        # e.g. "models/run/model/best_model.zip"
        # Strip the "models/" prefix so install goes to share/gp7_drl_inference/models/...
        rel_path = os.path.relpath(src, "models")          # "run/model/best_model.zip"
        install_dir = os.path.join("share", package_name, "models",
                                   os.path.dirname(rel_path))  # "share/gp7_drl_inference/models/run/model"
        model_files.append((install_dir, [src]))

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/" + f for f in os.listdir("launch") if f.endswith(".launch.py")]),
    ] + model_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Norman",
    maintainer_email="norman@example.com",
    description="DRL unified planner node (DDPG/SAC/TD3) for Yaskawa GP7",
    license="MIT",
    extras_require={"dev": ["pytest"]},
    entry_points={
        "console_scripts": [
            "drl_unified_planner_node = gp7_drl_inference.drl_unified_planner_node:main",
            "mock_environment_node = gp7_drl_inference.mock_environment_node:main",
        ],
    },
)
