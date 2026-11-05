from glob import glob

from setuptools import find_packages, setup

package_name = "bhl_hand_sdk"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/firmware", glob("firmware/*.md")),
        ("share/"+package_name+"/preview_model",glob("preview_model/*.*")+["preview_model/LICENSE"]),
        ("share/"+package_name+"/preview_model/meshes/right",glob("preview_model/meshes/right/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="SCPS Lab",
    maintainer_email="jeong510@koreatech.ac.kr",
    description="DexHand control over the arms' CAN bus, with a delivery result per command.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "hand_controller = bhl_hand_sdk.node:main",
            "hand_preview_state = bhl_hand_sdk.preview_state:main",
            "hand_camera_calibration = bhl_hand_sdk.hand_tracking_gui:main",
            "hand_cli = bhl_hand_sdk.cli:main",
            "hand_test = bhl_hand_sdk.demo:main",
            "hand_gui = bhl_hand_sdk.gui:main",
        ],
    },
)
