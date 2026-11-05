from glob import glob
from setuptools import find_packages, setup

name = "bhl_upper_body_sdk"
setup(name=name, version="0.2.0", packages=find_packages(exclude=["test"]),
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + name]),
                  ("share/" + name, ["package.xml", "README.md"]),
                  ("share/" + name + "/config", glob("config/*.yaml")),
                  ("share/" + name + "/examples", glob("examples/*.py"))],
      install_requires=["setuptools"], zip_safe=True, maintainer="SCPS Lab",
      maintainer_email="jeong510@koreatech.ac.kr", description="Coordinated arm/hand target contract and commissioning tools",
      license="MIT", entry_points={"console_scripts": [
          "upper_body_controller = bhl_upper_body_sdk.node:main",
          "upper_body_commission = bhl_upper_body_sdk.commissioning:main"]})
