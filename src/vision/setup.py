from setuptools import find_packages, setup

setup(name="xifeng_vision", version="0.1.0", packages=find_packages(),
      data_files=[("share/ament_index/resource_index/packages", ["resource/xifeng_vision"]),
                  ("share/xifeng_vision", ["package.xml"]),
                  ("share/xifeng_vision/config", ["config/object_catalog.v1.json"])],
      install_requires=["setuptools", "numpy"], zip_safe=True,
      entry_points={"console_scripts": [
          "rgbd_observer = vision_system.rgbd_observer_node:main",
          "vision_replay = vision_system.replay:main"]})
