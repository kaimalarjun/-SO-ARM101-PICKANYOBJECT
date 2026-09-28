from setuptools import setup

package_name = "kendra_robot"
setup(name=package_name, version="0.1.0", packages=[package_name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                  ("share/" + package_name, ["package.xml"]),
                  ("share/" + package_name + "/launch", ["launch/bringup.launch.py"])],
      install_requires=["setuptools", "PyYAML"], zip_safe=True,
      maintainer="Kendra robot contributors", maintainer_email="noreply@users.noreply.github.com",
      description="Safety-gated SO-ARM101 gateway", license="Apache-2.0",
      entry_points={"console_scripts": [
          "kendra-robot-gateway = kendra_robot.gateway:main",
          "kendra-robot-calibrate = kendra_robot.calibration:main",
          "kendra-robot-watchdog = kendra_robot.watchdog:main",
      ]})
