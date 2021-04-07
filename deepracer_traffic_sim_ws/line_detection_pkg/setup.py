from setuptools import setup
import os

package_name = "line_detection_pkg"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), ["launch/line_detection_pkg_launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AWS DeepRacer",
    maintainer_email="aws-deepracer@amazon.com",
    description="This package contains logic for line detection from input camera images",
    license="Apache 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "line_detection_node = line_detection_pkg.line_detection_node:main"
        ],
    },
)
