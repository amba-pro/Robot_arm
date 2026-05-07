from setuptools import setup
from glob import glob

package_name = "arm4_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="arm4_user",
    maintainer_email="user@example.com",
    description="ROS 2 bringup package for Robot ARM4.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "angle_cache_publisher = arm4_bringup.angle_cache_publisher:main",
        ],
    },
)
