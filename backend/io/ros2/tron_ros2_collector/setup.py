from setuptools import setup

package_name = "tron_ros2_collector"
setup(
    name=package_name, version="0.1.0", packages=[package_name],
    data_files=[("share/ament_index/resource_index/packages", ["resource/" + package_name]),
                ("share/" + package_name, ["package.xml"])],
    install_requires=["setuptools"], zip_safe=True,
    entry_points={"console_scripts": [
        "collector = tron_ros2_collector.collector:main",
        "rogue_publisher = tron_ros2_collector.rogue_publisher:main",
    ]},
)
