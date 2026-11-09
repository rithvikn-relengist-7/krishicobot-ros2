from setuptools import setup

package_name = "krishicobot"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="Rithvik N",
    maintainer_email="rithvik.n2024@vitstudent.ac.in",
    description="ROS 2 navigation and depth-assisted fruit perception modules",
    license="NOASSERTION",
    entry_points={
        "console_scripts": [
            "bug_navigator = krishicobot.bug_navigator:main",
            "fruit_tf_detector = krishicobot.fruit_tf_detector:main",
            "ur5_waypoint_controller = krishicobot.ur5_waypoint_controller:main",
        ],
    },
)
