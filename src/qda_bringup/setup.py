from setuptools import setup

package_name = "qda_bringup"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/pick_place.launch.py"]),
        ("share/" + package_name + "/rviz", ["rviz/factory.rviz"]),
        ("share/" + package_name + "/config", ["config/nav2_quad.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={'console_scripts': ['visualization_node = qda_bringup.visualization_node:main']},
)
