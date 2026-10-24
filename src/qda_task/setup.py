from setuptools import setup

package_name = "qda_task"
setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "pick_place_node = qda_task.pick_place_node:main",
            "quad_nav_node = qda_task.quad_nav_node:main",
        ]
    },
)
