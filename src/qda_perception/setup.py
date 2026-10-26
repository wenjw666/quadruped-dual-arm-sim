from setuptools import setup

package_name = "qda_perception"
setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "detector_node = qda_perception.detector_node:main",
            "frame_dump = qda_perception.frame_dump:main",
            "mapper_node = qda_perception.mapper_node:main",
        ]
    },
)
