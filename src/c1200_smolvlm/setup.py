from setuptools import setup

package_name = "c1200_smolvlm"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/smolvlm_minimal.launch.py"]),
        ("share/" + package_name + "/config", ["config/smolvlm.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "camera_node = c1200_smolvlm.camera_node:main",
            "vlm_node = c1200_smolvlm.vlm_node:main",
        ]
    },
)
