from setuptools import setup

package_name = "qda_kinematics"
setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    package_data={package_name: ["residual_policy.npz"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
)
