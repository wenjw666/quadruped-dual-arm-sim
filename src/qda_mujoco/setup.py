from setuptools import setup
setup(name='qda_mujoco', version='0.1.0', packages=['qda_mujoco'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/qda_mujoco']),
                  ('share/qda_mujoco', ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      entry_points={'console_scripts': ['mujoco_node = qda_mujoco.node:main']})
