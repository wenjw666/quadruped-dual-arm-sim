from setuptools import setup
setup(name='qda_brain',version='0.2.0',packages=['qda_brain'],
      data_files=[('share/ament_index/resource_index/packages',['resource/qda_brain']),('share/qda_brain',['package.xml'])],
      install_requires=['setuptools'],entry_points={'console_scripts':[
          'brain_node=qda_brain.brain_node:main','skill_server=qda_brain.skill_server:main',
          'visual_servo=qda_brain.visual_servo:main','robot_mcp=qda_brain.mcp_server:main',
          'mission_coordinator=qda_brain.mission_node:main']})
