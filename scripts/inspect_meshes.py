import trimesh
from pathlib import Path
from xml.etree import ElementTree as E
root=E.parse('/ws/output/mujoco/factory.xml').getroot()
for e in root.findall('asset/mesh'):
 m=trimesh.load(e.get('file'));print(e.attrib,m.extents)
