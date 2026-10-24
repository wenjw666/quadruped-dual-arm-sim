"""Deterministic CAD-like shells; all units are metres, no texture dependencies."""
from pathlib import Path
import numpy as np
import trimesh
root=Path('/ws/src/qda_description/meshes/refined');root.mkdir(parents=True,exist_ok=True)
def rounded_box(size,radius,path):
 x,y,z=size
 outline=[]
 for cx,cy,angle in [(x/2-radius,y/2-radius,0),(-x/2+radius,y/2-radius,90),(-x/2+radius,-y/2+radius,180),(x/2-radius,-y/2+radius,270)]:
  for a in np.linspace(angle,angle+90,13):outline.append([cx+radius*np.cos(np.radians(a)),cy+radius*np.sin(np.radians(a))])
 # Outline here proceeds from NE east through north, then NW etc.
 verts=np.array([[a,b,c] for c in [-z/2,z/2] for a,b in outline]);n=len(outline);faces=[]
 verts=np.vstack((verts,[[0,0,-z/2],[0,0,z/2]]))
 for i in range(n):
  j=(i+1)%n;faces.extend([[i,j,j+n],[i,j+n,i+n],[2*n,j,i],[2*n+1,i+n,j+n]])
 m=trimesh.Trimesh(vertices=verts,faces=faces);m.fix_normals();m.export(root/path)
 return m
rounded_box((.54,.40,.036),.032,'deck_shell.stl')
profile=np.array([[0,0],[.073,0],[.073,.012],[.067,.024],[.058,.047],[.055,.19],[.052,.235],[.052,.268],[0,.268]])
trimesh.creation.revolve(profile,sections=64).export(root/'column_shell.stl')
m=rounded_box((.198,.052,.044),.022,'camera_housing.stl')
# Construction box's long X becomes camera Y, thin Z becomes optical X.
m.vertices=m.vertices[:,[2,0,1]];m.fix_normals();m.export(root/'camera_housing.stl')
m=rounded_box((.172,.025,.002),.012,'camera_glass.stl');m.vertices=m.vertices[:,[2,0,1]];m.vertices[:,0]+=.0225;m.fix_normals();m.export(root/'camera_glass.stl')
print('Generated',list(root.glob('*.stl')))
# Additional visible service cable on the rear deck, with a gentle bend radius.
def tube(points,radius=.003):
 pieces=[]
 for a,b in zip(points[:-1],points[1:]):
  direction=np.asarray(b)-a
  c=trimesh.creation.cylinder(radius=radius,height=np.linalg.norm(direction),sections=12)
  matrix=trimesh.geometry.align_vectors([0,0,1],direction);matrix[:3,3]=(np.asarray(a)+b)/2
  c.apply_transform(matrix);pieces.append(c)
 return trimesh.util.concatenate(pieces)
t=np.linspace(0,1,45)
p0=np.array([.085,-.072,.048]);p1=np.array([-.27,-.17,.055]);p2=np.array([-.24,.08,.055]);p3=np.array([-.15,.03,.06])
points=((1-t)**3)[:,None]*p0+(3*(1-t)**2*t)[:,None]*p1+(3*(1-t)*t*t)[:,None]*p2+(t**3)[:,None]*p3
tube(points).export(root/'service_cable.stl')

