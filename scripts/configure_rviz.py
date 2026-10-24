"""Generate a bounded and legible RViz dashboard; output is committed as YAML."""
from pathlib import Path
import yaml

root = Path('/ws/src/qda_bringup/rviz/factory.rviz')
def topic(value, latched=False, sensor=False):
    return {'Value': value, 'Depth': 2, 'History Policy': 'Keep Last', 'Durability Policy': 'Transient Local' if latched else 'Volatile', 'Reliability Policy': 'Best Effort' if sensor else 'Reliable'}
def display(cls, name, tp=None, enabled=True, **attrs):
    obj = {'Class': 'rviz_default_plugins/'+cls, 'Name': name, 'Enabled': enabled}
    if tp: obj['Topic'] = tp
    obj.update(attrs)
    return obj
def group(name, items):
    return {'Class': 'rviz_common/Group', 'Name': name, 'Enabled': True, 'Displays': items}
def path(name, tp, color, width=.03):
    return display('Path', name, topic(tp, tp.startswith('/viz/')), Color=color, **{'Line Style':'Billboards', 'Line Width':width, 'Buffer Length':1})
def cloud(name, tp, transformer, enabled, decay=0):
    return display('PointCloud2', name, topic(tp, sensor=True), enabled, **{'Style':'Points','Size (m)':0.025, 'Size (Pixels)':2, 'Color Transformer':transformer, 'Position Transformer':'XYZ','Channel Name':'rgb','Axis':'Z','Decay Time':decay,'Use Fixed Frame':True})
cfg = {
 'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'},{'Class':'rviz_common/Views','Name':'Views'}],
 'Visualization Manager':{'Class':'','Global Options':{'Background Color':'23; 29; 39','Fixed Frame':'odom','Frame Rate':15},
 'Displays':[
   display('Grid','Ground reference',None,True, **{'Cell Size':.5,'Plane Cell Count':24,'Color':'65; 78; 93'}),
   display('RobotModel','B2 + dual Aubo',None,True, **{'Description Source':'Topic','Description Topic':topic('/robot_description',True),'Collision Enabled':False,'Visual Enabled':True}),
   display('MarkerArray','Task / backend / reference stations',topic('/viz/markers',True)),
   group('01 Navigation',[
    display('MarkerArray','Obstacle / inflation grid',topic('/viz/grids',True)),
    display('Map','Occupancy texture (optional)',topic('/map',True),False, Alpha=.45, **{'Color Scheme':'map','Draw Behind':True}),
    display('Map','Local cost texture (optional)',topic('/local_costmap/costmap',True),False, Alpha=.5, **{'Color Scheme':'costmap'}),
    display('Map','Global costs (optional)',topic('/global_costmap/costmap',True),False, Alpha=.35, **{'Color Scheme':'costmap'}),
    path('Global plan - cyan','/viz/global_plan','40; 195; 255',.035),
    path('Executed base path - green','/viz/actual_path','70; 235; 155',.035),
    path('MPPI transformed plan','/transformed_global_plan','255; 195; 55',.022),
    display('MarkerArray','MPPI selected / sampled rollouts',topic('/trajectories')),
    display('Polygon','Collision footprint',topic('/local_costmap/published_footprint'),True, Color='255; 100; 80'),
    display('Pose','Arm approach goal',topic('/task/goal'),True, Color='255; 110; 90', **{'Shaft Length':.25,'Shaft Radius':.02,'Head Length':.08,'Head Radius':.05})]),
   group('02 Manipulation',[
    path('Planned left tool path - purple','/viz/planned_tool_path','205; 130; 255',.02),
    path('Measured left tool path - gold','/viz/tool_path','255; 200; 75',.02),
    display('MarkerArray','RGB-D targets',topic('/perception/markers'))]),
   group('03 Mapping',[
    cloud('LIVO colored scan / odom aligned (2 s)','/cloud_registered','RGB8',True,2),
    cloud('Measured geometry / height','/mapping/cloud','AxisColor',True),
    cloud('Raw Mid360 scan','/livox/lidar_PointCloud2','AxisColor',False),
    path('LIVO estimated path (diagnostic)','/path','225; 110; 240',.02)]),
   group('04 Cameras',[
    display('Image','RGB-D detection',topic('/perception/debug')),
    display('Image','Factory overview',topic('/overview/image_raw',sensor=True)),
    display('Image','Raw camera (optional)',topic('/camera/image_raw',sensor=True),False)]),
   display('TF','TF axes (optional)',None,False, **{'Show Names':False,'Show Axes':True,'Show Arrows':False,'Marker Scale':.1})],
 'Tools':[{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Interact'},{'Class':'rviz_default_plugins/FocusCamera'}],
 'Views':{'Current':{'Class':'rviz_default_plugins/Orbit','Distance':6.2,'Focal Point':{'X':1.1,'Y':-.25,'Z':.55},'Pitch':.70,'Yaw':.95,'Name':'Algorithm overview','Target Frame':'odom'}}},
 'Window Geometry':{'Height':950,'Width':1500,'X':70,'Y':40,'Displays':{'collapsed':False},'Views':{'collapsed':True}}}
root.write_text(yaml.safe_dump(cfg,sort_keys=False,allow_unicode=True))
