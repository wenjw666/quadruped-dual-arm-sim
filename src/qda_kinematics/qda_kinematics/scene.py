"""Shared layout of the factory pick-and-place world, in meters."""
import os
import math

# B2 hip height 0.50 m plus the foot sphere, with a small gap under the feet.
SPAWN_XYZ = (0.0, 0.0, 0.56)
SPAWN_YAW = 0.0

# A3 bases on the pedestals, in base_link. Z points up and the mounts yaw outward.
from .assembly import LEFT_ARM_BASE, RIGHT_ARM_BASE

# Downward pad IK solves for x in about [0.58, 0.70]. 0.64 leaves margin for
# the last few centimetres of base error, and the nose stays ~4 cm off the bench.
APPROACH_IN_BASE = (0.64, 0.32)

TABLE_TOP_Z = 0.55
TABLE_SIZE = (0.80, 0.55, 0.55)
OBJECT_SIZE = 0.04
OBJECT_Z = TABLE_TOP_Z + OBJECT_SIZE / 2.0

PICK_OBJECT_XYZ = (0.78, 0.16, OBJECT_Z)
# South of the aisle so the stopped body does not stand inside the rack.
PLACE_MARKER_XYZ = (2.15, -1.00, TABLE_TOP_Z + 0.004)
PLACE_OBJECT_Z = OBJECT_Z

# Rack sits south of the pick bench and across the diagonal to the place stance.
# The place stance is far enough south that the body clears this rack.
RACK_XYZ = (1.05, -0.48, 0.55)
RACK_SIZE = (0.42, 0.36, 1.10)


def table_center(object_xy):
    # Keep the near edge ahead of the B2 chest when the base is at the approach pose.
    return (object_xy[0] + 0.22, object_xy[1], TABLE_SIZE[2] / 2.0)


PICK_TABLE_XYZ = table_center(PICK_OBJECT_XYZ)
PLACE_TABLE_XYZ = table_center((PLACE_MARKER_XYZ[0], PLACE_MARKER_XYZ[1]))

# One selected profile is inherited by all ROS processes and both backends.
# Route hints are checkpoints, never a replacement for Nav2's computed path.
SCENARIO = os.environ.get('QDA_SCENARIO', 'auto')
if SCENARIO not in ('legacy', 'auto', 'north', 'south', 'side'):
    raise ValueError(f'Unknown QDA_SCENARIO: {SCENARIO}')
EXPANDED = SCENARIO != 'legacy'
PLACE_YAW = math.pi / 2 if SCENARIO == 'side' else 0.0
PICK_YAW = 0.0
PLACE_TABLE_SIZE = TABLE_SIZE
ROUTE_OPTIONS = {
    'north': ((-.25, 2.25, 0.), (3.7, 2.25, 0.)),
    'south': ((.1, -1.85, 0.), (3.7, -1.85, 0.)),
}
ROUTE_WAYPOINTS = ROUTE_OPTIONS.get('north' if SCENARIO == 'side' else SCENARIO, ()) if EXPANDED else ()
if SCENARIO=='side':
    # Rotate in the open bay before approaching the table's perpendicular edge.
    ROUTE_WAYPOINTS = (*ROUTE_OPTIONS['north'],(3.7,1.8,math.pi/2))
EXTRA_BOXES = []
if EXPANDED:
    BOUNDS = (-2., 7., -3.8, 4.5)
    PLACE_MARKER_XYZ = (4.9, 2.8, TABLE_TOP_Z + .004) if SCENARIO == 'side' else (5., .2, TABLE_TOP_Z + .004)
    c, s = math.cos(PLACE_YAW), math.sin(PLACE_YAW)
    PLACE_TABLE_XYZ = (PLACE_MARKER_XYZ[0] + .22*c, PLACE_MARKER_XYZ[1] + .22*s, TABLE_SIZE[2]/2)
    if SCENARIO == 'side': PLACE_TABLE_SIZE = (TABLE_SIZE[1], TABLE_SIZE[0], TABLE_SIZE[2])
    RACK_XYZ, RACK_SIZE = (2.4, .15, .7), (.65, 2.2, 1.4)
    other_center, other_size = ((5.22,.2,.275),TABLE_SIZE) if SCENARIO=='side' else ((4.9,3.02,.275),(.55,.8,.55))
    EXTRA_BOXES = [
        ('standby_station',other_center,other_size,'Gazebo/Wood'),
        ('shelf_north',(1.4,3.9,.8),(1.7,.4,1.6),'Gazebo/DarkGrey'),
        ('shelf_south',(4.6,-3.2,.7),(1.7,.4,1.4),'Gazebo/Grey'),
        ('column_east',(6.5,3.8,1.2),(.3,.3,2.4),'Gazebo/Grey'),
    ]
else:
    BOUNDS = (-1.3,4.1,-2.2,2.8)

def world_boxes():
    """Shared collision boxes: (name, center, size, material)."""
    x0,x1,y0,y1=BOUNDS
    walls=[('wall_west',(x0,(y0+y1)/2,1.2),(.15,y1-y0,2.4),'Gazebo/Bricks'),
           ('wall_east',(x1,(y0+y1)/2,1.2),(.15,y1-y0,2.4),'Gazebo/Bricks'),
           ('wall_south',((x0+x1)/2,y0,1.2),(x1-x0,.15,2.4),'Gazebo/Bricks'),
           ('wall_north',((x0+x1)/2,y1,1.2),(x1-x0,.15,2.4),'Gazebo/Bricks')]
    if not EXPANDED:
        walls[0]=('wall_west',(-1.3,.4,1.2),(.15,5.2,2.4),'Gazebo/Bricks')
        walls[1]=('wall_east',(4.1,.4,1.2),(.15,5.2,2.4),'Gazebo/Bricks')
        walls[3]=('wall_north',(1.4,2.8,1.2),(5.4,.15,2.4),'Gazebo/Bricks')
    return walls + [('aisle_rack',RACK_XYZ,RACK_SIZE,'Gazebo/Grey')] + EXTRA_BOXES

def planning_boxes():
    return [('pick_table',PICK_TABLE_XYZ,TABLE_SIZE,True),
            ('place_table',PLACE_TABLE_XYZ,PLACE_TABLE_SIZE,True)] + [(n,p,s,n=='standby_station') for n,p,s,_ in world_boxes()]

def scene_manifest():
    return dict(scenario=SCENARIO,bounds=BOUNDS,pick=PICK_OBJECT_XYZ,place=PLACE_MARKER_XYZ,
                place_yaw=PLACE_YAW,waypoints=ROUTE_WAYPOINTS,route_options=ROUTE_OPTIONS,
                boxes=planning_boxes(),navigation_map='sensor-built occupancy',robot='refined_v1',
                motion='prescribed kinematics',grasp='virtual fixed attachment')
