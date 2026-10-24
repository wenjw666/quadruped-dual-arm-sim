"""Factory world built from Gazebo's built-in materials."""

from qda_kinematics.scene import (
    EXPANDED, BOUNDS, PLACE_TABLE_SIZE, world_boxes,
    OBJECT_SIZE,
    OBJECT_Z,
    PLACE_MARKER_XYZ,
    PLACE_TABLE_XYZ,
    PICK_OBJECT_XYZ,
    PICK_TABLE_XYZ,
    RACK_SIZE,
    RACK_XYZ,
    SPAWN_XYZ,
    TABLE_SIZE,
)


def _material(name: str) -> str:
    return f"""
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{name}</name>
            </script>
          </material>"""


def _box_model(name, xyz, size, material, static, mass=0.0, link="link", collision=True, rpy="0 0 0"):
    x, y, z = xyz
    sx, sy, sz = size
    inertial = ""
    if not static:
        inertial = f"""
        <inertial>
          <mass>{mass}</mass>
          <inertia>
            <ixx>1e-4</ixx><iyy>1e-4</iyy><izz>1e-4</izz>
            <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
          </inertia>
        </inertial>"""
    static_tag = "<static>true</static>" if static else ""
    collision_xml = ""
    if collision:
        collision_xml = f"""
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          <surface>
            <friction><ode><mu>0.9</mu><mu2>0.9</mu2></ode></friction>
            <contact><ode><kp>1e5</kp><kd>50</kd></ode></contact>
          </surface>
        </collision>"""
    return f"""
    <model name="{name}">
      {static_tag}
      <pose>{x} {y} {z} {rpy}</pose>
      <link name="{link}">
        {inertial}
        {collision_xml}
        <visual name="visual">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
          {_material(material)}
        </visual>
      </link>
    </model>"""


def _workbench(name, xyz, size):
    """Grey frame and a wood top. Collision is the whole bench."""
    x, y, z = xyz
    sx, sy, sz = size
    top = 0.04
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {sz}</size></box></geometry>
        </collision>
        <visual name="top">
          <pose>0 0 {sz / 2 - top / 2} 0 0 0</pose>
          <geometry><box><size>{sx} {sy} {top}</size></box></geometry>
          {_material("Gazebo/Wood")}
        </visual>
        <visual name="body">
          <pose>0 0 {-top / 2} 0 0 0</pose>
          <geometry><box><size>{sx * 0.92} {sy * 0.92} {sz - top}</size></box></geometry>
          {_material("Gazebo/DarkGrey")}
        </visual>
      </link>
    </model>"""


def _cylinder(name, xyz, radius, length, material):
    x, y, z = xyz
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} {z} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><cylinder><radius>{radius}</radius><length>{length}</length></cylinder></geometry>
          {_material(material)}
        </visual>
      </link>
    </model>"""


def world_xml(fast_sim: bool) -> str:
    rate = 0 if fast_sim else 1000
    z = SPAWN_XYZ[2]
    pick_table = _workbench("pick_table", PICK_TABLE_XYZ, TABLE_SIZE)
    place_table = _workbench("place_table", PLACE_TABLE_XYZ, PLACE_TABLE_SIZE)
    rack = _box_model("aisle_rack", RACK_XYZ, RACK_SIZE, "Gazebo/Grey", True)
    pick = _box_model(
        "pick_object",
        PICK_OBJECT_XYZ,
        (OBJECT_SIZE, OBJECT_SIZE, OBJECT_SIZE),
        "Gazebo/Red",
        False,
        mass=0.04,
        link="object_link",
    )
    marker = _box_model(
        "place_marker",
        PLACE_MARKER_XYZ,
        (0.10, 0.10, 0.008),
        "Gazebo/Green",
        True,
    )
    walls = "\n".join(
        [
            _box_model("wall_west", (-1.3, 0.4, 1.2), (0.15, 5.2, 2.4), "Gazebo/Bricks", True),
            _box_model("wall_east", (4.1, 0.4, 1.2), (0.15, 5.2, 2.4), "Gazebo/Bricks", True),
            _box_model("wall_south", (1.4, -2.2, 1.2), (5.4, 0.15, 2.4), "Gazebo/Bricks", True),
            _box_model("wall_north", (1.4, 2.8, 1.2), (5.4, 0.15, 2.4), "Gazebo/Bricks", True),
            _box_model("floor_pad", (1.4, 0.3, 0.005), (5.2, 4.8, 0.01), "Gazebo/CeilingTiled", True, collision=False),
            _box_model("lane_line", (1.5, -0.7, 0.012), (3.2, 0.08, 0.004), "Gazebo/Yellow", True, collision=False),
            _box_model("shelf_a", (3.4, 1.6, 0.9), (0.45, 1.6, 1.8), "Gazebo/FlatBlack", True),
            _box_model("shelf_b", (3.4, -0.4, 0.7), (0.45, 1.2, 1.4), "Gazebo/DarkGrey", True),
            _box_model("column_a", (0.2, 1.8, 1.2), (0.28, 0.28, 2.4), "Gazebo/Grey", True),
            _box_model("column_b", (2.8, -1.5, 1.2), (0.28, 0.28, 2.4), "Gazebo/Grey", True),
        ]
    )
    drums = "\n".join(
        [
            _cylinder("drum_a", (-0.4, 1.3, 0.28), 0.18, 0.55, "Gazebo/Blue"),
            _cylinder("drum_b", (-0.4, 1.7, 0.28), 0.18, 0.55, "Gazebo/Orange"),
            _cylinder("drum_c", (3.5, 0.6, 0.22), 0.16, 0.44, "Gazebo/ZincYellow"),
        ]
    )
    overview_pose = '1.4 -2.0 2.2 0 0.7 1.5708'
    if EXPANDED:
        x0,x1,y0,y1=BOUNDS
        walls='\n'.join(_box_model(n,p,s,mat,True) for n,p,s,mat in world_boxes() if n!='aisle_rack')
        walls+=_box_model('floor_pad',((x0+x1)/2,(y0+y1)/2,.005),(x1-x0,y1-y0,.01),'Gazebo/CeilingTiled',True,collision=False)
        for name,y,material in [('north',2.25,'Gazebo/Blue'),('south',-1.85,'Gazebo/Yellow')]:
            walls+=_box_model('route_'+name,(1.8,y,.012),(4.1,.055,.004),material,True,collision=False)
        drums=''
        overview_pose='2.5 -4.8 7.8 0 0.96 1.5708'
    return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <world name="qda_factory">
    <gravity>0 0 -9.81</gravity>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.62 0.68 0.74 1</background>
      <shadows>true</shadows>
    </scene>
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <real_time_update_rate>{rate}</real_time_update_rate>
      <ode>
        <solver><type>quick</type><iters>40</iters><sor>1.3</sor></solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>10</contact_max_correcting_vel>
          <contact_surface_layer>0.004</contact_surface_layer>
        </constraints>
      </ode>
    </physics>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <model name="spawn_anchor">
      <static>true</static>
      <pose>0 0 {z} 0 0 0</pose>
      <link name="link">
        <inertial>
          <mass>1</mass>
          <inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
      </link>
    </model>
    {walls}
    {drums}
    {pick_table}
    {place_table}
    {rack}
    {pick}
    {marker}
    <model name="overview_cam">
      <static>true</static>
      <pose>{overview_pose}</pose>
      <link name="link">
        <sensor type="camera" name="overview">
          <always_on>true</always_on>
          <update_rate>4</update_rate>
          <camera>
            <horizontal_fov>1.3</horizontal_fov>
            <image><width>640</width><height>480</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>16</far></clip>
          </camera>
          <plugin name="overview_plugin" filename="libgazebo_ros_camera.so">
            <ros>
              <remapping>~/image_raw:=/overview/image_raw</remapping>
              <remapping>~/camera_info:=/overview/camera_info</remapping>
            </ros>
            <frame_name>overview_cam</frame_name>
          </plugin>
        </sensor>
        <inertial>
          <mass>0.01</mass>
          <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
        </inertial>
      </link>
    </model>
    <plugin name="qda_grasp" filename="libqda_grasp_plugin.so">
      <pelvis_z>{z}</pelvis_z>
    </plugin>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/</namespace></ros>
      <update_rate>5.0</update_rate>
    </plugin>
  </world>
</sdf>
"""
