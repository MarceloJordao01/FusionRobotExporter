# -*- coding: utf-8 -*-
"""
Write functions for URDF ROS1 exporter
Based on fusion2urdf by syuntoku14
"""

import adsk
import os
from xml.etree.ElementTree import Element, SubElement
from . import link as Link
from . import joint as Joint
from . import utils
from ...core import sensors as core_sensors


def write_link_urdf(joints_dict, repo, links_xyz_dict, links_rpy_dict, file_name, inertial_dict):
    """Write links information into URDF file"""
    with open(file_name, mode='a') as f:
        # Base link
        center_of_mass = inertial_dict['base_link']['center_of_mass']
        link = Link.Link(
            name='base_link',
            xyz=[0, 0, 0],
            center_of_mass=center_of_mass,
            repo=repo,
            mass=inertial_dict['base_link']['mass'],
            inertia_tensor=inertial_dict['base_link']['inertia'],
            rpy=[0, 0, 0]
        )
        links_xyz_dict[link.name] = link.xyz
        links_rpy_dict[link.name] = link.rpy
        link.make_link_xml()
        f.write(link.link_xml)
        f.write('\n')

        # Other links
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            center_of_mass = [
                i - j for i, j in zip(
                    inertial_dict[name]['center_of_mass'],
                    joints_dict[joint]['xyz']
                )
            ]
            # Get rpy from inertial_dict (extracted from occurrence transform)
            rpy = inertial_dict[name].get('rpy', [0, 0, 0])
            link = Link.Link(
                name=name,
                xyz=joints_dict[joint]['xyz'],
                center_of_mass=center_of_mass,
                repo=repo,
                mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia'],
                rpy=rpy
            )
            links_xyz_dict[link.name] = link.xyz
            links_rpy_dict[link.name] = link.rpy
            link.make_link_xml()
            f.write(link.link_xml)
            f.write('\n')


def write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name):
    """Write joints information into URDF file"""
    with open(file_name, mode='a') as f:
        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']

            try:
                xyz = [
                    round(p - c, 6)
                    for p, c in zip(links_xyz_dict[parent], links_xyz_dict[child])
                ]
            except KeyError:
                app = adsk.core.Application.get()
                ui = app.userInterface
                ui.messageBox(
                    f"Error with connection between {parent} and {child}.\n"
                    f"Check if parent=component2={parent} and child=component1={child} are correct.",
                    "Error!"
                )
                return

            joint = Joint.Joint(
                name=j,
                joint_type=joint_type,
                xyz=xyz,
                axis=joints_dict[j]['axis'],
                parent=parent,
                child=child,
                upper_limit=upper_limit,
                lower_limit=lower_limit
            )
            joint.make_joint_xml()
            f.write(joint.joint_xml)
            f.write('\n')


def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir, sensors=None):
    """Write main URDF/XACRO file"""
    sensors = sensors or []
    links_rpy_dict = {}

    try:
        os.mkdir(save_dir + '/urdf')
    except:
        pass

    file_name = save_dir + '/urdf/' + robot_name + '.xacro'
    repo = package_name + '/meshes/'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(f'<robot name="{robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro">\n')
        f.write('\n')
        f.write(f'<xacro:include filename="$(find {package_name})/urdf/materials.xacro" />\n')
        f.write(f'<xacro:include filename="$(find {package_name})/urdf/{robot_name}.trans" />\n')
        f.write(f'<xacro:include filename="$(find {package_name})/urdf/{robot_name}.gazebo" />\n')
        f.write('\n')

    write_link_urdf(joints_dict, repo, links_xyz_dict, links_rpy_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)

    with open(file_name, mode='a') as f:
        # Write sensor links and joints
        if sensors:
            f.write(core_sensors.generate_sensors_urdf(sensors))

        f.write('</robot>\n')


def write_materials_xacro(package_name, robot_name, save_dir):
    """Write materials XACRO file"""
    try:
        os.mkdir(save_dir + '/urdf')
    except:
        pass

    file_name = save_dir + '/urdf/materials.xacro'
    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(f'<robot name="{robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n')
        f.write('\n')
        f.write('<material name="silver">\n')
        f.write('  <color rgba="0.700 0.700 0.700 1.000"/>\n')
        f.write('</material>\n')
        f.write('\n')
        f.write('</robot>\n')


def write_transmissions_xacro(joints_dict, links_xyz_dict, package_name, robot_name, save_dir):
    """Write transmissions XACRO file"""
    file_name = save_dir + '/urdf/' + robot_name + '.trans'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(f'<robot name="{robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n')
        f.write('\n')

        for j in joints_dict:
            parent = joints_dict[j]['parent']
            child = joints_dict[j]['child']
            joint_type = joints_dict[j]['type']
            upper_limit = joints_dict[j]['upper_limit']
            lower_limit = joints_dict[j]['lower_limit']

            try:
                xyz = [
                    round(p - c, 6)
                    for p, c in zip(links_xyz_dict[parent], links_xyz_dict[child])
                ]
            except KeyError:
                continue

            joint = Joint.Joint(
                name=j,
                joint_type=joint_type,
                xyz=xyz,
                axis=joints_dict[j]['axis'],
                parent=parent,
                child=child,
                upper_limit=upper_limit,
                lower_limit=lower_limit
            )

            if joint_type != 'fixed':
                joint.make_transmission_xml()
                f.write(joint.tran_xml)
                f.write('\n')

        f.write('</robot>\n')


def write_gazebo_xacro(joints_dict, package_name, robot_name, save_dir, sensors=None):
    """Write Gazebo XACRO file"""
    sensors = sensors or []

    try:
        os.mkdir(save_dir + '/urdf')
    except:
        pass

    file_name = save_dir + '/urdf/' + robot_name + '.gazebo'

    with open(file_name, mode='w') as f:
        f.write('<?xml version="1.0" ?>\n')
        f.write(f'<robot name="{robot_name}" xmlns:xacro="http://www.ros.org/wiki/xacro" >\n')
        f.write('\n')
        f.write('<xacro:property name="body_color" value="Gazebo/Silver" />\n')
        f.write('\n')

        # Gazebo plugin
        gazebo = Element('gazebo')
        plugin = SubElement(gazebo, 'plugin')
        plugin.attrib = {'name': 'control', 'filename': 'libgazebo_ros_control.so'}
        gazebo_xml = "\n".join(utils.prettify(gazebo).split("\n")[1:])
        f.write(gazebo_xml)

        # Base link
        f.write('<gazebo reference="base_link">\n')
        f.write('  <material>${body_color}</material>\n')
        f.write('  <mu1>0.2</mu1>\n')
        f.write('  <mu2>0.2</mu2>\n')
        f.write('  <selfCollide>true</selfCollide>\n')
        f.write('  <gravity>true</gravity>\n')
        f.write('</gazebo>\n')
        f.write('\n')

        # Other links
        for joint in joints_dict:
            name = joints_dict[joint]['child']
            f.write(f'<gazebo reference="{name}">\n')
            f.write('  <material>${body_color}</material>\n')
            f.write('  <mu1>0.2</mu1>\n')
            f.write('  <mu2>0.2</mu2>\n')
            f.write('  <selfCollide>true</selfCollide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        # Sensor plugins
        if sensors:
            f.write(core_sensors.generate_sensors_gazebo_urdf(sensors))

        f.write('</robot>\n')


def write_display_launch(package_name, robot_name, save_dir):
    """Write display launch file for RViz"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    launch = Element('launch')

    arg1 = SubElement(launch, 'arg')
    arg1.attrib = {'name': 'model', 'default': f'$(find {package_name})/urdf/{robot_name}.xacro'}

    arg2 = SubElement(launch, 'arg')
    arg2.attrib = {'name': 'gui', 'default': 'true'}

    arg3 = SubElement(launch, 'arg')
    arg3.attrib = {'name': 'rvizconfig', 'default': f'$(find {package_name})/launch/urdf.rviz'}

    param1 = SubElement(launch, 'param')
    param1.attrib = {'name': 'robot_description', 'command': '$(find xacro)/xacro $(arg model)'}

    param2 = SubElement(launch, 'param')
    param2.attrib = {'name': 'use_gui', 'value': '$(arg gui)'}

    node1 = SubElement(launch, 'node')
    node1.attrib = {
        'name': 'joint_state_publisher_gui',
        'pkg': 'joint_state_publisher_gui',
        'type': 'joint_state_publisher_gui'
    }

    node2 = SubElement(launch, 'node')
    node2.attrib = {
        'name': 'robot_state_publisher',
        'pkg': 'robot_state_publisher',
        'type': 'robot_state_publisher'
    }

    node3 = SubElement(launch, 'node')
    node3.attrib = {
        'name': 'rviz',
        'pkg': 'rviz',
        'args': '-d $(arg rvizconfig)',
        'type': 'rviz',
        'required': 'true'
    }

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])
    file_name = save_dir + '/launch/display.launch'

    with open(file_name, mode='w') as f:
        f.write(launch_xml)


def write_gazebo_launch(package_name, robot_name, save_dir):
    """Write Gazebo launch file"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    launch = Element('launch')

    param = SubElement(launch, 'param')
    param.attrib = {
        'name': 'robot_description',
        'command': f'$(find xacro)/xacro $(find {package_name})/urdf/{robot_name}.xacro'
    }

    node = SubElement(launch, 'node')
    node.attrib = {
        'name': 'spawn_urdf',
        'pkg': 'gazebo_ros',
        'type': 'spawn_model',
        'args': f'-param robot_description -urdf -model {robot_name}'
    }

    include_ = SubElement(launch, 'include')
    include_.attrib = {'file': '$(find gazebo_ros)/launch/empty_world.launch'}

    args_pairs = [
        ['paused', 'true'],
        ['use_sim_time', 'true'],
        ['gui', 'true'],
        ['headless', 'false'],
        ['debug', 'false']
    ]

    for name, value in args_pairs:
        arg = SubElement(include_, 'arg')
        arg.attrib = {'name': name, 'value': value}

    launch_xml = "\n".join(utils.prettify(launch).split("\n")[1:])
    file_name = save_dir + '/launch/gazebo.launch'

    with open(file_name, mode='w') as f:
        f.write(launch_xml)


def write_controller_launch(package_name, robot_name, save_dir, joints_dict):
    """Write controller launch file"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    controller_args_str = ""
    for j in joints_dict:
        joint_type = joints_dict[j]['type']
        if joint_type != 'fixed':
            controller_args_str += j + '_position_controller '
    controller_args_str += 'joint_state_controller '

    node_controller = Element('node')
    node_controller.attrib = {
        'name': 'controller_spawner',
        'pkg': 'controller_manager',
        'type': 'spawner',
        'respawn': 'false',
        'output': 'screen',
        'ns': robot_name,
        'args': controller_args_str
    }

    node_publisher = Element('node')
    node_publisher.attrib = {
        'name': 'robot_state_publisher',
        'pkg': 'robot_state_publisher',
        'type': 'robot_state_publisher',
        'respawn': 'false',
        'output': 'screen'
    }
    remap = SubElement(node_publisher, 'remap')
    remap.attrib = {'from': '/joint_states', 'to': '/' + robot_name + '/joint_states'}

    launch_xml = "\n".join(utils.prettify(node_controller).split("\n")[1:])
    launch_xml += "\n".join(utils.prettify(node_publisher).split("\n")[1:])

    file_name = save_dir + '/launch/controller.launch'

    with open(file_name, mode='w') as f:
        f.write('<launch>\n')
        f.write('\n')
        f.write(f'<rosparam file="$(find {package_name})/launch/controller.yaml" command="load"/>\n')
        f.write('\n')
        f.write(launch_xml)
        f.write('\n')
        f.write('</launch>')


def write_controller_yaml(robot_name, save_dir, joints_dict):
    """Write controller YAML file"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    controller_name = robot_name + '_controller'
    file_name = save_dir + '/launch/controller.yaml'

    with open(file_name, 'w') as f:
        f.write(controller_name + ':\n')
        f.write('  # Publish all joint states\n')
        f.write('  joint_state_controller:\n')
        f.write('    type: joint_state_controller/JointStateController\n')
        f.write('    publish_rate: 50\n\n')
        f.write('  # Position Controllers\n')

        for joint in joints_dict:
            joint_type = joints_dict[joint]['type']
            if joint_type != 'fixed':
                f.write(f'  {joint}_position_controller:\n')
                f.write('    type: effort_controllers/JointPositionController\n')
                f.write(f'    joint: {joint}\n')
                f.write('    pid: {p: 100.0, i: 0.01, d: 10.0}\n')


def write_display_rviz(package_name, robot_name, save_dir):
    """Write RViz configuration file for ROS1"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    file_name = os.path.join(save_dir, 'launch', 'urdf.rviz')

    content = '''Panels:
  - Class: rviz/Displays
    Name: Displays
  - Class: rviz/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.03
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 10
      Reference Frame: <Fixed Frame>
      Value: true
    - Alpha: 1
      Class: rviz/RobotModel
      Collision Enabled: false
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Name: RobotModel
      Robot Description: robot_description
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.3
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Update Interval: 0
      Value: true
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz/Interact
      Hide Inactive Objects: true
    - Class: rviz/MoveCamera
    - Class: rviz/Select
    - Class: rviz/FocusCamera
    - Class: rviz/Measure
      Line color: 128; 128; 0
  Value: true
  Views:
    Current:
      Class: rviz/Orbit
      Distance: 2
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.06
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  Width: 1200
  X: 65
  Y: 24
'''

    with open(file_name, mode='w') as f:
        f.write(content)
