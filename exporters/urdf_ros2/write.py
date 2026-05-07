# -*- coding: utf-8 -*-
"""
Write functions for URDF ROS2 exporter
Based on fusion2urdf-ros2 by dheena2k2
"""

import adsk
import os
from xml.etree.ElementTree import Element, SubElement
from . import link as Link
from . import joint as Joint
from . import utils
from . import launch_templates
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
                    f"Error with connection between {parent} and {child}.",
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
        f.write('  <self_collide>true</self_collide>\n')
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
            f.write('  <self_collide>true</self_collide>\n')
            f.write('</gazebo>\n')
            f.write('\n')

        # Sensor plugins
        if sensors:
            f.write(core_sensors.generate_sensors_gazebo_urdf(sensors))

        f.write('</robot>\n')


def write_display_launch(package_name, robot_name, save_dir):
    """Write display launch file for RViz2"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    file_text = launch_templates.get_display_launch_text(package_name, robot_name)
    file_name = os.path.join(save_dir, 'launch', 'display.launch.py')

    with open(file_name, mode='w') as f:
        f.write(file_text)


def write_gazebo_launch(package_name, robot_name, save_dir):
    """Write Gazebo launch file"""
    try:
        os.mkdir(save_dir + '/launch')
    except:
        pass

    file_text = launch_templates.get_gazebo_launch_text(package_name, robot_name)
    file_name = os.path.join(save_dir, 'launch', 'gazebo.launch.py')

    with open(file_name, mode='w') as f:
        f.write(file_text)


def write_setup_py(package_name, save_dir):
    """Write setup.py for ament_python package"""
    file_name = os.path.join(save_dir, 'setup.py')

    content = f'''from setuptools import setup
import os
from glob import glob

package_name = '{package_name}'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='Robot description package generated by FusionRobotExporter',
    license='MIT',
    tests_require=['pytest'],
)
'''

    with open(file_name, mode='w') as f:
        f.write(content)


def write_package_xml(package_name, save_dir):
    """Write package.xml for ROS2"""
    file_name = os.path.join(save_dir, 'package.xml')

    content = f'''<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.1</version>
  <description>Robot description package generated by FusionRobotExporter</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
'''

    with open(file_name, mode='w') as f:
        f.write(content)


def write_display_rviz(package_name, robot_name, save_dir):
    """Write RViz2 configuration file"""
    try:
        os.mkdir(save_dir + '/config')
    except:
        pass

    file_name = os.path.join(save_dir, 'config', 'display.rviz')

    content = f'''Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
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
      Class: rviz_default_plugins/RobotModel
      Collision Enabled: false
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Links:
        All Links Enabled: true
        Expand Joint Details: false
        Expand Link Details: false
        Expand Tree: false
        Link Tree Style: Links in Alphabetic Order
      Mass Properties:
        Inertia: false
        Mass: false
      Name: RobotModel
      TF Prefix: ""
      Update Interval: 0
      Value: true
      Visual Enabled: true
    - Class: rviz_default_plugins/TF
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
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
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
      Value: Orbit (rviz_default_plugins)
      Yaw: 0.785
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd00000004000000000000016a00000300fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d00000300000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f00000300fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d00000300000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d006501000000000000045000000000000000000000021b0000030000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Views:
    collapsed: false
  Width: 1200
  X: 65
  Y: 24
'''

    with open(file_name, mode='w') as f:
        f.write(content)
