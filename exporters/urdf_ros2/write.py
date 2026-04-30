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


def write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict):
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
            inertia_tensor=inertial_dict['base_link']['inertia']
        )
        links_xyz_dict[link.name] = link.xyz
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
            link = Link.Link(
                name=name,
                xyz=joints_dict[joint]['xyz'],
                center_of_mass=center_of_mass,
                repo=repo,
                mass=inertial_dict[name]['mass'],
                inertia_tensor=inertial_dict[name]['inertia']
            )
            links_xyz_dict[link.name] = link.xyz
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


def write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir):
    """Write main URDF/XACRO file"""
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

    write_link_urdf(joints_dict, repo, links_xyz_dict, file_name, inertial_dict)
    write_joint_urdf(joints_dict, repo, links_xyz_dict, file_name)

    with open(file_name, mode='a') as f:
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


def write_gazebo_xacro(joints_dict, package_name, robot_name, save_dir):
    """Write Gazebo XACRO file"""
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
