# -*- coding: utf-8 -*-
"""
Main exporter for URDF ROS1
Based on fusion2urdf by syuntoku14
"""

import adsk
import adsk.core
import adsk.fusion
import os

from . import joint as Joint
from . import link as Link
from . import write
from ...core import mesh as core_mesh


def export(design, save_dir, options=None):
    """
    Export Fusion 360 design to URDF format for ROS1

    Parameters
    ----------
    design: adsk.fusion.Design - The active design
    save_dir: str - Directory to save the exported files
    options: dict - Export options (optional)

    Returns
    ----------
    tuple: (success: bool, message: str)
    """
    options = options or {}

    try:
        root = design.rootComponent
        components = design.allComponents

        # Get base_link from options
        base_link_name = options.get('base_link')

        # Set names
        robot_name = root.name.split()[0]
        package_name = robot_name + '_description'
        save_dir = os.path.join(save_dir, package_name)

        try:
            os.mkdir(save_dir)
        except:
            pass

        # Generate joints dictionary
        success_msg = 'Successfully created URDF file'
        msg = success_msg

        joints_dict, msg = Joint.make_joints_dict(root, msg, base_link_name)
        if msg != success_msg:
            return False, msg

        # Generate inertial dictionary
        inertial_dict, msg = Link.make_inertial_dict(root, msg, base_link_name)
        if msg != success_msg:
            return False, msg

        if 'base_link' not in inertial_dict:
            return False, 'No base_link found. Please select a base link component.'

        links_xyz_dict = {}

        # Generate URDF files
        write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        write.write_materials_xacro(package_name, robot_name, save_dir)
        write.write_transmissions_xacro(joints_dict, links_xyz_dict, package_name, robot_name, save_dir)
        write.write_gazebo_xacro(joints_dict, package_name, robot_name, save_dir)

        # Generate launch files
        if options.get('export_launch', True):
            write.write_display_launch(package_name, robot_name, save_dir)
            write.write_gazebo_launch(package_name, robot_name, save_dir)
            write.write_controller_launch(package_name, robot_name, save_dir, joints_dict)
            write.write_controller_yaml(robot_name, save_dir, joints_dict)

        # Export meshes
        if options.get('export_meshes', True):
            exported, errors = core_mesh.export_stl(design, save_dir, base_link_name=base_link_name)
            if errors:
                for err in errors:
                    print(f"Mesh warning: {err}")

        return True, f'Successfully exported to {save_dir}'

    except Exception as e:
        return False, f'Export failed: {str(e)}'
