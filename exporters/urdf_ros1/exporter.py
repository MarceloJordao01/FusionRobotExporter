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
from ...core import rigid_groups as core_rigid
from ...core import sensors as core_sensors
from ...core.progress import ProgressReporter, count_link_occurrences


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
        ui = adsk.core.Application.get().userInterface
    except Exception:
        ui = None
    progress = ProgressReporter(ui)

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

        success_msg = 'Successfully created URDF file'
        msg = success_msg
        links_xyz_dict = {}
        export_meshes = options.get('export_meshes', True)

        if options.get('link_mode') == 'rigid_groups':
            # Links are defined by Rigid Groups; loose components are ignored.
            group_links = core_rigid.resolve_group_links(root, base_link_name)
            if not group_links:
                return False, 'No rigid groups found. Create rigid groups or use Components mode.'

            occ_to_group = core_rigid.occurrence_to_group_map(group_links)
            joints_dict, link_frames, msg = Joint.make_rigid_group_joints_dict(root, occ_to_group, msg)
            if msg != success_msg:
                return False, msg

            total_bodies = core_rigid.count_group_bodies(group_links)
            progress.start(total_bodies, 'Exporting URDF (ROS1)', 'Merging rigid-group bodies...', unit='bodies')
            inertial_dict, mesh_errors = core_rigid.build_group_link_data(
                design, save_dir, group_links, link_frames, export_meshes=export_meshes, progress=progress)
            for err in mesh_errors:
                print(f"Rigid group warning: {err}")

            if progress.is_cancelled():
                return False, 'Export cancelled by user.'

            if 'base_link' not in inertial_dict:
                return False, 'Selected base link group not found among rigid groups.'
        else:
            # Default mode: one component = one link.
            joints_dict, msg = Joint.make_joints_dict(root, msg, base_link_name)
            if msg != success_msg:
                return False, msg

            n_links = count_link_occurrences(root)
            total = n_links + (n_links if export_meshes else 0)
            progress.start(total, 'Exporting URDF (ROS1)', 'Calculating inertia...')
            progress.set_phase('Calculating inertia')
            inertial_dict, msg = Link.make_inertial_dict(root, msg, base_link_name, progress=progress)
            if msg != success_msg:
                return False, msg

            if progress.is_cancelled():
                return False, 'Export cancelled by user.'

            if 'base_link' not in inertial_dict:
                return False, 'No base_link found. Please select a base link component.'

        # Collect sensors (sensor__* components) for the Gazebo xacro.
        if options.get('link_mode') == 'rigid_groups':
            sensor_link_tfs = core_sensors.build_rigid_link_transforms(link_frames)
        else:
            sensor_link_tfs = core_sensors.build_component_link_transforms(root, base_link_name)
        sensors = core_sensors.collect_sensors(root, sensor_link_tfs)
        for s in sensors:
            if 'error' in s:
                print(f"Sensor warning: {s['error']}")

        # Generate URDF files
        write.write_urdf(joints_dict, links_xyz_dict, inertial_dict, package_name, robot_name, save_dir)
        write.write_materials_xacro(package_name, robot_name, save_dir)
        write.write_transmissions_xacro(joints_dict, links_xyz_dict, package_name, robot_name, save_dir)
        write.write_gazebo_xacro(joints_dict, package_name, robot_name, save_dir, sensors=sensors)

        # Generate launch files and RViz config
        if options.get('export_launch', True):
            write.write_display_launch(package_name, robot_name, save_dir)
            write.write_gazebo_launch(package_name, robot_name, save_dir)
            write.write_controller_launch(package_name, robot_name, save_dir, joints_dict)
            write.write_controller_yaml(robot_name, save_dir, joints_dict)
            write.write_display_rviz(package_name, robot_name, save_dir)

        # Export meshes (component mode only; rigid-group meshes are exported above)
        if export_meshes and options.get('link_mode') != 'rigid_groups':
            progress.set_phase('Exporting meshes')
            exported, errors = core_mesh.export_stl(design, save_dir, base_link_name=base_link_name, progress=progress)
            if errors:
                for err in errors:
                    print(f"Mesh warning: {err}")

        return True, f'Successfully exported to {save_dir}'

    except Exception as e:
        return False, f'Export failed: {str(e)}'
    finally:
        progress.finish()
