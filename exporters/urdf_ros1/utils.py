# -*- coding: utf-8 -*-
"""
Utility functions for URDF ROS1 exporter
Based on fusion2urdf by syuntoku14
"""

import adsk
import adsk.core
import adsk.fusion
import os.path
import re
import math
import unicodedata
from xml.etree import ElementTree
from xml.dom import minidom
import shutil
import fileinput
import sys


def normalize_name(name):
    """
    Normalize a name for URDF compatibility.
    Removes accents, replaces spaces and special characters with underscores.
    """
    # Normalize unicode to decomposed form (separate base char from accent)
    normalized = unicodedata.normalize('NFKD', name)
    # Remove accents (combining characters)
    ascii_name = ''.join(c for c in normalized if not unicodedata.combining(c))
    # Replace spaces and special characters with underscore
    ascii_name = re.sub(r'[^a-zA-Z0-9_]', '_', ascii_name)
    # Remove multiple consecutive underscores
    ascii_name = re.sub(r'_+', '_', ascii_name)
    # Remove leading/trailing underscores
    ascii_name = ascii_name.strip('_')
    return ascii_name


def copy_occs(root):
    """Duplicate all the components"""

    def copy_body(allOccs, occs):
        """Copy the old occs to new component"""
        bodies = occs.bRepBodies
        transform = adsk.core.Matrix3D.create()

        new_occs = allOccs.addNewComponent(transform)
        if occs.component.name == 'base_link':
            occs.component.name = 'old_component'
            new_occs.component.name = 'base_link'
        else:
            new_occs.component.name = normalize_name(occs.name)
        new_occs = allOccs.item((allOccs.count - 1))
        for i in range(bodies.count):
            body = bodies.item(i)
            body.copyToComponent(new_occs)

    allOccs = root.occurrences
    oldOccs = []
    coppy_list = [occs for occs in allOccs]
    for occs in coppy_list:
        if occs.bRepBodies.count > 0:
            copy_body(allOccs, occs)
            oldOccs.append(occs)

    for occs in oldOccs:
        occs.component.name = 'old_component'


def export_stl(design, save_dir, components):
    """
    Export STL files into save_dir/meshes/

    Parameters
    ----------
    design: adsk.fusion.Design
    save_dir: str - directory path to save
    components: design.allComponents
    """
    exportMgr = design.exportManager
    try:
        os.mkdir(save_dir + '/meshes')
    except:
        pass
    scriptDir = save_dir + '/meshes'

    for component in components:
        allOccus = component.allOccurrences
        for occ in allOccus:
            if 'old_component' not in occ.component.name:
                try:
                    fileName = scriptDir + "/" + occ.component.name
                    stlExportOptions = exportMgr.createSTLExportOptions(occ, fileName)
                    stlExportOptions.sendToPrintUtility = False
                    stlExportOptions.isBinaryFormat = True
                    stlExportOptions.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                    exportMgr.execute(stlExportOptions)
                except:
                    print('Component ' + occ.component.name + ' has something wrong.')


def file_dialog(ui):
    """Display the dialog to save the file"""
    folderDlg = ui.createFolderDialog()
    folderDlg.title = 'Select folder to save URDF'

    dlgResult = folderDlg.showDialog()
    if dlgResult == adsk.core.DialogResults.DialogOK:
        return folderDlg.folder
    return False


def origin2center_of_mass(inertia, center_of_mass, mass):
    """
    Convert moment of inertia about world coordinate to center of mass coordinate

    Parameters
    ----------
    inertia: [xx, yy, zz, xy, yz, xz]
    center_of_mass: [x, y, z]
    mass: float

    Returns
    ----------
    moment of inertia about center of mass: [xx, yy, zz, xy, yz, xz]
    """
    x = center_of_mass[0]
    y = center_of_mass[1]
    z = center_of_mass[2]
    translation_matrix = [
        y**2 + z**2,
        x**2 + z**2,
        x**2 + y**2,
        -x * y,
        -y * z,
        -x * z
    ]
    return [round(i - mass * t, 6) for i, t in zip(inertia, translation_matrix)]


def prettify(elem):
    """Return a pretty-printed XML string for the Element"""
    rough_string = ElementTree.tostring(elem, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def copy_package(save_dir, package_dir):
    """Copy package template files"""
    try:
        if not os.path.exists(save_dir + '/launch'):
            os.mkdir(save_dir + '/launch')
        if not os.path.exists(save_dir + '/urdf'):
            os.mkdir(save_dir + '/urdf')

        if os.path.exists(package_dir):
            shutil.copytree(package_dir, save_dir, dirs_exist_ok=True)
    except Exception as e:
        print(f"Error copying package: {e}")


def update_cmakelists(save_dir, package_name):
    """Update CMakeLists.txt with package name"""
    file_name = save_dir + '/CMakeLists.txt'
    if not os.path.exists(file_name):
        return

    for line in fileinput.input(file_name, inplace=True):
        if 'project(fusion2urdf)' in line:
            sys.stdout.write("project(" + package_name + ")\n")
        else:
            sys.stdout.write(line)


def update_package_xml(save_dir, package_name):
    """Update package.xml with package name"""
    file_name = save_dir + '/package.xml'
    if not os.path.exists(file_name):
        return

    for line in fileinput.input(file_name, inplace=True):
        if '<name>' in line:
            sys.stdout.write("  <name>" + package_name + "</name>\n")
        elif '<description>' in line:
            sys.stdout.write("<description>The " + package_name + " package</description>\n")
        else:
            sys.stdout.write(line)


def matrix3d_to_rpy(matrix3d):
    """
    Convert a Fusion 360 Matrix3D to roll, pitch, yaw angles.
    Returns [roll, pitch, yaw] in radians.
    """
    R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for row in range(3):
        for col in range(3):
            R[row][col] = matrix3d.getCell(row, col)

    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    singular = sy < 1e-6

    if not singular:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0

    return [round(roll, 6), round(pitch, 6), round(yaw, 6)]


def get_occurrence_transform(occurrence):
    """
    Get the xyz position and rpy rotation from an occurrence's transform.
    Returns (xyz, rpy) where both are lists of 3 floats.
    xyz is in meters, rpy is in radians.
    """
    transform = occurrence.transform

    xyz = [round(v / 100.0, 6) for v in transform.translation.asArray()]
    rpy = matrix3d_to_rpy(transform)

    return xyz, rpy


def _rotation_matrix(transform):
    """Extract the 3x3 rotation matrix (row-major) from a Fusion Matrix3D."""
    R = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for row in range(3):
        for col in range(3):
            R[row][col] = transform.getCell(row, col)
    return R


def _rpy_from_matrix(R):
    """Extract URDF roll/pitch/yaw (radians) from a 3x3 rotation matrix R."""
    sy = math.sqrt(R[0][0] ** 2 + R[1][0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0
    return [round(roll, 6), round(pitch, 6), round(yaw, 6)]


def get_relative_rpy(parent_transform, child_transform):
    """
    Calculate the relative rotation (rpy) from parent frame to child frame.
    Returns [roll, pitch, yaw] in radians.
    """
    R_parent = _rotation_matrix(parent_transform)
    R_child = _rotation_matrix(child_transform)

    # R_relative = R_parent^T * R_child
    R_rel = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for i in range(3):
        for j in range(3):
            R_rel[i][j] = sum(R_parent[k][i] * R_child[k][j] for k in range(3))

    return _rpy_from_matrix(R_rel)


def get_relative_transform(parent_transform, child_transform):
    """
    Full relative pose of the child occurrence frame expressed in the PARENT
    occurrence frame: T_rel = M_parent^-1 * M_child.

    This is exactly what a URDF <joint><origin> needs: the child link frame
    located relative to the parent link frame.

    Returns (xyz, rpy):
      xyz : [x, y, z] translation in METERS, expressed in the parent frame
      rpy : [roll, pitch, yaw] in radians
    """
    R_parent = _rotation_matrix(parent_transform)
    t_parent = parent_transform.translation.asArray()  # cm
    t_child = child_transform.translation.asArray()     # cm

    # t_rel = R_parent^T * (t_child - t_parent)   (still in cm)
    dt = [t_child[i] - t_parent[i] for i in range(3)]
    t_rel = [sum(R_parent[k][i] * dt[k] for k in range(3)) for i in range(3)]
    xyz = [round(t_rel[i] / 100.0, 6) for i in range(3)]

    rpy = get_relative_rpy(parent_transform, child_transform)
    return xyz, rpy


def world_axis_to_child(child_transform, axis_world):
    """
    Express a world-frame axis vector in the child occurrence frame.
    In URDF the joint <axis> is given in the child (joint) frame, while Fusion
    reports rotation/slide axes in world coordinates.
      axis_child = R_child^T * axis_world
    """
    R_child = _rotation_matrix(child_transform)
    ac = [sum(R_child[k][i] * axis_world[k] for k in range(3)) for i in range(3)]
    return [round(v, 6) for v in ac]


def world_point_to_link(occ_transform, world_point_cm):
    """
    Express a world-frame point (in cm) in the occurrence/link local frame.
    Returns [x, y, z] in METERS.
      p_link = R^T * (p_world - t)
    """
    R = _rotation_matrix(occ_transform)
    t = occ_transform.translation.asArray()  # cm
    d = [world_point_cm[i] - t[i] for i in range(3)]
    p_link = [sum(R[k][i] * d[k] for k in range(3)) for i in range(3)]
    return [round(v / 100.0, 6) for v in p_link]
