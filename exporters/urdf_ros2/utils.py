# -*- coding: utf-8 -*-
"""
Utility functions for URDF ROS2 exporter
Based on fusion2urdf-ros2 by dheena2k2
"""

import adsk
import adsk.core
import adsk.fusion
import os.path
import re
from xml.etree import ElementTree
from xml.dom import minidom
import shutil


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
            new_occs.component.name = re.sub('[ :()]', '_', occs.name)
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
    """Export STL files into save_dir/meshes/"""
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
    """Convert moment of inertia from world to center of mass coordinate"""
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
