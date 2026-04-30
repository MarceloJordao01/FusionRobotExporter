# -*- coding: utf-8 -*-
"""
Shared mesh export utilities for FusionRobotExporter
Handles linked components and subassemblies
"""

import adsk.core
import adsk.fusion
import os
import re


def normalize_name(name: str) -> str:
    """Normalize component name for file system"""
    return re.sub(r'[ :()\\/<>"|?*]', '_', name)


def get_all_occurrences(root: adsk.fusion.Component):
    """
    Get all occurrences recursively, including subassemblies

    Returns list of (occurrence, prefix) tuples
    """
    results = []

    def collect_recursive(occurrences, prefix=''):
        for occ in occurrences:
            occ_name = normalize_name(occ.name)
            results.append((occ, prefix))

            if occ.childOccurrences.count > 0:
                child_prefix = prefix + occ_name + '__'
                collect_recursive(occ.childOccurrences, child_prefix)

    collect_recursive(root.occurrences)
    return results


def get_bodies_from_occurrence(occ: adsk.fusion.Occurrence):
    """
    Get bodies from occurrence, handling linked components

    Returns (bodies, source) where source is 'direct' or 'nativeObject'
    """
    bodies = occ.bRepBodies

    if bodies.count > 0:
        return bodies, 'direct'

    if occ.isReferencedComponent and hasattr(occ, 'nativeObject') and occ.nativeObject:
        native_bodies = occ.nativeObject.bRepBodies
        if native_bodies and native_bodies.count > 0:
            return native_bodies, 'nativeObject'

    return None, 'none'


def export_stl(design: adsk.fusion.Design, save_dir: str, occurrences=None, prefix_names=False, base_link_name=None):
    """
    Export STL files for all occurrences

    Parameters
    ----------
    design: adsk.fusion.Design
    save_dir: str - directory to save meshes (will create 'meshes' subfolder)
    occurrences: list - optional list of (occurrence, prefix) tuples. If None, exports all.
    prefix_names: bool - if True, prepend prefix to filename for subassembly components
    base_link_name: str - occurrence name to rename to 'base_link' (optional)

    Returns
    -------
    dict: {component_name: file_path} for successful exports
    list: error messages
    """
    export_mgr = design.exportManager
    meshes_dir = os.path.join(save_dir, 'meshes')
    os.makedirs(meshes_dir, exist_ok=True)

    if occurrences is None:
        occurrences = get_all_occurrences(design.rootComponent)

    exported = {}
    errors = []

    for occ, prefix in occurrences:
        comp_name = normalize_name(occ.component.name)

        if comp_name in exported or 'old_component' in comp_name:
            continue

        bodies, source = get_bodies_from_occurrence(occ)

        if bodies is None or bodies.count == 0:
            if occ.childOccurrences.count == 0:
                errors.append(f"{comp_name}: no bodies found")
            continue

        # Check if this is the base_link
        if base_link_name and occ.name == base_link_name:
            file_name = 'base_link'
        elif prefix_names and prefix:
            file_name = prefix + comp_name
        else:
            file_name = comp_name

        file_path = os.path.join(meshes_dir, file_name + '.stl')

        try:
            export_target = occ if source == 'direct' else occ.nativeObject

            stl_options = export_mgr.createSTLExportOptions(export_target, file_path)
            stl_options.sendToPrintUtility = False
            stl_options.isBinaryFormat = True
            stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow

            if export_mgr.execute(stl_options):
                exported[comp_name] = file_path
            else:
                errors.append(f"{comp_name}: export failed")
        except Exception as e:
            errors.append(f"{comp_name}: {str(e)}")

    return exported, errors


def export_obj(design: adsk.fusion.Design, save_dir: str, occurrences=None, prefix_names=True):
    """
    Export OBJ files for all bodies in occurrences

    Parameters
    ----------
    design: adsk.fusion.Design
    save_dir: str - directory to save meshes (will create 'meshes' subfolder)
    occurrences: list - optional list of (occurrence, prefix) tuples. If None, exports all.
    prefix_names: bool - if True, prepend prefix to filename

    Returns
    -------
    dict: {body_name: file_path} for successful exports
    list: error messages
    """
    export_mgr = design.exportManager
    meshes_dir = os.path.join(save_dir, 'meshes')
    os.makedirs(meshes_dir, exist_ok=True)

    if occurrences is None:
        occurrences = get_all_occurrences(design.rootComponent)

    exported = {}
    errors = []

    for occ, prefix in occurrences:
        comp_name = normalize_name(occ.component.name)

        if 'old_component' in comp_name:
            continue

        bodies, source = get_bodies_from_occurrence(occ)

        if bodies is None or bodies.count == 0:
            continue

        actual_bodies = bodies if source == 'direct' else occ.nativeObject.bRepBodies

        for body in actual_bodies:
            body_name = normalize_name(body.name)

            if prefix_names and prefix:
                file_name = prefix + comp_name + '__' + body_name
            else:
                file_name = comp_name + '__' + body_name

            file_path = os.path.join(meshes_dir, file_name + '.obj')

            if file_name in exported:
                continue

            try:
                export_options = export_mgr.createOBJExportOptions(body, file_path)
                export_options.sendToPrintUtility = False
                export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow

                if export_mgr.execute(export_options):
                    exported[file_name] = file_path
                else:
                    errors.append(f"{file_name}: export failed")
            except Exception as e:
                errors.append(f"{file_name}: {str(e)}")

    return exported, errors


def export_body_to_obj(export_mgr, body, file_path: str) -> tuple:
    """
    Export a single body to OBJ file

    Parameters
    ----------
    export_mgr: adsk.fusion.ExportManager
    body: adsk.fusion.BRepBody - the body to export
    file_path: str - full path for the output file

    Returns
    -------
    tuple: (success: bool, error_message: str or None)
    """
    try:
        export_options = export_mgr.createOBJExportOptions(body, file_path)
        export_options.sendToPrintUtility = False
        export_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow

        if export_mgr.execute(export_options):
            return True, None
        else:
            return False, "export failed"
    except Exception as e:
        return False, str(e)
