# -*- coding: utf-8 -*-
"""
Shared Rigid Group helpers for FusionRobotExporter.

In the "rigid_groups" link mode each Fusion Rigid Group becomes a single
merged link (all bodies of all member occurrences fused together). Components
that are not in any rigid group are ignored. The selected base group becomes
'base_link'.

Link frame convention (kept consistent with the component-mode invariant
"mesh is aligned with the link frame, visual origin = 0"):
  - the base_link group frame is the world origin (identity)
  - every other group-link frame is WORLD-ALIGNED and translated to the point
    of the joint that connects it to its parent (P)

The merged STL is exported in link-local coordinates by placing the temporary
occurrence at translation P before copying the bodies, so the visual origin
stays 0 and the joint <axis> stays in world coordinates (no child-frame
rotation needed). See exporters/urdf_*/joint.py:make_rigid_group_joints_dict.
"""

import adsk.core
import adsk.fusion
import os

from .mesh import normalize_name


def collect_rigid_groups(root):
    """
    Collect the rigid groups defined at the TOP LEVEL of the assembly (the root
    component only). Rigid groups defined inside sub-assemblies are intentionally
    ignored: only the assembly-level rigid groups define links.

    Returns a list of (rigid_group, group_name) tuples, de-duplicated by the
    normalized group name.
    """
    groups = []
    seen = set()

    for rg in root.rigidGroups:
        name = normalize_name(rg.name)
        if name in seen:
            continue
        seen.add(name)
        groups.append((rg, name))

    return groups


def list_rigid_group_names(root):
    """Return the normalized names of every rigid group (for the UI dropdown)."""
    return [name for _, name in collect_rigid_groups(root)]


def resolve_group_links(root, base_group_name=None):
    """
    Map each rigid group to its final link name.

    The group whose normalized name matches base_group_name becomes 'base_link';
    every other group keeps its normalized name.

    Returns a list of dicts: {'group', 'link_name', 'group_name'}.
    """
    base_norm = normalize_name(base_group_name) if base_group_name else None
    result = []
    for rg, name in collect_rigid_groups(root):
        link_name = 'base_link' if (base_norm and name == base_norm) else name
        result.append({'group': rg, 'link_name': link_name, 'group_name': name})
    return result


def occurrence_to_group_map(group_links):
    """
    Build {occurrence.name: link_name} for every occurrence that belongs to a
    rigid group. Occurrences absent from this map are "loose" and ignored.
    """
    mapping = {}
    for gl in group_links:
        for occ in gl['group'].occurrences:
            mapping[occ.name] = gl['link_name']
    return mapping


def _collect_proxy_bodies(occ):
    """
    Collect the proxy bRepBodies of `occ` AND every nested child occurrence,
    recursively. Proxy bodies already carry the full assembly (world) transform,
    so a sub-assembly member of a rigid group (a component whose geometry lives
    in nested sub-components rather than directly on it) contributes all of its
    descendant bodies too.
    """
    bodies = []
    for body in occ.bRepBodies:
        bodies.append(body)
    for child in occ.childOccurrences:
        bodies.extend(_collect_proxy_bodies(child))
    return bodies


def count_occurrence_bodies(occ):
    """
    Number of bodies an occurrence contributes to a merged rigid-group link:
    its proxy bodies (including nested children, which `merge_occurrence_bodies`
    copies recursively) or, for inaccessible linked components, the nativeObject
    bodies. Mirrors the body loops in `merge_occurrence_bodies`.
    """
    n = len(_collect_proxy_bodies(occ))
    if n == 0:
        try:
            if getattr(occ, 'isReferencedComponent', False) and getattr(occ, 'nativeObject', None):
                n = occ.nativeObject.bRepBodies.count
        except Exception:
            n = 0
    return n


def count_group_bodies(group_links):
    """Total bodies that will be merged across all resolved group links."""
    total = 0
    for gl in group_links:
        for occ in gl['group'].occurrences:
            total += count_occurrence_bodies(occ)
    return total


def count_rigid_groups_bodies(root):
    """Total bodies across every top-level rigid group (used to size the SDF
    progress bar in rigid-group mode)."""
    total = 0
    try:
        for rg in root.rigidGroups:
            for occ in rg.occurrences:
                total += count_occurrence_bodies(occ)
    except Exception:
        pass
    return total


def merge_occurrence_bodies(root, target_occ, occurrences, errors=None, progress=None, step_bodies=False, phase=None):
    """
    Copy every body of `occurrences` (including nested sub-occurrences) into
    `target_occ`, handling linked/external (referenced) components.

    Why not `BRepBody.copyToComponent`: it raises `InternalValidationError` on
    the 2nd+ body of an external/referenced component, so merging a rigid group
    made of linked parts silently dropped all but one body (a rotor link came out
    with a single body; base_link kept ~570 of 5210). Verified empirically on the
    Hermit_v2 design (2026-06-06).

    Instead we copy each body with `TemporaryBRepManager.copy` and add it to the
    target component through a BaseFeature (parametric) or directly (direct
    design). tbm.copy keeps the body's WORLD geometry, and `bRepBodies.add` stores
    raw geometry as component-LOCAL, so we apply the inverse of the target
    occurrence transform to reproduce copyToComponent's old behaviour (the body
    ends up at world - P when `target_occ` sits at +P). tbm.copy also drops the
    body material, which would corrupt mass/inertia, so each merged body's
    `material` is restored from its source afterwards.

    - Normal (or fully loaded linked) occurrences expose proxy `bRepBodies` that
      already carry the assembly (world) transform -> copied directly. Bodies of
      nested child occurrences are gathered recursively, so a sub-assembly added
      to a rigid group contributes all of its parts (e.g. a propeller made of
      several bodies/components), not just the bodies sitting directly on it.
    - Linked components whose proxy bodies are not accessible expose their bodies
      only through `nativeObject` (in component-LOCAL coords); those are mapped to
      world with the occurrence's `transform2` before the world->local step.

    Note: only top-level (root) rigid groups are used, so `occurrence.transform2`
    is the occurrence's world transform.
    """
    if errors is None:
        errors = []

    def advance(label):
        # Count one body (step) or just keep the UI alive (tick), depending on
        # whether this merge is the phase that owns the progress budget.
        if progress is None:
            return
        if step_bodies:
            progress.step(label)
        else:
            progress.tick(label)

    design = root.parentDesign
    is_parametric = (design.designType == adsk.fusion.DesignTypes.ParametricDesignType)
    tbm = adsk.fusion.TemporaryBRepManager.get()
    comp = target_occ.component

    # World -> target-local transform (inverse of the occurrence pose) so the
    # added geometry lands where copyToComponent used to put it.
    to_local = target_occ.transform.copy()
    to_local.invert()

    # In parametric designs added bodies must live inside a BaseFeature edit.
    base_feat = None
    if is_parametric:
        base_feat = comp.features.baseFeatures.add()
        base_feat.startEdit()

    # Source body for each added body, in add order, so the materials lost by
    # tbm.copy can be restored once the BaseFeature edit is closed.
    added_sources = []

    def add_body(src_body, to_world, label):
        # src_body geometry is WORLD already unless `to_world` maps it there
        # (nativeObject bodies are in component-local coords).
        try:
            tcopy = tbm.copy(src_body)
            if to_world is not None:
                tbm.transform(tcopy, to_world)
            tbm.transform(tcopy, to_local)
            if is_parametric:
                comp.bRepBodies.add(tcopy, base_feat)
            else:
                comp.bRepBodies.add(tcopy)
            added_sources.append(src_body)
        except Exception as e:
            errors.append(f"copy of body in '{label}' failed: {e}")
        advance(label)

    for occ in occurrences:
        if progress is not None and progress.is_cancelled():
            break

        label = phase or occ.name
        proxy_bodies = _collect_proxy_bodies(occ)
        if proxy_bodies:
            for body in proxy_bodies:
                add_body(body, None, label)
            continue

        if getattr(occ, 'isReferencedComponent', False) and getattr(occ, 'nativeObject', None):
            native = occ.nativeObject.bRepBodies
            if native.count == 0:
                errors.append(f"linked occurrence '{occ.name}' has no bodies")
                continue
            world_tf = occ.transform2
            for body in native:
                add_body(body, world_tf, label)
            continue

        errors.append(f"occurrence '{occ.name}' has no accessible bodies (broken link?)")

    if base_feat is not None:
        try:
            base_feat.finishEdit()
        except Exception as e:
            errors.append(f"finishEdit failed: {e}")

    # Restore per-body materials (tbm.copy dropped them). Bodies appear in
    # comp.bRepBodies in add order, so index alignment with added_sources holds.
    merged = comp.bRepBodies
    for i, src in enumerate(added_sources):
        if i >= merged.count:
            break
        try:
            merged.item(i).material = src.material
        except Exception:
            pass

    return errors


def _origin_to_com_inertia(inertia, center_of_mass, mass):
    """
    Parallel-axis shift of a world-aligned inertia tensor from the world origin
    to the center of mass. Mirrors urdf_*/utils.py:origin2center_of_mass so the
    core module stays independent of a specific exporter.
    inertia: [ixx, iyy, izz, ixy, iyz, ixz] in kg.m^2 about the world origin
    center_of_mass: [x, y, z] in meters, mass in kg.
    """
    x, y, z = center_of_mass
    translation = [
        y ** 2 + z ** 2,
        x ** 2 + z ** 2,
        x ** 2 + y ** 2,
        -x * y,
        -y * z,
        -x * z,
    ]
    return [round(i - mass * t, 6) for i, t in zip(inertia, translation)]


def build_group_link_data(design, save_dir, group_links, link_frames, export_meshes=True, progress=None):
    """
    For each group link in the kinematic tree, export the merged STL (in
    link-local coordinates) and compute its inertial properties.

    Parameters
    ----------
    design : adsk.fusion.Design
    save_dir : str - package directory; STLs go to save_dir/meshes
    group_links : list - output of resolve_group_links()
    link_frames : dict - {link_name: [x, y, z]} frame translation P in METERS.
        Built by make_rigid_group_joints_dict; 'base_link' is [0, 0, 0].
    export_meshes : bool - when False, only the inertial dict is produced.

    Returns
    -------
    tuple: (inertial_dict, errors)
        inertial_dict keyed by link_name, each entry:
        {'name', 'mass', 'center_of_mass', 'inertia', 'xyz', 'rpy'}
        (same shape as urdf_*/link.py:make_inertial_dict output)
    """
    root = design.rootComponent
    export_mgr = design.exportManager
    meshes_dir = os.path.join(save_dir, 'meshes')
    if export_meshes:
        os.makedirs(meshes_dir, exist_ok=True)

    inertial_dict = {}
    errors = []

    for gl in group_links:
        if progress is not None and progress.is_cancelled():
            break

        link_name = gl['link_name']

        # Skip groups that never become part of the kinematic tree (no parent
        # joint and not the base link).
        if link_name not in link_frames:
            errors.append(f"{link_name}: not connected by any joint, skipped")
            continue

        P = link_frames[link_name]              # meters
        P_cm = [v * 100.0 for v in P]           # Fusion internal units

        # Temporary occurrence translated to P so that copied bodies land in
        # link-local coordinates (world - P) for the STL export.
        transform = adsk.core.Matrix3D.create()
        transform.translation = adsk.core.Vector3D.create(P_cm[0], P_cm[1], P_cm[2])
        temp_occ = root.occurrences.addNewComponent(transform)
        temp_occ.component.name = link_name + '_merged'

        # Copy every member body into temp_occ (positioned at P), handling
        # linked/external components. temp_occ at +P means the copied bodies land
        # in link-local coordinates (world - P). One progress step per body keeps
        # the count body-based and the UI responsive on large assemblies.
        if progress is not None:
            progress.set_phase(f'Merging "{link_name}"')
        merge_occurrence_bodies(
            root, temp_occ, gl['group'].occurrences, errors,
            progress=progress, step_bodies=True, phase=f'Merging "{link_name}"')

        if export_meshes:
            if progress is not None:
                progress.set_phase(f'Exporting mesh "{link_name}"')
            file_path = os.path.join(meshes_dir, link_name + '.stl')
            try:
                stl_options = export_mgr.createSTLExportOptions(temp_occ, file_path)
                stl_options.sendToPrintUtility = False
                stl_options.isBinaryFormat = True
                stl_options.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementLow
                if not export_mgr.execute(stl_options):
                    errors.append(f"{link_name}: STL export failed")
            except Exception as e:
                errors.append(f"{link_name}: {str(e)}")

        # Physical properties of the occurrence are reported in the world frame.
        if progress is not None:
            progress.set_phase(f'Computing inertia "{link_name}"')
        prop = temp_occ.getPhysicalProperties(
            adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
        world_com_m = [c / 100.0 for c in prop.centerOfMass.asArray()]
        (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
        moment_world = [v / 10000.0 for v in (xx, yy, zz, xy, yz, xz)]
        inertia_com = _origin_to_com_inertia(moment_world, world_com_m, prop.mass)

        # The link frame is world-aligned and translated by P, so the COM in the
        # link frame is simply the world COM minus P.
        com_link = [round(world_com_m[i] - P[i], 6) for i in range(3)]

        inertial_dict[link_name] = {
            'name': link_name,
            'mass': prop.mass,
            'center_of_mass': com_link,
            'inertia': inertia_com,
            'xyz': [round(v, 6) for v in P],
            'rpy': [0, 0, 0],
        }

        temp_occ.deleteMe()

    return inertial_dict, errors
