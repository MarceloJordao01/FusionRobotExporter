# -*- coding: utf-8 -*-
"""
SDF generation for SDF exporter
Based on FusionSDF by andreasBihlmaier
"""

import os
import shutil
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, List

import adsk.core
import adsk.fusion

from .log import log
from .util import normalize_name, transform2_to_pose, prettify_xml, cm_to_m, kg_cm2_to_kg_m2, world_inertia_to_com_inertia, name_to_path
from .link import Link, LinkElement, LinkElementType, LinkGeometry, LinkGeometryType, LinkInertial
from .joint import Joint, JointType
from .pose import Pose
from ...core.mesh import export_occurrence_to_stl
from ...core.rigid_groups import merge_occurrence_bodies
from ...core import sensors as core_sensors


class SDF:
    def __init__(self, design: adsk.fusion.Design, meshes_cache_dir_path: Path = None,
                 link_mode: str = 'components', base_link_name: str = None, progress=None):
        self.design = design
        self.meshes_cache_dir_path = meshes_cache_dir_path
        # Optional ProgressReporter; one step per link actually created.
        self.progress = progress
        # link_mode: 'components' (one component = one link) or 'rigid_groups'
        # (each rigid group = one merged link; loose components are ignored).
        self.link_mode = link_mode
        self.base_link_name = normalize_name(base_link_name) if base_link_name else None
        self.tmp_dir_path: Path = Path(tempfile.mkdtemp())
        self.tmp_meshes_dir_path: Path = self.tmp_dir_path / 'meshes'
        self.name: str = None
        self.links: Dict[str, Link] = {}
        self.joints: Dict[str, Joint] = {}
        self.fusion_joints: List[List] = []
        self.occurrences_in_rigid_groups: Dict[str, str] = {}
        self.root_link: str = None
        # link_name -> world Matrix3D, used to place sensors relative to a link.
        self.link_world: Dict[str, object] = {}

        self.parse_root_component()

    def parse_root_component(self):
        root_component = self.design.rootComponent
        self.name = normalize_name(root_component.name)
        log(f'parse_root_component: {root_component.name}\n')

        if self.link_mode == 'rigid_groups':
            for rigid_group in root_component.rigidGroups:
                self.add_rigid_group(root_component.occurrences, adsk.core.Matrix3D.create(), rigid_group, '')

        for occurrence in root_component.occurrences:
            self.parse_occurrence(occurrence, '')

        for joint in root_component.joints:
            self.add_joint(joint, False, '')
        for joint in root_component.asBuiltJoints:
            self.add_joint(joint, True, '')
        for fusion_joint, as_built, prefix in self.fusion_joints:
            self.add_joint(fusion_joint, as_built, prefix)

        # Pick the root link: honor the user-selected base group in rigid-group
        # mode, otherwise auto-detect the link that is no joint's child.
        chosen_root = None
        if self.link_mode == 'rigid_groups' and self.base_link_name:
            candidate = self.base_link_name + '_1'
            if candidate in self.links:
                chosen_root = candidate
            else:
                log(f'WARNING: selected base group "{self.base_link_name}" not found, auto-detecting root\n')

        if chosen_root is not None:
            self.root_link = chosen_root
        else:
            links_with_parent_joints = set([joint.child for joint in self.joints.values()])
            root_links = set(self.links.keys()) - links_with_parent_joints

            if len(root_links) == 0:
                log(f'WARNING: No root link found\n')
            else:
                if len(root_links) != 1:
                    log(f'WARNING: Multiple root links found: {root_links}\n')
                self.root_link = next(iter(root_links))

        # Rename the real root link to 'base_link' (like the URDF exporter),
        # instead of adding a separate massless dummy base_link + fixed joint.
        # The robot's actual base component thus carries its real mass/inertia
        # and is the model root, matching the URDF 1:1.
        if self.root_link is not None:
            self._rename_link(self.root_link, 'base_link')
            self.root_link = 'base_link'

        # Attach sensors (sensor__* components) to their target links.
        self.attach_sensors()

    def _rename_link(self, old_name: str, new_name: str):
        """Rename a link everywhere it is referenced: the links map, its
        visual/collision elements (names + mesh uri), the exported mesh file, the
        joint endpoints and the sensor-frame map. Used to turn the detected root
        link into 'base_link' without a dummy link."""
        if old_name == new_name or old_name not in self.links:
            return
        if new_name in self.links:
            log(f'_rename_link: "{new_name}" already exists, keeping "{old_name}" as root\n')
            self.root_link = old_name
            return

        link = self.links.pop(old_name)
        link.name = new_name
        self.links[new_name] = link

        old_uri = 'meshes/' + name_to_path(old_name) + '.stl'
        new_uri = 'meshes/' + name_to_path(new_name) + '.stl'
        old_path = self.tmp_dir_path / old_uri
        new_path = self.tmp_dir_path / new_uri
        try:
            if old_path.exists():
                os.makedirs(new_path.parent, exist_ok=True)
                shutil.move(str(old_path), str(new_path))
        except Exception as e:
            log(f'_rename_link: mesh rename failed ({e})\n')

        # Rebuild the visual/collision dicts with the new element names and uri.
        for elements in (link.visuals, link.collisions):
            suffix = '_visual' if elements is link.visuals else '_collision'
            renamed = {}
            for elem in elements.values():
                if elem.geometry is not None and elem.geometry.mesh_uri == old_uri:
                    elem.geometry.mesh_uri = new_uri
                elem.name = new_name + suffix
                renamed[elem.name] = elem
            elements.clear()
            elements.update(renamed)

        for joint in self.joints.values():
            if joint.parent == old_name:
                joint.parent = new_name
            if joint.child == old_name:
                joint.child = new_name

        if old_name in self.link_world:
            self.link_world[new_name] = self.link_world.pop(old_name)

    def attach_sensors(self):
        """Place sensor__* components on their target links as <sensor> elements."""
        sensors = core_sensors.collect_sensors(self.design.rootComponent, dict(self.link_world))
        for s in sensors:
            if 'error' in s:
                log(f'Sensor warning: {s["error"]}\n')
                continue
            element = core_sensors.make_sdf_sensor_element(s)
            link = self.links.get(s['link'])
            if link is None or element is None:
                log(f'Sensor warning: link "{s.get("link")}" not found for sensor "{s.get("name")}"\n')
                continue
            link.sensors.append(element)
            log(f'Attached sensor "{s["name"]}" ({s["type"]}) to link "{s["link"]}"\n')

    def parse_occurrence(self, occurrence: adsk.fusion.Occurrence, prefix: str):
        self.add_link(occurrence, prefix)

        children_prefix = prefix + normalize_name(occurrence.name) + '__'

        # Only top-level (root component) rigid groups define links; rigid groups
        # nested inside sub-assemblies are intentionally ignored. The root-level
        # groups are handled once in parse_root_component().

        for suboccurrence in occurrence.childOccurrences:
            self.parse_occurrence(suboccurrence, children_prefix)

        for joint in occurrence.component.joints:
            self.fusion_joints.append([joint, False, children_prefix])

        for joint in occurrence.component.asBuiltJoints:
            self.fusion_joints.append([joint, True, children_prefix])

    def add_link(self, occurrence: adsk.fusion.Occurrence, prefix: str, rigid_group_pose: adsk.core.Matrix3D = None,
                 is_rigid_group_link: bool = False):
        link_name = prefix + normalize_name(occurrence.name)

        if self.progress is not None and self.progress.is_cancelled():
            return

        # Sensor mounts (sensor__*) are not links.
        if not is_rigid_group_link and core_sensors.is_sensor_occurrence(occurrence):
            return

        if link_name in self.links:
            return

        if occurrence.name in self.occurrences_in_rigid_groups:
            occurrence.attributes.add('FusionSDF', 'link_name', self.occurrences_in_rigid_groups[occurrence.name])
            return

        # In rigid-group mode, components that are not part of any rigid group
        # are ignored (only the merged rigid-group links are emitted).
        if self.link_mode == 'rigid_groups' and not is_rigid_group_link:
            return
        occurrence.attributes.add('FusionSDF', 'link_name', link_name)

        # Get bodies, handling linked components
        bodies = occurrence.bRepBodies
        bodies_source = occurrence
        if bodies.count == 0 and occurrence.isReferencedComponent:
            if hasattr(occurrence, 'nativeObject') and occurrence.nativeObject:
                native_bodies = occurrence.nativeObject.bRepBodies
                if native_bodies and native_bodies.count > 0:
                    bodies = native_bodies
                    bodies_source = occurrence.nativeObject
                    log(f'Using nativeObject for linked component "{occurrence.name}"\n')

        if bodies.count == 0:
            log(f'Link "{link_name}" has no bodies, skipping\n')
            return

        link = Link(link_name)
        if rigid_group_pose is not None:
            link.pose = rigid_group_pose
        else:
            link.pose = transform2_to_pose(occurrence.transform2)

        # One mesh per link: export the whole occurrence as a single STL in
        # link-local coordinates, the same mechanism the URDF exporter uses
        # (core/mesh.py:export_stl). Visual and collision both reference this one
        # mesh — no per-body OBJ and no oriented minimum bounding box (the OBB
        # crashed with "invalid argument widthDirection" on degenerate bodies).
        # Because the mesh is in component-local coords and link.pose =
        # occurrence.transform2, the visual/collision origin coincides with the
        # link frame (pose = None).
        mesh_uri = 'meshes/' + name_to_path(link.name) + '.stl'
        mesh_path = self.tmp_dir_path / mesh_uri

        visual = LinkElement(LinkElementType.VISUAL, link.name + '_visual')
        visual.geometry = LinkGeometry(LinkGeometryType.MESH)
        visual.geometry.mesh_uri = mesh_uri
        visual.pose = None
        link.visuals[visual.name] = visual

        collision = LinkElement(LinkElementType.COLLISION, link.name + '_collision')
        collision.geometry = LinkGeometry(LinkGeometryType.MESH)
        collision.geometry.mesh_uri = mesh_uri
        collision.pose = None
        link.collisions[collision.name] = collision

        if self.progress is not None:
            # Component mode budgets one step per link mesh; rigid-group links
            # already consumed their (body-based) budget during the merge above,
            # so they only tick to keep the UI alive without double-counting.
            if is_rigid_group_link:
                self.progress.tick(link_name)
            else:
                self.progress.step(link_name)

        os.makedirs(mesh_path.parent, exist_ok=True)

        cached = False
        if self.meshes_cache_dir_path:
            mesh_cache_path = self.meshes_cache_dir_path / mesh_uri.replace('meshes/', '')
            if mesh_cache_path.exists():
                shutil.copy(mesh_cache_path, mesh_path)
                cached = True

        if not cached:
            # Exporting a merged rigid-group occurrence (thousands of bodies) is a
            # single blocking call with no per-body progress: label it so the
            # parked counter clearly reads as "working on this mesh", not frozen.
            if self.progress is not None:
                self.progress.set_phase(f'Exporting mesh "{link_name}"')
            success, error = export_occurrence_to_stl(
                self.design.exportManager, bodies_source, str(mesh_path))
            if not success:
                log(f'ERROR: Failed to export mesh "{mesh_path}": {error}\n')

        # Same for the inertia: VeryHigh accuracy over a merged occurrence is the
        # slowest single step on big assemblies; surface it as a phase so the user
        # sees what the parked counter is doing.
        if self.progress is not None:
            self.progress.set_phase(f'Computing inertia "{link_name}"')
        # Low accuracy: VeryHigh over a merged rigid-group occurrence (thousands
        # of bodies) is the slowest single step and parks the progress counter.
        # Low is far faster and plenty precise for Gazebo simulation. Applies to
        # both link modes (component and rigid_groups share this path). Bump to
        # MediumCalculationAccuracy if a link's mass/inertia ever looks off.
        physical_properties = occurrence.getPhysicalProperties(adsk.fusion.CalculationAccuracy.LowCalculationAccuracy)
        center_of_mass = cm_to_m(physical_properties.centerOfMass.asArray())
        inertia_valid, xx, yy, zz, xy, yz, xz = physical_properties.getXYZMomentsOfInertia()

        if not inertia_valid:
            log(f'ERROR: Failed to get moments of inertia for "{link.name}"\n')
            xx, yy, zz, xy, yz, xz = 1, 1, 1, 0, 0, 0

        xx, yy, zz, xy, yz, xz = [kg_cm2_to_kg_m2(value) for value in [xx, yy, zz, xy, yz, xz]]
        ixx, iyy, izz, ixy, iyz, ixz = world_inertia_to_com_inertia([xx, yy, zz, xy, yz, xz], center_of_mass, physical_properties.mass)

        inertial_pose_model = Pose(center_of_mass, [0, 0, 0])
        if rigid_group_pose is not None:
            inertial_pose_link = link.pose.inverse() * rigid_group_pose * inertial_pose_model
        else:
            inertial_pose_link = link.pose.inverse() * inertial_pose_model
        inertial_pose_link.relative_to = None

        link.inertial = LinkInertial(inertial_pose_link, physical_properties.mass, ixx, ixy, ixz, iyy, iyz, izz)

        self.links[link.name] = link
        # Record the link's world transform for sensor placement (component mode;
        # rigid-group links use the merged temp occurrence and are best-effort).
        self.link_world[link.name] = occurrence.transform
        return link

    def add_joint(self, fusion_joint: adsk.fusion.Joint, as_built: bool, prefix: str):
        joint_name = prefix + normalize_name(fusion_joint.name)
        log(f'add_joint: fusion_joint="{fusion_joint.name}"; as_built={as_built}; prefix="{prefix}" -> joint_name="{joint_name}"\n')

        if joint_name in self.joints:
            log(f'Joint "{joint_name}" already exists, skipping\n')
            return

        if fusion_joint.occurrenceOne is None or fusion_joint.occurrenceTwo is None:
            log(f'ERROR: Ignoring joint "{fusion_joint.name}" because one of the occurrences is missing\n')
            return

        joint = Joint(joint_name)
        fusion_joint_to_sdf_joint_type = {
            adsk.fusion.JointTypes.RigidJointType: JointType.FIXED,
            adsk.fusion.JointTypes.RevoluteJointType: JointType.REVOLUTE,
            adsk.fusion.JointTypes.SliderJointType: JointType.PRISMATIC
        }

        joint_motion = fusion_joint.jointMotion
        if joint_motion.jointType in fusion_joint_to_sdf_joint_type:
            joint.joint_type = fusion_joint_to_sdf_joint_type[joint_motion.jointType]
        else:
            log(f'ERROR: Unsupported joint type "{joint_motion.jointType}" for "{joint_name}, using fixed"\n')
            joint.joint_type = JointType.FIXED

        if joint.joint_type == JointType.REVOLUTE:
            joint.axis_xyz = joint_motion.rotationAxisVector.asArray()
            rotation_limits = joint_motion.rotationLimits
            if rotation_limits.isMinimumValueEnabled or rotation_limits.isMaximumValueEnabled:
                joint.lower_limit = rotation_limits.minimumValue
                joint.upper_limit = rotation_limits.maximumValue
            else:
                joint.joint_type = JointType.CONTINUOUS
        elif joint.joint_type == JointType.PRISMATIC:
            joint.axis_xyz = joint_motion.slideDirectionVector.asArray()
            slide_limits = joint_motion.slideLimits
            joint.lower_limit = cm_to_m(slide_limits.minimumValue)
            joint.upper_limit = cm_to_m(slide_limits.maximumValue)

        if fusion_joint.occurrenceTwo.name in self.occurrences_in_rigid_groups:
            joint.parent = self.occurrences_in_rigid_groups[fusion_joint.occurrenceTwo.name]
        else:
            attr = fusion_joint.occurrenceTwo.attributes.itemByName('FusionSDF', 'link_name')
            joint.parent = attr.value if attr else normalize_name(fusion_joint.occurrenceTwo.name)

        if fusion_joint.occurrenceOne.name in self.occurrences_in_rigid_groups:
            joint.child = self.occurrences_in_rigid_groups[fusion_joint.occurrenceOne.name]
        else:
            attr = fusion_joint.occurrenceOne.attributes.itemByName('FusionSDF', 'link_name')
            joint.child = attr.value if attr else normalize_name(fusion_joint.occurrenceOne.name)

        if self.design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
            swap_parent_child_parameter = self.design.userParameters.itemByName(joint.name + '_SWAP_PARENT_CHILD')
            if swap_parent_child_parameter is not None and swap_parent_child_parameter.value:
                joint.parent, joint.child = joint.child, joint.parent
                if joint.axis_xyz is not None:
                    joint.axis_xyz = [-value for value in joint.axis_xyz]

        if as_built:
            joint_origin = fusion_joint.geometry
            if joint_origin is not None:
                joint.pose = Pose(cm_to_m(joint_origin.origin.asArray()), [0, 0, 0])
        else:
            joint_origin = fusion_joint.geometryOrOriginTwo
            if joint_origin is not None:
                if joint_origin.classType() == 'adsk::fusion::JointGeometry':
                    joint.pose = Pose(cm_to_m(joint_origin.origin.asArray()), [0, 0, 0])
                else:
                    joint.pose = transform2_to_pose(joint_origin.transform)

        # Only emit joints that connect two distinct, real links. Rigid/as-built
        # joints that merely assemble the sub-parts inside one component (e.g.
        # everything inside base_link) collapse to parent == child, and a joint
        # touching a component that never became a link points at a missing link.
        # Both are invalid SDF and make Gazebo reject the whole model. The URDF
        # exporter effectively drops these same internal joints — this keeps the
        # SDF kinematic tree equivalent (only the rotor joints survive on Hermit).
        if joint.parent == joint.child:
            log(f'Skipping joint "{joint_name}": parent == child ("{joint.parent}")\n')
            return
        if joint.parent not in self.links or joint.child not in self.links:
            missing = joint.parent if joint.parent not in self.links else joint.child
            log(f'Skipping joint "{joint_name}": "{missing}" is not a link\n')
            return

        self.joints[joint.name] = joint

    def add_rigid_group(self, parent_occurrences, parent_pose, rigid_group, prefix: str):
        link_name = prefix + normalize_name(rigid_group.name)
        log(f'add_rigid_group: rigid_group="{rigid_group.name}"; prefix="{prefix}" -> link_name={link_name}\n')

        rigid_group_occurrence = parent_occurrences.addNewComponent(adsk.core.Matrix3D.create())
        rigid_group_component = rigid_group_occurrence.component
        rigid_group_component.name = link_name

        # Merge member bodies into the temp occurrence, handling linked/external
        # components (proxy bodies may be empty -> fall back to nativeObject).
        # The merge is the slow phase (one tbm.copy + BaseFeature add per body),
        # so it owns the progress budget (step_bodies=True): the counter advances
        # here instead of sitting at 0 until the merge finishes. add_link then
        # only ticks while exporting these same bodies, so the count stays
        # body-based and isn't doubled.
        if self.progress is not None:
            self.progress.set_phase(f'Merging "{link_name}"')
        merge_errors = merge_occurrence_bodies(
            self.design.rootComponent, rigid_group_occurrence, rigid_group.occurrences,
            progress=self.progress, step_bodies=True, phase=f'Merging "{link_name}"')
        for err in merge_errors:
            log(f'add_rigid_group: {err}\n')

        if self.progress is not None:
            self.progress.set_phase(f'Exporting "{link_name}"')
        rigid_group_link = self.add_link(rigid_group_occurrence, '', rigid_group_pose=transform2_to_pose(parent_pose), is_rigid_group_link=True)
        rigid_group_occurrence.deleteMe()

        for occurrence in rigid_group.occurrences:
            self.occurrences_in_rigid_groups[occurrence.name] = link_name + '_1'

        return rigid_group_link

    def print(self):
        log(f'SDF: {self.name}\n')
        log(f'tmp_dir_path: {self.tmp_dir_path}\n')
        log('Links:\n')
        for link in self.links.values():
            log(f'  {link}\n')
        log('Joints:\n')
        for joint in self.joints.values():
            log(f'  {joint}\n')

    def to_sdf_string(self):
        sdf_node = ET.Element('sdf', {'version': '1.11'})
        model_node = ET.SubElement(sdf_node, 'model', {'name': self.name})

        if self.root_link is not None:
            self.links[self.root_link].to_sdf_element(model_node)

        for link in self.links.values():
            if link.name == self.root_link:
                continue
            link.to_sdf_element(model_node)

        for joint in self.joints.values():
            joint.to_sdf_element(model_node)

        return ET.tostring(sdf_node)

    def save(self, path: Path):
        log(f'Saving SDF to "{path}"\n')
        os.makedirs(path, exist_ok=True)

        mesh_dir_path = path / 'meshes'
        if mesh_dir_path.exists():
            shutil.rmtree(mesh_dir_path)
        if self.tmp_meshes_dir_path.exists():
            shutil.move(str(self.tmp_meshes_dir_path), str(path))

        with open(path / 'model.sdf', 'w') as f:
            f.write(prettify_xml(self.to_sdf_string()))
