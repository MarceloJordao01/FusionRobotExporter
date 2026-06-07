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
from .util import normalize_name, transform2_to_pose, prettify_xml, cm_to_m, kg_cm2_to_kg_m2, world_inertia_to_com_inertia, matrix3d_to_rpy, name_to_path
from .link import Link, LinkElement, LinkElementType, LinkGeometry, LinkGeometryType, LinkInertial
from .joint import Joint, JointType
from .pose import Pose
from ...core.mesh import export_body_to_obj
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

        # Add dummy base_link
        self.links['base_link'] = Link('base_link')
        self.links['base_link'].inertial = LinkInertial(Pose(), mass=1e-4, ixx=1e-9, ixy=0, ixz=0, iyy=1e-9, iyz=0, izz=1e-9)
        self.joints['base_link_joint'] = Joint('base_link_joint', JointType.FIXED, None, 'base_link', self.root_link)
        self.root_link = 'base_link'
        # The dummy base_link sits at the world origin.
        self.link_world['base_link'] = adsk.core.Matrix3D.create()

        # Attach sensors (sensor__* components) to their target links.
        self.attach_sensors()

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
        log(f'parse_occurrence: {occurrence.name}; prefix="{prefix}"\n')

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
        log(f'add_link: occurence="{occurrence.name}"; prefix="{prefix}" -> link_name={link_name}\n')

        if self.progress is not None and self.progress.is_cancelled():
            return

        # Sensor mounts (sensor__*) are not links.
        if not is_rigid_group_link and core_sensors.is_sensor_occurrence(occurrence):
            log(f'Occurence "{occurrence.name}" is a sensor mount, skipping\n')
            return

        if link_name in self.links:
            log(f'Link "{link_name}" already exists, skipping\n')
            return

        if occurrence.name in self.occurrences_in_rigid_groups:
            occurrence.attributes.add('FusionSDF', 'link_name', self.occurrences_in_rigid_groups[occurrence.name])
            log(f'Occurence "{occurrence.name}" is in rigid group, skipping\n')
            return

        # In rigid-group mode, components that are not part of any rigid group
        # are ignored (only the merged rigid-group links are emitted).
        if self.link_mode == 'rigid_groups' and not is_rigid_group_link:
            log(f'Occurence "{occurrence.name}" is loose (no rigid group), skipping\n')
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
        link_dir_path = self.tmp_meshes_dir_path / name_to_path(link.name)

        for body in bodies:
            if self.progress is not None:
                if self.progress.is_cancelled():
                    break
                # One step per body so the count reflects bodies, not links.
                self.progress.step(link_name)
            log(f'visual: {body.name}\n')
            visual = LinkElement(LinkElementType.VISUAL, link.name + '__' + normalize_name(body.name) + '_visual')
            visual.geometry = LinkGeometry(LinkGeometryType.MESH)
            visual.geometry.mesh_uri = 'meshes/' + name_to_path(visual.name) + '.obj'
            # Visual pose is relative to link frame
            # Since meshes are exported in component-local coords, and link.pose = occurrence.transform2,
            # visual should have identity pose (mesh origin = link origin)
            visual.pose = None
            mesh_path = self.tmp_dir_path / visual.geometry.mesh_uri
            link.visuals[visual.name] = visual

            collision = LinkElement(LinkElementType.COLLISION, link.name + '__' + normalize_name(body.name) + '_collision')

            use_collision_mesh = False
            if self.design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
                use_collision_mesh_parameter = self.design.userParameters.itemByName(collision.name + '_USE_MESH')
                use_collision_mesh = use_collision_mesh_parameter is not None and use_collision_mesh_parameter.value

            if use_collision_mesh:
                log(f'using mesh for {collision.name}\n')
                collision.pose = visual.pose
                collision.geometry = visual.geometry
            else:
                log(f'using oriented minimum bounding box for {collision.name}\n')
                obb = body.orientedMinimumBoundingBox
                obb_matrix3d = adsk.core.Matrix3D.create()
                obb_matrix3d.setWithCoordinateSystem(obb.centerPoint, obb.heightDirection, obb.widthDirection, obb.lengthDirection)
                # OBB centerPoint is in component-local coords, transform to link-relative
                # Since link.pose = occurrence.transform2, we need inverse
                obb_pose_global = Pose(cm_to_m(obb.centerPoint.asArray()), matrix3d_to_rpy(obb_matrix3d))
                collision.pose = link.pose.inverse() * obb_pose_global
                collision.pose.relative_to = None
                collision.geometry = LinkGeometry(LinkGeometryType.BOX)
                collision.geometry.size = [cm_to_m(obb.height), cm_to_m(obb.width), cm_to_m(obb.length)]
            link.collisions[collision.name] = collision

            os.makedirs(link_dir_path, exist_ok=True)

            if self.meshes_cache_dir_path:
                mesh_cache_path = self.meshes_cache_dir_path / visual.geometry.mesh_uri.replace('meshes/', '')
                if mesh_cache_path.exists():
                    log(f'using cached mesh "{mesh_cache_path}"\n')
                    shutil.copy(mesh_cache_path, mesh_path)
                    if mesh_cache_path.with_suffix('.mtl').exists():
                        shutil.copy(mesh_cache_path.with_suffix('.mtl'), mesh_path.with_suffix('.mtl'))
                    continue

            export_manager = self.design.exportManager
            success, error = export_body_to_obj(export_manager, body, str(mesh_path))
            if not success:
                log(f'ERROR: Failed to export mesh "{mesh_path}": {error}\n')

        physical_properties = occurrence.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)
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

        self.joints[joint.name] = joint

    def add_rigid_group(self, parent_occurrences, parent_pose, rigid_group, prefix: str):
        link_name = prefix + normalize_name(rigid_group.name)
        log(f'add_rigid_group: rigid_group="{rigid_group.name}"; prefix="{prefix}" -> link_name={link_name}\n')

        rigid_group_occurrence = parent_occurrences.addNewComponent(adsk.core.Matrix3D.create())
        rigid_group_component = rigid_group_occurrence.component
        rigid_group_component.name = link_name

        # Merge member bodies into the temp occurrence, handling linked/external
        # components (proxy bodies may be empty -> fall back to nativeObject).
        # The merge only keeps the UI alive (tick); the bodies are counted once
        # when add_link exports them below, so the count stays body-based.
        if self.progress is not None:
            self.progress.set_phase(f'Merging "{link_name}"')
        merge_errors = merge_occurrence_bodies(
            self.design.rootComponent, rigid_group_occurrence, rigid_group.occurrences,
            progress=self.progress, step_bodies=False, phase=f'Merging "{link_name}"')
        for err in merge_errors:
            log(f'add_rigid_group: {err}\n')

        if self.progress is not None:
            self.progress.set_phase(f'Exporting "{link_name}"')
        rigid_group_link = self.add_link(rigid_group_occurrence, '', rigid_group_pose=transform2_to_pose(parent_pose), is_rigid_group_link=True)
        rigid_group_occurrence.deleteMe()

        for occurrence in rigid_group.occurrences:
            self.occurrences_in_rigid_groups[occurrence.name] = link_name + '_1'

        log(f'self.occurrences_in_rigid_groups: {self.occurrences_in_rigid_groups}\n')
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
