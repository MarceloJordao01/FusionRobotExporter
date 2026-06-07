# -*- coding: utf-8 -*-
"""
Link classes for URDF ROS2 exporter
Based on fusion2urdf-ros2 by dheena2k2
"""

import adsk
import re
from xml.etree.ElementTree import Element, SubElement
from . import utils
from ...core import sensors as core_sensors


class Link:
    def __init__(self, name, xyz, center_of_mass, repo, mass, inertia_tensor, rpy=None):
        self.name = name
        self.xyz = [-_ for _ in xyz]
        self.rpy = [-_ for _ in rpy] if rpy else [0, 0, 0]
        self.center_of_mass = center_of_mass
        self.link_xml = None
        self.repo = repo
        self.mass = mass
        self.inertia_tensor = inertia_tensor

    def make_link_xml(self):
        """Generate the link XML"""
        link = Element('link')
        link.attrib = {'name': self.name}

        # Inertial
        inertial = SubElement(link, 'inertial')
        origin_i = SubElement(inertial, 'origin')
        origin_i.attrib = {'xyz': ' '.join([str(_) for _ in self.center_of_mass]), 'rpy': '0 0 0'}
        mass = SubElement(inertial, 'mass')
        mass.attrib = {'value': str(self.mass)}
        inertia = SubElement(inertial, 'inertia')
        inertia.attrib = {
            'ixx': str(self.inertia_tensor[0]),
            'iyy': str(self.inertia_tensor[1]),
            'izz': str(self.inertia_tensor[2]),
            'ixy': str(self.inertia_tensor[3]),
            'iyz': str(self.inertia_tensor[4]),
            'ixz': str(self.inertia_tensor[5])
        }

        # Visual
        # The STL is exported per-occurrence in the component's LOCAL coordinates,
        # so the mesh is already aligned with the link frame: origin stays 0.
        visual = SubElement(link, 'visual')
        origin_v = SubElement(visual, 'origin')
        origin_v.attrib = {'xyz': '0 0 0', 'rpy': '0 0 0'}
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.attrib = {
            'filename': 'package://' + self.repo + self.name + '.stl',
            'scale': '0.001 0.001 0.001'
        }
        material = SubElement(visual, 'material')
        material.attrib = {'name': 'silver'}

        # Collision
        collision = SubElement(link, 'collision')
        origin_c = SubElement(collision, 'origin')
        origin_c.attrib = {'xyz': '0 0 0', 'rpy': '0 0 0'}
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.attrib = {
            'filename': 'package://' + self.repo + self.name + '.stl',
            'scale': '0.001 0.001 0.001'
        }

        self.link_xml = "\n".join(utils.prettify(link).split("\n")[1:])


def make_inertial_dict(root, msg, base_link_name=None, progress=None):
    """Generate inertial dictionary from Fusion design"""
    inertial_dict = {}

    def process_occurrences(occurrences):
        for occs in occurrences:
            if progress is not None and progress.is_cancelled():
                return

            # Sensor mounts are not links.
            if core_sensors.is_sensor_occurrence(occs):
                continue

            # Skip if no bodies (subassembly container)
            if occs.bRepBodies.count == 0 and occs.childOccurrences.count > 0:
                process_occurrences(occs.childOccurrences)
                continue

            occs_dict = {}
            if progress is not None:
                progress.step(utils.normalize_name(occs.name))
            prop = occs.getPhysicalProperties(adsk.fusion.CalculationAccuracy.VeryHighCalculationAccuracy)

            occs_dict['name'] = utils.normalize_name(occs.name)
            occs_dict['mass'] = prop.mass

            # World COM (m) for the parallel-axis shift of the world-aligned
            # inertia tensor; LOCAL-frame COM for the URDF <inertial><origin>.
            world_com_m = [_ / 100.0 for _ in prop.centerOfMass.asArray()]
            occs_dict['center_of_mass'] = utils.world_point_to_link(
                occs.transform, prop.centerOfMass.asArray())

            (_, xx, yy, zz, xy, yz, xz) = prop.getXYZMomentsOfInertia()
            moment_inertia_world = [_ / 10000.0 for _ in [xx, yy, zz, xy, yz, xz]]
            occs_dict['inertia'] = utils.origin2center_of_mass(moment_inertia_world, world_com_m, prop.mass)

            # Extract transform (xyz and rpy) from occurrence
            xyz, rpy = utils.get_occurrence_transform(occs)
            occs_dict['xyz'] = xyz
            occs_dict['rpy'] = rpy

            # Check if this is the base_link
            is_base = (base_link_name and occs.name == base_link_name) or \
                      (not base_link_name and occs.component.name == 'base_link')

            if is_base:
                inertial_dict['base_link'] = occs_dict
            else:
                inertial_dict[utils.normalize_name(occs.name)] = occs_dict

            # Process child occurrences (subassemblies)
            if occs.childOccurrences.count > 0:
                process_occurrences(occs.childOccurrences)

    process_occurrences(root.occurrences)
    return inertial_dict, msg
