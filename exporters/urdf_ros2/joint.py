# -*- coding: utf-8 -*-
"""
Joint classes for URDF ROS2 exporter
Based on fusion2urdf-ros2 by dheena2k2
"""

import adsk
import re
from xml.etree.ElementTree import Element, SubElement
from . import utils


class Joint:
    def __init__(self, name, xyz, axis, parent, child, joint_type, upper_limit, lower_limit, rpy=None):
        self.name = name
        self.type = joint_type
        self.xyz = xyz
        self.rpy = rpy or [0, 0, 0]
        self.parent = parent
        self.child = child
        self.joint_xml = None
        self.tran_xml = None
        self.axis = axis
        self.upper_limit = upper_limit
        self.lower_limit = lower_limit

    def make_joint_xml(self):
        """Generate the joint XML"""
        joint = Element('joint')
        joint.attrib = {'name': self.name, 'type': self.type}

        origin = SubElement(joint, 'origin')
        rpy_str = ' '.join([str(_) for _ in self.rpy])
        origin.attrib = {'xyz': ' '.join([str(_) for _ in self.xyz]), 'rpy': rpy_str}
        parent = SubElement(joint, 'parent')
        parent.attrib = {'link': self.parent}
        child = SubElement(joint, 'child')
        child.attrib = {'link': self.child}

        if self.type in ('revolute', 'continuous', 'prismatic'):
            axis = SubElement(joint, 'axis')
            axis.attrib = {'xyz': ' '.join([str(_) for _ in self.axis])}

        if self.type in ('revolute', 'prismatic'):
            limit = SubElement(joint, 'limit')
            limit.attrib = {
                'upper': str(self.upper_limit),
                'lower': str(self.lower_limit),
                'effort': '100',
                'velocity': '100'
            }

        self.joint_xml = "\n".join(utils.prettify(joint).split("\n")[1:])

    def make_transmission_xml(self):
        """Generate the transmission XML"""
        tran = Element('transmission')
        tran.attrib = {'name': self.name + '_tran'}

        joint_type = SubElement(tran, 'type')
        joint_type.text = 'transmission_interface/SimpleTransmission'

        joint = SubElement(tran, 'joint')
        joint.attrib = {'name': self.name}
        hardwareInterface_joint = SubElement(joint, 'hardwareInterface')
        hardwareInterface_joint.text = 'hardware_interface/EffortJointInterface'

        actuator = SubElement(tran, 'actuator')
        actuator.attrib = {'name': self.name + '_actr'}
        hardwareInterface_actr = SubElement(actuator, 'hardwareInterface')
        hardwareInterface_actr.text = 'hardware_interface/EffortJointInterface'
        mechanicalReduction = SubElement(actuator, 'mechanicalReduction')
        mechanicalReduction.text = '1'

        self.tran_xml = "\n".join(utils.prettify(tran).split("\n")[1:])


def make_joints_dict(root, msg, base_link_name=None):
    """Generate joints dictionary from Fusion design"""
    joint_type_list = [
        'fixed', 'revolute', 'prismatic', 'Cylinderical',
        'PinSlot', 'Planner', 'Ball'
    ]

    joints_dict = {}

    def collect_joints(component):
        """Collect joints from component and its children"""
        joints = list(component.joints)
        for occ in component.occurrences:
            joints.extend(collect_joints(occ.component))
        return joints

    all_joints = collect_joints(root)

    for joint in all_joints:
        joint_dict = {}
        joint_type = joint_type_list[joint.jointMotion.jointType]
        joint_dict['type'] = joint_type

        joint_dict['axis'] = [0, 0, 0]
        joint_dict['upper_limit'] = 0.0
        joint_dict['lower_limit'] = 0.0

        if joint_type == 'revolute':
            joint_dict['axis'] = [round(i, 6) for i in joint.jointMotion.rotationAxisVector.asArray()]
            max_enabled = joint.jointMotion.rotationLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.rotationLimits.isMinimumValueEnabled

            # URDF q=0 is the assembled pose (the joint origin is built from the
            # current occurrence transforms), but Fusion limits are measured from
            # the joint's internal zero. Shift them by the current angle so the
            # assembled pose lands correctly inside the limit range.
            try:
                cur = joint.jointMotion.rotationValue
            except:
                cur = 0.0

            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round(joint.jointMotion.rotationLimits.maximumValue - cur, 6)
                joint_dict['lower_limit'] = round(joint.jointMotion.rotationLimits.minimumValue - cur, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + ' is not set its lower limit.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + ' is not set its upper limit.'
                break
            else:
                joint_dict['type'] = 'continuous'

        elif joint_type == 'prismatic':
            joint_dict['axis'] = [round(i, 6) for i in joint.jointMotion.slideDirectionVector.asArray()]
            max_enabled = joint.jointMotion.slideLimits.isMaximumValueEnabled
            min_enabled = joint.jointMotion.slideLimits.isMinimumValueEnabled

            # Same offset as revolute: shift the slide limits by the current
            # (assembled) slide value so URDF q=0 matches the assembled pose.
            try:
                cur = joint.jointMotion.slideValue
            except:
                cur = 0.0

            if max_enabled and min_enabled:
                joint_dict['upper_limit'] = round((joint.jointMotion.slideLimits.maximumValue - cur) / 100, 6)
                joint_dict['lower_limit'] = round((joint.jointMotion.slideLimits.minimumValue - cur) / 100, 6)
            elif max_enabled and not min_enabled:
                msg = joint.name + ' is not set its lower limit.'
                break
            elif not max_enabled and min_enabled:
                msg = joint.name + ' is not set its upper limit.'
                break

        # Check if parent is base_link
        is_parent_base = (base_link_name and joint.occurrenceTwo.name == base_link_name) or \
                         (not base_link_name and joint.occurrenceTwo.component.name == 'base_link')

        if is_parent_base:
            joint_dict['parent'] = 'base_link'
        else:
            joint_dict['parent'] = utils.normalize_name(joint.occurrenceTwo.name)

        # Check if child is base_link
        is_child_base = (base_link_name and joint.occurrenceOne.name == base_link_name) or \
                        (not base_link_name and joint.occurrenceOne.component.name == 'base_link')

        if is_child_base:
            joint_dict['child'] = 'base_link'
        else:
            joint_dict['child'] = utils.normalize_name(joint.occurrenceOne.name)

        # The exported STL meshes are in each component's LOCAL coordinates, so
        # every link frame coincides with its component origin and the visual
        # origin is 0. The joint origin must therefore be the full relative pose
        # of the child component frame expressed in the parent frame:
        #     T_rel = M_parent^-1 * M_child
        # This yields both translation (in the parent frame) and rotation (rpy),
        # so it works for rotated components -- unlike the old world-axis
        # subtraction, which scrambled limbs whenever a part was rotated.
        try:
            parent_tf = joint.occurrenceTwo.transform
            child_tf = joint.occurrenceOne.transform
            xyz, rpy = utils.get_relative_transform(parent_tf, child_tf)
            joint_dict['xyz'] = xyz
            joint_dict['rpy'] = rpy

            # Fusion reports the rotation/slide axis in world coordinates;
            # URDF expects it in the child (joint) frame.
            if joint_type in ('revolute', 'prismatic') or joint_dict['type'] == 'continuous':
                joint_dict['axis'] = utils.world_axis_to_child(child_tf, joint_dict['axis'])
        except Exception as e:
            msg = joint.name + " transform error: " + str(e)
            break

        joints_dict[utils.normalize_name(joint.name)] = joint_dict

    return joints_dict, msg


# ---------------------------------------------------------------------------
# Rigid Group link mode
# ---------------------------------------------------------------------------

_JOINT_TYPE_LIST = [
    'fixed', 'revolute', 'prismatic', 'Cylinderical', 'PinSlot', 'Planner', 'Ball'
]


def _collect_joints(component):
    """Collect joints from a component and all of its nested components."""
    joints = list(component.joints)
    for occ in component.occurrences:
        joints.extend(_collect_joints(occ.component))
    return joints


def _joint_point_m(joint):
    """World position of the joint origin, in METERS."""
    try:
        geo = joint.geometryOrOriginOne
        if geo is not None:
            return [v / 100.0 for v in geo.origin.asArray()]
    except:
        pass
    try:
        return [v / 100.0 for v in joint.geometry.origin.asArray()]  # as-built
    except:
        pass
    return [v / 100.0 for v in joint.occurrenceOne.transform.translation.asArray()]


def _joint_motion(joint, success_msg):
    """
    Extract (type, axis, upper_limit, lower_limit, msg) from a Fusion joint.
    The axis is left in WORLD coordinates because rigid-group link frames are
    world-aligned. Limits are shifted by the current (assembled) joint value,
    mirroring make_joints_dict.
    """
    jtype = _JOINT_TYPE_LIST[joint.jointMotion.jointType]
    axis = [0, 0, 0]
    upper = 0.0
    lower = 0.0

    if jtype == 'revolute':
        axis = [round(i, 6) for i in joint.jointMotion.rotationAxisVector.asArray()]
        limits = joint.jointMotion.rotationLimits
        max_enabled = limits.isMaximumValueEnabled
        min_enabled = limits.isMinimumValueEnabled
        try:
            cur = joint.jointMotion.rotationValue
        except:
            cur = 0.0
        if max_enabled and min_enabled:
            upper = round(limits.maximumValue - cur, 6)
            lower = round(limits.minimumValue - cur, 6)
        elif max_enabled and not min_enabled:
            return jtype, axis, upper, lower, joint.name + ' is not set its lower limit.'
        elif min_enabled and not max_enabled:
            return jtype, axis, upper, lower, joint.name + ' is not set its upper limit.'
        else:
            jtype = 'continuous'

    elif jtype == 'prismatic':
        axis = [round(i, 6) for i in joint.jointMotion.slideDirectionVector.asArray()]
        limits = joint.jointMotion.slideLimits
        max_enabled = limits.isMaximumValueEnabled
        min_enabled = limits.isMinimumValueEnabled
        try:
            cur = joint.jointMotion.slideValue
        except:
            cur = 0.0
        if max_enabled and min_enabled:
            upper = round((limits.maximumValue - cur) / 100, 6)
            lower = round((limits.minimumValue - cur) / 100, 6)
        elif max_enabled and not min_enabled:
            return jtype, axis, upper, lower, joint.name + ' is not set its lower limit.'
        elif min_enabled and not max_enabled:
            return jtype, axis, upper, lower, joint.name + ' is not set its upper limit.'

    return jtype, axis, upper, lower, success_msg


def make_rigid_group_joints_dict(root, occ_to_group, msg):
    """
    Build the joints dictionary for the rigid-group link mode.

    Each kept joint connects two different group-links. Joints whose endpoints
    are loose (not in any group) or fall inside the same group are ignored.

    Returns (joints_dict, link_frames, msg) where link_frames maps every link
    name to its world-aligned frame translation P (meters); 'base_link' is the
    world origin.
    """
    success_msg = msg
    joints_dict = {}
    link_frames = {'base_link': [0.0, 0.0, 0.0]}

    raw = []
    for joint in _collect_joints(root):
        occ_one = joint.occurrenceOne
        occ_two = joint.occurrenceTwo
        if occ_one is None or occ_two is None:
            continue

        parent_link = occ_to_group.get(occ_two.name)
        child_link = occ_to_group.get(occ_one.name)
        if parent_link is None or child_link is None:
            continue
        if parent_link == child_link:
            continue

        jtype, axis, upper, lower, msg2 = _joint_motion(joint, success_msg)
        if msg2 != success_msg:
            return {}, {}, msg2

        point = _joint_point_m(joint)
        raw.append({
            'name': utils.normalize_name(joint.name),
            'parent': parent_link,
            'child': child_link,
            'point': point,
            'type': jtype,
            'axis': axis,
            'upper': upper,
            'lower': lower,
        })
        # The child link frame sits at the connecting joint point.
        link_frames[child_link] = point

    # Second pass: joint origin = P_child - P_parent (both world points).
    for r in raw:
        p_parent = link_frames.get(r['parent'], [0.0, 0.0, 0.0])
        xyz = [round(r['point'][i] - p_parent[i], 6) for i in range(3)]
        joints_dict[r['name']] = {
            'type': r['type'],
            'axis': r['axis'],
            'upper_limit': r['upper'],
            'lower_limit': r['lower'],
            'parent': r['parent'],
            'child': r['child'],
            'xyz': xyz,
            'rpy': [0, 0, 0],
        }

    return joints_dict, link_frames, msg
