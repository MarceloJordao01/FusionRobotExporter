# -*- coding: utf-8 -*-
"""
Sensor support via naming convention (shared by all exporters).

A component named

    sensor__<type>__<link>[__<name>]

(double underscore '__' as the field separator, parsed from the RAW component
name so it survives the single-underscore collapse of normalize_name) is treated
as a simulation sensor mounted on <link>. Its pose is taken from the component's
transform expressed in the <link> frame. Such components are excluded from the
links / joints / meshes of the robot.

  <type> in: camera, depth, lidar, imu, gps, contact
  <link>   : the target link name (normalized); 'base_link' for the base
  <name>   : optional sensor name (defaults to <type>)

Examples:
  sensor__camera__base_link__frontal
  sensor__lidar__base_link__topo
  sensor__imu__base_link
"""

import math
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

import adsk.core

from .mesh import normalize_name

SENSOR_PREFIX = 'sensor__'
SENSOR_TYPES = ('camera', 'depth', 'lidar', 'imu', 'gps', 'contact')

# Reasonable defaults per sensor type. Tweak in the generated files as needed.
SENSOR_DEFAULTS = {
    'camera':  {'update_rate': 30, 'horizontal_fov': 1.396263, 'width': 640, 'height': 480,
                'near': 0.1, 'far': 100.0},
    'depth':   {'update_rate': 30, 'horizontal_fov': 1.047198, 'width': 640, 'height': 480,
                'near': 0.05, 'far': 8.0},
    'lidar':   {'update_rate': 10, 'samples': 360, 'min_angle': -math.pi, 'max_angle': math.pi,
                'range_min': 0.1, 'range_max': 30.0},
    'imu':     {'update_rate': 100},
    'gps':     {'update_rate': 1},
    'contact': {'update_rate': 50},
}


# ---------------------------------------------------------------------------
# Detection / parsing
# ---------------------------------------------------------------------------

def is_sensor_name(raw_name):
    return bool(raw_name) and raw_name.startswith(SENSOR_PREFIX)


def is_sensor_occurrence(occ):
    """True if this occurrence is a sensor mount (by its component name)."""
    try:
        return is_sensor_name(occ.component.name)
    except Exception:
        return False


def parse_sensor_name(raw_name):
    """
    Parse 'sensor__<type>__<link>[__<name>]'. Returns a dict
    {'type', 'link', 'name'} or None if the name is not a valid sensor tag.
    """
    if not is_sensor_name(raw_name):
        return None
    parts = raw_name.split('__')
    # parts[0] == 'sensor'
    if len(parts) < 3:
        return None
    sensor_type = parts[1].strip().lower()
    link_token = parts[2].strip()
    name_token = parts[3].strip() if len(parts) >= 4 and parts[3].strip() else sensor_type
    if sensor_type not in SENSOR_TYPES or not link_token:
        return None
    return {
        'type': sensor_type,
        'link': normalize_name(link_token),
        'name': normalize_name(name_token),
    }


# ---------------------------------------------------------------------------
# Pose helpers (kept independent of any exporter's utils)
# ---------------------------------------------------------------------------

def _mat_mul(a, b):
    """Multiply two 4x4 matrices given as flat 16-element row-major lists."""
    out = [0.0] * 16
    for r in range(4):
        for c in range(4):
            out[r * 4 + c] = (
                a[r * 4 + 0] * b[0 * 4 + c] +
                a[r * 4 + 1] * b[1 * 4 + c] +
                a[r * 4 + 2] * b[2 * 4 + c] +
                a[r * 4 + 3] * b[3 * 4 + c]
            )
    return out


def _rigid_inverse(m):
    """Inverse of a rigid transform (rotation + translation), flat 16 list."""
    # R^T
    r = [
        m[0], m[4], m[8],
        m[1], m[5], m[9],
        m[2], m[6], m[10],
    ]
    t = [m[3], m[7], m[11]]
    # -R^T t
    tx = -(r[0] * t[0] + r[1] * t[1] + r[2] * t[2])
    ty = -(r[3] * t[0] + r[4] * t[1] + r[5] * t[2])
    tz = -(r[6] * t[0] + r[7] * t[1] + r[8] * t[2])
    return [
        r[0], r[1], r[2], tx,
        r[3], r[4], r[5], ty,
        r[6], r[7], r[8], tz,
        0.0, 0.0, 0.0, 1.0,
    ]


def _matrix_to_xyz_rpy(m):
    """
    Extract (xyz in METERS, rpy radians) from a flat 16 row-major transform whose
    translation is in Fusion internal cm. rpy is fixed-axis XYZ (URDF/SDF).
    """
    xyz = [round(m[3] / 100.0, 6), round(m[7] / 100.0, 6), round(m[11] / 100.0, 6)]

    r00, r01, r02 = m[0], m[1], m[2]
    r10, r11, r12 = m[4], m[5], m[6]
    r20, r21, r22 = m[8], m[9], m[10]

    sy = math.sqrt(r00 * r00 + r10 * r10)
    if sy > 1e-6:
        roll = math.atan2(r21, r22)
        pitch = math.atan2(-r20, sy)
        yaw = math.atan2(r10, r00)
    else:
        roll = math.atan2(-r12, r11)
        pitch = math.atan2(-r20, sy)
        yaw = 0.0
    rpy = [round(roll, 6), round(pitch, 6), round(yaw, 6)]
    return xyz, rpy


def _relative_xyz_rpy(link_tf, sensor_tf):
    """Pose of sensor_tf expressed in link_tf's frame -> (xyz m, rpy rad)."""
    link = list(link_tf.asArray())
    sensor = list(sensor_tf.asArray())
    rel = _mat_mul(_rigid_inverse(link), sensor)
    return _matrix_to_xyz_rpy(rel)


# ---------------------------------------------------------------------------
# Link transform maps (per link mode) + sensor collection
# ---------------------------------------------------------------------------

def build_component_link_transforms(root, base_link_name=None):
    """
    Map link_name -> world Matrix3D for the component link mode. Sensor mounts
    are skipped. The base link is keyed as 'base_link'.
    """
    transforms = {}

    def rec(occurrences):
        for occ in occurrences:
            if is_sensor_occurrence(occ):
                continue
            is_base = (base_link_name and occ.name == base_link_name) or \
                      (not base_link_name and occ.component.name == 'base_link')
            key = 'base_link' if is_base else normalize_name(occ.name)
            transforms[key] = occ.transform
            if occ.childOccurrences.count > 0:
                rec(occ.childOccurrences)

    rec(root.occurrences)
    return transforms


def build_rigid_link_transforms(link_frames):
    """
    Map link_name -> world Matrix3D for the rigid-group link mode. Each group
    link frame is world-aligned and translated to its joint point P (meters),
    matching make_rigid_group_joints_dict. 'base_link' is the world origin.
    """
    transforms = {}
    for link_name, P in link_frames.items():
        m = adsk.core.Matrix3D.create()
        m.translation = adsk.core.Vector3D.create(P[0] * 100.0, P[1] * 100.0, P[2] * 100.0)
        transforms[link_name] = m
    return transforms


def collect_sensors(root, link_transforms):
    """
    Walk the assembly and return a list of sensor dicts:
      {'type', 'name', 'link', 'xyz', 'rpy', 'params'}  (or {'error': ...})

    link_transforms: {link_name: Matrix3D} built by build_component_link_transforms
    (component mode) or by the caller for the rigid-group mode.
    """
    sensors = []

    def rec(occurrences):
        for occ in occurrences:
            if is_sensor_occurrence(occ):
                spec = parse_sensor_name(occ.component.name)
                if spec is None:
                    sensors.append({'error': f"invalid sensor name '{occ.component.name}'"})
                else:
                    link_tf = link_transforms.get(spec['link'])
                    if link_tf is None:
                        sensors.append({**spec,
                                        'error': f"sensor '{spec['name']}': link '{spec['link']}' not found"})
                    else:
                        xyz, rpy = _relative_xyz_rpy(link_tf, occ.transform)
                        sensors.append({
                            'type': spec['type'],
                            'name': spec['name'],
                            'link': spec['link'],
                            'xyz': xyz,
                            'rpy': rpy,
                            'params': dict(SENSOR_DEFAULTS.get(spec['type'], {})),
                        })
            if occ.childOccurrences.count > 0:
                rec(occ.childOccurrences)

    rec(root.occurrences)
    return sensors


# ---------------------------------------------------------------------------
# XML generation
# ---------------------------------------------------------------------------

# Sensor <type> string per target. ros1 = Gazebo classic, ros2/sdf = new Gz.
_TYPE_STR = {
    'camera':  {'classic': 'camera',      'gz': 'camera'},
    'depth':   {'classic': 'depth',       'gz': 'depth_camera'},
    'lidar':   {'classic': 'ray',         'gz': 'gpu_lidar'},
    'imu':     {'classic': 'imu',         'gz': 'imu'},
    'gps':     {'classic': 'gps',         'gz': 'navsat'},
    'contact': {'classic': 'contact',     'gz': 'contact'},
}


def _pose_str(sensor):
    vals = list(sensor['xyz']) + list(sensor['rpy'])
    return ' '.join(str(v) for v in vals)


def _add_payload(sensor_el, sensor):
    """Add the type-specific child block (camera/ray/imu...) to <sensor>."""
    stype = sensor['type']
    p = sensor['params']

    if stype in ('camera', 'depth'):
        cam = SubElement(sensor_el, 'camera')
        SubElement(cam, 'horizontal_fov').text = str(p['horizontal_fov'])
        image = SubElement(cam, 'image')
        SubElement(image, 'width').text = str(p['width'])
        SubElement(image, 'height').text = str(p['height'])
        SubElement(image, 'format').text = 'R8G8B8' if stype == 'camera' else 'L8'
        clip = SubElement(cam, 'clip')
        SubElement(clip, 'near').text = str(p['near'])
        SubElement(clip, 'far').text = str(p['far'])

    elif stype == 'lidar':
        ray = SubElement(sensor_el, 'ray')
        scan = SubElement(ray, 'scan')
        hz = SubElement(scan, 'horizontal')
        SubElement(hz, 'samples').text = str(p['samples'])
        SubElement(hz, 'resolution').text = '1'
        SubElement(hz, 'min_angle').text = str(p['min_angle'])
        SubElement(hz, 'max_angle').text = str(p['max_angle'])
        rng = SubElement(ray, 'range')
        SubElement(rng, 'min').text = str(p['range_min'])
        SubElement(rng, 'max').text = str(p['range_max'])
        SubElement(rng, 'resolution').text = '0.01'

    elif stype == 'imu':
        SubElement(sensor_el, 'imu')

    elif stype == 'gps':
        SubElement(sensor_el, 'gps')

    elif stype == 'contact':
        contact = SubElement(sensor_el, 'contact')
        SubElement(contact, 'collision').text = '__default__'


def _add_classic_plugin(sensor_el, sensor):
    """Add a Gazebo-classic gazebo_ros plugin (ROS1)."""
    stype = sensor['type']
    name = sensor['name']
    if stype in ('camera', 'depth'):
        fn = 'libgazebo_ros_camera.so'
    elif stype == 'lidar':
        fn = 'libgazebo_ros_ray_sensor.so'
    elif stype == 'imu':
        fn = 'libgazebo_ros_imu_sensor.so'
    elif stype == 'gps':
        fn = 'libgazebo_ros_gps_sensor.so'
    else:
        return
    plugin = SubElement(sensor_el, 'plugin')
    plugin.set('name', name + '_plugin')
    plugin.set('filename', fn)
    ros = SubElement(plugin, 'ros')
    SubElement(ros, 'namespace').text = '/'
    SubElement(plugin, 'frame_name').text = sensor['link']


def make_urdf_sensor_xml(sensor, ros_version='ros2'):
    """
    Build a '<gazebo reference="link"><sensor>...</sensor></gazebo>' XML string
    for one sensor. ros_version: 'ros1' (Gazebo classic plugins) or 'ros2'
    (new Gz; sensors driven by the gz sensors system, bridged via ros_gz).
    """
    if 'error' in sensor:
        return f'<!-- sensor error: {sensor["error"]} -->\n'

    target = 'classic' if ros_version == 'ros1' else 'gz'
    gazebo = Element('gazebo')
    gazebo.set('reference', sensor['link'])

    sensor_el = SubElement(gazebo, 'sensor')
    sensor_el.set('name', sensor['name'])
    sensor_el.set('type', _TYPE_STR[sensor['type']][target])

    SubElement(sensor_el, 'always_on').text = '1'
    SubElement(sensor_el, 'update_rate').text = str(sensor['params'].get('update_rate', 30))
    SubElement(sensor_el, 'visualize').text = 'true'
    SubElement(sensor_el, 'pose').text = _pose_str(sensor)
    SubElement(sensor_el, 'topic').text = sensor['name']

    _add_payload(sensor_el, sensor)

    if ros_version == 'ros1':
        _add_classic_plugin(sensor_el, sensor)

    rough = tostring(gazebo, 'utf-8')
    pretty = minidom.parseString(rough).toprettyxml(indent='  ')
    return '\n'.join(pretty.split('\n')[1:])  # drop the <?xml ?> header


def make_sdf_sensor_element(sensor):
    """
    Build an ElementTree <sensor> Element for SDF (to be appended to the parent
    <link>). The pose is relative to the link frame. Returns None on error.
    """
    if 'error' in sensor:
        return None

    sensor_el = Element('sensor')
    sensor_el.set('name', sensor['name'])
    sensor_el.set('type', _TYPE_STR[sensor['type']]['gz'])
    SubElement(sensor_el, 'always_on').text = '1'
    SubElement(sensor_el, 'update_rate').text = str(sensor['params'].get('update_rate', 30))
    SubElement(sensor_el, 'visualize').text = 'true'
    SubElement(sensor_el, 'pose').text = _pose_str(sensor)
    SubElement(sensor_el, 'topic').text = sensor['name']
    _add_payload(sensor_el, sensor)
    return sensor_el
