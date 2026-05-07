# -*- coding: utf-8 -*-
"""
Sensor definitions loader and XML generator for URDF/SDF
Reads sensor configuration from JSON file and generates appropriate XML
"""

import json
import os
from pathlib import Path


# Default parameters for each sensor type
SENSOR_DEFAULTS = {
    'camera': {
        'update_rate': 30.0,
        'width': 640,
        'height': 480,
        'fov': 1.047,  # ~60 degrees in radians
        'clip': {'near': 0.1, 'far': 100.0},
        'noise': {'type': 'gaussian', 'mean': 0.0, 'stddev': 0.007}
    },
    'depth_camera': {
        'update_rate': 30.0,
        'width': 640,
        'height': 480,
        'fov': 1.047,
        'clip': {'near': 0.1, 'far': 10.0},
        'noise': {'type': 'gaussian', 'mean': 0.0, 'stddev': 0.007}
    },
    'lidar': {
        'update_rate': 10.0,
        'samples': 360,
        'resolution': 1.0,
        'range': {'min': 0.1, 'max': 10.0},
        'angle': {'min': -3.14159, 'max': 3.14159},  # full 360 degrees
        'noise': {'type': 'gaussian', 'mean': 0.0, 'stddev': 0.01}
    },
    'imu': {
        'update_rate': 100.0,
        'accel_noise': {'mean': 0.0, 'stddev': 0.01},
        'gyro_noise': {'mean': 0.0, 'stddev': 0.01}
    },
    'gps': {
        'update_rate': 10.0,
        'position_noise': {'horizontal': 0.5, 'vertical': 1.0},
        'velocity_noise': {'horizontal': 0.1, 'vertical': 0.1}
    },
    'contact': {
        'update_rate': 100.0,
        'topic': 'contact_sensor'
    }
}


def load_sensors(json_path):
    """
    Load sensors from JSON file

    Parameters
    ----------
    json_path : str or Path
        Path to sensors.json file

    Returns
    -------
    list : List of sensor dictionaries with defaults applied
    """
    json_path = Path(json_path)
    if not json_path.exists():
        return []

    with open(json_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    sensors = data.get('sensors', [])

    # Apply defaults for each sensor
    for sensor in sensors:
        sensor_type = sensor.get('type', 'camera')
        defaults = SENSOR_DEFAULTS.get(sensor_type, {})

        # Merge defaults with provided params
        if 'params' not in sensor:
            sensor['params'] = {}

        for key, value in defaults.items():
            if key not in sensor['params']:
                sensor['params'][key] = value

    return sensors


def find_sensors_file(save_dir, design_name=None):
    """
    Find sensors.json file in common locations

    Searches in order:
    1. save_dir/sensors.json
    2. Fusion document directory (if available)

    Parameters
    ----------
    save_dir : str
        Export save directory
    design_name : str, optional
        Name of the design for filename matching

    Returns
    -------
    str or None : Path to sensors.json if found
    """
    search_paths = [
        Path(save_dir) / 'sensors.json',
        Path(save_dir).parent / 'sensors.json',
    ]

    for path in search_paths:
        if path.exists():
            return str(path)

    return None


def generate_sensor_link_urdf(sensor, indent='  '):
    """
    Generate URDF link element for a sensor

    Parameters
    ----------
    sensor : dict
        Sensor configuration
    indent : str
        Indentation string

    Returns
    -------
    str : URDF XML for sensor link
    """
    name = sensor['name']

    xml = f'{indent}<link name="{name}_link">\n'
    xml += f'{indent}  <visual>\n'
    xml += f'{indent}    <geometry>\n'
    xml += f'{indent}      <box size="0.01 0.01 0.01"/>\n'
    xml += f'{indent}    </geometry>\n'
    xml += f'{indent}  </visual>\n'
    xml += f'{indent}</link>\n'

    return xml


def generate_sensor_joint_urdf(sensor, indent='  '):
    """
    Generate URDF joint element to attach sensor to parent link

    Parameters
    ----------
    sensor : dict
        Sensor configuration
    indent : str
        Indentation string

    Returns
    -------
    str : URDF XML for sensor joint
    """
    name = sensor['name']
    parent = sensor.get('parent_link', 'base_link')
    pose = sensor.get('pose', {})
    xyz = pose.get('xyz', [0, 0, 0])
    rpy = pose.get('rpy', [0, 0, 0])

    xyz_str = ' '.join(str(v) for v in xyz)
    rpy_str = ' '.join(str(v) for v in rpy)

    xml = f'{indent}<joint name="{name}_joint" type="fixed">\n'
    xml += f'{indent}  <parent link="{parent}"/>\n'
    xml += f'{indent}  <child link="{name}_link"/>\n'
    xml += f'{indent}  <origin xyz="{xyz_str}" rpy="{rpy_str}"/>\n'
    xml += f'{indent}</joint>\n'

    return xml


def generate_camera_gazebo_urdf(sensor, indent='  '):
    """Generate Gazebo camera plugin URDF"""
    name = sensor['name']
    params = sensor.get('params', {})

    width = params.get('width', 640)
    height = params.get('height', 480)
    fov = params.get('fov', 1.047)
    update_rate = params.get('update_rate', 30.0)
    clip = params.get('clip', {'near': 0.1, 'far': 100.0})
    noise = params.get('noise', {'type': 'gaussian', 'mean': 0.0, 'stddev': 0.007})

    xml = f'{indent}<gazebo reference="{name}_link">\n'
    xml += f'{indent}  <sensor type="camera" name="{name}">\n'
    xml += f'{indent}    <update_rate>{update_rate}</update_rate>\n'
    xml += f'{indent}    <camera name="{name}_camera">\n'
    xml += f'{indent}      <horizontal_fov>{fov}</horizontal_fov>\n'
    xml += f'{indent}      <image>\n'
    xml += f'{indent}        <width>{width}</width>\n'
    xml += f'{indent}        <height>{height}</height>\n'
    xml += f'{indent}        <format>R8G8B8</format>\n'
    xml += f'{indent}      </image>\n'
    xml += f'{indent}      <clip>\n'
    xml += f'{indent}        <near>{clip["near"]}</near>\n'
    xml += f'{indent}        <far>{clip["far"]}</far>\n'
    xml += f'{indent}      </clip>\n'
    xml += f'{indent}      <noise>\n'
    xml += f'{indent}        <type>{noise["type"]}</type>\n'
    xml += f'{indent}        <mean>{noise["mean"]}</mean>\n'
    xml += f'{indent}        <stddev>{noise["stddev"]}</stddev>\n'
    xml += f'{indent}      </noise>\n'
    xml += f'{indent}    </camera>\n'
    xml += f'{indent}    <plugin name="{name}_controller" filename="libgazebo_ros_camera.so">\n'
    xml += f'{indent}      <ros>\n'
    xml += f'{indent}        <namespace>/</namespace>\n'
    xml += f'{indent}        <remapping>image_raw:={name}/image_raw</remapping>\n'
    xml += f'{indent}        <remapping>camera_info:={name}/camera_info</remapping>\n'
    xml += f'{indent}      </ros>\n'
    xml += f'{indent}      <camera_name>{name}</camera_name>\n'
    xml += f'{indent}      <frame_name>{name}_link</frame_name>\n'
    xml += f'{indent}    </plugin>\n'
    xml += f'{indent}  </sensor>\n'
    xml += f'{indent}</gazebo>\n'

    return xml


def generate_depth_camera_gazebo_urdf(sensor, indent='  '):
    """Generate Gazebo depth camera plugin URDF"""
    name = sensor['name']
    params = sensor.get('params', {})

    width = params.get('width', 640)
    height = params.get('height', 480)
    fov = params.get('fov', 1.047)
    update_rate = params.get('update_rate', 30.0)
    clip = params.get('clip', {'near': 0.1, 'far': 10.0})

    xml = f'{indent}<gazebo reference="{name}_link">\n'
    xml += f'{indent}  <sensor type="depth" name="{name}">\n'
    xml += f'{indent}    <update_rate>{update_rate}</update_rate>\n'
    xml += f'{indent}    <camera name="{name}_camera">\n'
    xml += f'{indent}      <horizontal_fov>{fov}</horizontal_fov>\n'
    xml += f'{indent}      <image>\n'
    xml += f'{indent}        <width>{width}</width>\n'
    xml += f'{indent}        <height>{height}</height>\n'
    xml += f'{indent}        <format>R8G8B8</format>\n'
    xml += f'{indent}      </image>\n'
    xml += f'{indent}      <clip>\n'
    xml += f'{indent}        <near>{clip["near"]}</near>\n'
    xml += f'{indent}        <far>{clip["far"]}</far>\n'
    xml += f'{indent}      </clip>\n'
    xml += f'{indent}    </camera>\n'
    xml += f'{indent}    <plugin name="{name}_controller" filename="libgazebo_ros_camera.so">\n'
    xml += f'{indent}      <ros>\n'
    xml += f'{indent}        <namespace>/</namespace>\n'
    xml += f'{indent}        <remapping>image_raw:={name}/image_raw</remapping>\n'
    xml += f'{indent}        <remapping>depth/image_raw:={name}/depth/image_raw</remapping>\n'
    xml += f'{indent}        <remapping>points:={name}/points</remapping>\n'
    xml += f'{indent}        <remapping>camera_info:={name}/camera_info</remapping>\n'
    xml += f'{indent}      </ros>\n'
    xml += f'{indent}      <camera_name>{name}</camera_name>\n'
    xml += f'{indent}      <frame_name>{name}_link</frame_name>\n'
    xml += f'{indent}      <min_depth>{clip["near"]}</min_depth>\n'
    xml += f'{indent}      <max_depth>{clip["far"]}</max_depth>\n'
    xml += f'{indent}    </plugin>\n'
    xml += f'{indent}  </sensor>\n'
    xml += f'{indent}</gazebo>\n'

    return xml


def generate_lidar_gazebo_urdf(sensor, indent='  '):
    """Generate Gazebo lidar/ray plugin URDF"""
    name = sensor['name']
    params = sensor.get('params', {})

    samples = params.get('samples', 360)
    resolution = params.get('resolution', 1.0)
    update_rate = params.get('update_rate', 10.0)
    range_config = params.get('range', {'min': 0.1, 'max': 10.0})
    angle_config = params.get('angle', {'min': -3.14159, 'max': 3.14159})
    noise = params.get('noise', {'type': 'gaussian', 'mean': 0.0, 'stddev': 0.01})

    xml = f'{indent}<gazebo reference="{name}_link">\n'
    xml += f'{indent}  <sensor type="ray" name="{name}">\n'
    xml += f'{indent}    <pose>0 0 0 0 0 0</pose>\n'
    xml += f'{indent}    <visualize>true</visualize>\n'
    xml += f'{indent}    <update_rate>{update_rate}</update_rate>\n'
    xml += f'{indent}    <ray>\n'
    xml += f'{indent}      <scan>\n'
    xml += f'{indent}        <horizontal>\n'
    xml += f'{indent}          <samples>{samples}</samples>\n'
    xml += f'{indent}          <resolution>{resolution}</resolution>\n'
    xml += f'{indent}          <min_angle>{angle_config["min"]}</min_angle>\n'
    xml += f'{indent}          <max_angle>{angle_config["max"]}</max_angle>\n'
    xml += f'{indent}        </horizontal>\n'
    xml += f'{indent}      </scan>\n'
    xml += f'{indent}      <range>\n'
    xml += f'{indent}        <min>{range_config["min"]}</min>\n'
    xml += f'{indent}        <max>{range_config["max"]}</max>\n'
    xml += f'{indent}        <resolution>0.01</resolution>\n'
    xml += f'{indent}      </range>\n'
    xml += f'{indent}      <noise>\n'
    xml += f'{indent}        <type>{noise["type"]}</type>\n'
    xml += f'{indent}        <mean>{noise["mean"]}</mean>\n'
    xml += f'{indent}        <stddev>{noise["stddev"]}</stddev>\n'
    xml += f'{indent}      </noise>\n'
    xml += f'{indent}    </ray>\n'
    xml += f'{indent}    <plugin name="{name}_controller" filename="libgazebo_ros_ray_sensor.so">\n'
    xml += f'{indent}      <ros>\n'
    xml += f'{indent}        <namespace>/</namespace>\n'
    xml += f'{indent}        <remapping>~/out:={name}/scan</remapping>\n'
    xml += f'{indent}      </ros>\n'
    xml += f'{indent}      <output_type>sensor_msgs/LaserScan</output_type>\n'
    xml += f'{indent}      <frame_name>{name}_link</frame_name>\n'
    xml += f'{indent}    </plugin>\n'
    xml += f'{indent}  </sensor>\n'
    xml += f'{indent}</gazebo>\n'

    return xml


def generate_imu_gazebo_urdf(sensor, indent='  '):
    """Generate Gazebo IMU plugin URDF"""
    name = sensor['name']
    params = sensor.get('params', {})

    update_rate = params.get('update_rate', 100.0)
    accel_noise = params.get('accel_noise', {'mean': 0.0, 'stddev': 0.01})
    gyro_noise = params.get('gyro_noise', {'mean': 0.0, 'stddev': 0.01})

    xml = f'{indent}<gazebo reference="{name}_link">\n'
    xml += f'{indent}  <sensor type="imu" name="{name}">\n'
    xml += f'{indent}    <always_on>true</always_on>\n'
    xml += f'{indent}    <update_rate>{update_rate}</update_rate>\n'
    xml += f'{indent}    <imu>\n'
    xml += f'{indent}      <angular_velocity>\n'
    xml += f'{indent}        <x>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{gyro_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{gyro_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </x>\n'
    xml += f'{indent}        <y>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{gyro_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{gyro_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </y>\n'
    xml += f'{indent}        <z>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{gyro_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{gyro_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </z>\n'
    xml += f'{indent}      </angular_velocity>\n'
    xml += f'{indent}      <linear_acceleration>\n'
    xml += f'{indent}        <x>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{accel_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{accel_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </x>\n'
    xml += f'{indent}        <y>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{accel_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{accel_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </y>\n'
    xml += f'{indent}        <z>\n'
    xml += f'{indent}          <noise type="gaussian">\n'
    xml += f'{indent}            <mean>{accel_noise["mean"]}</mean>\n'
    xml += f'{indent}            <stddev>{accel_noise["stddev"]}</stddev>\n'
    xml += f'{indent}          </noise>\n'
    xml += f'{indent}        </z>\n'
    xml += f'{indent}      </linear_acceleration>\n'
    xml += f'{indent}    </imu>\n'
    xml += f'{indent}    <plugin name="{name}_controller" filename="libgazebo_ros_imu_sensor.so">\n'
    xml += f'{indent}      <ros>\n'
    xml += f'{indent}        <namespace>/</namespace>\n'
    xml += f'{indent}        <remapping>~/out:={name}/data</remapping>\n'
    xml += f'{indent}      </ros>\n'
    xml += f'{indent}      <frame_name>{name}_link</frame_name>\n'
    xml += f'{indent}    </plugin>\n'
    xml += f'{indent}  </sensor>\n'
    xml += f'{indent}</gazebo>\n'

    return xml


def generate_sensor_gazebo_urdf(sensor, indent='  '):
    """
    Generate Gazebo sensor plugin XML based on sensor type

    Parameters
    ----------
    sensor : dict
        Sensor configuration
    indent : str
        Indentation string

    Returns
    -------
    str : URDF/Gazebo XML for sensor
    """
    sensor_type = sensor.get('type', 'camera')

    generators = {
        'camera': generate_camera_gazebo_urdf,
        'depth_camera': generate_depth_camera_gazebo_urdf,
        'lidar': generate_lidar_gazebo_urdf,
        'ray': generate_lidar_gazebo_urdf,
        'imu': generate_imu_gazebo_urdf,
    }

    generator = generators.get(sensor_type)
    if generator:
        return generator(sensor, indent)

    return f'{indent}<!-- Sensor type "{sensor_type}" not supported yet -->\n'


def generate_sensors_urdf(sensors, indent='  '):
    """
    Generate complete URDF XML for all sensors (links + joints)

    Parameters
    ----------
    sensors : list
        List of sensor configurations
    indent : str
        Indentation string

    Returns
    -------
    str : Complete URDF XML for all sensors
    """
    if not sensors:
        return ''

    xml = f'\n{indent}<!-- Sensors -->\n'

    for sensor in sensors:
        xml += generate_sensor_link_urdf(sensor, indent)
        xml += generate_sensor_joint_urdf(sensor, indent)

    return xml


def generate_sensors_gazebo_urdf(sensors, indent='  '):
    """
    Generate complete Gazebo plugin XML for all sensors

    Parameters
    ----------
    sensors : list
        List of sensor configurations
    indent : str
        Indentation string

    Returns
    -------
    str : Complete Gazebo XML for all sensors
    """
    if not sensors:
        return ''

    xml = f'\n{indent}<!-- Sensor Plugins -->\n'

    for sensor in sensors:
        xml += generate_sensor_gazebo_urdf(sensor, indent)

    return xml


# SDF generation functions

def generate_sensor_sdf(sensor, indent='      '):
    """
    Generate SDF sensor element

    Parameters
    ----------
    sensor : dict
        Sensor configuration
    indent : str
        Indentation string

    Returns
    -------
    str : SDF XML for sensor
    """
    sensor_type = sensor.get('type', 'camera')
    name = sensor['name']
    params = sensor.get('params', {})
    pose = sensor.get('pose', {})
    xyz = pose.get('xyz', [0, 0, 0])
    rpy = pose.get('rpy', [0, 0, 0])

    pose_str = ' '.join(str(v) for v in xyz + rpy)
    update_rate = params.get('update_rate', 30.0)

    xml = f'{indent}<sensor name="{name}" type="{sensor_type}">\n'
    xml += f'{indent}  <pose>{pose_str}</pose>\n'
    xml += f'{indent}  <always_on>true</always_on>\n'
    xml += f'{indent}  <update_rate>{update_rate}</update_rate>\n'

    if sensor_type == 'camera':
        xml += generate_camera_sdf_content(params, indent + '  ')
    elif sensor_type == 'depth_camera':
        xml += generate_depth_camera_sdf_content(params, indent + '  ')
    elif sensor_type in ('lidar', 'ray'):
        xml += generate_lidar_sdf_content(params, indent + '  ')
    elif sensor_type == 'imu':
        xml += generate_imu_sdf_content(params, indent + '  ')

    xml += f'{indent}</sensor>\n'

    return xml


def generate_camera_sdf_content(params, indent):
    """Generate camera-specific SDF content"""
    width = params.get('width', 640)
    height = params.get('height', 480)
    fov = params.get('fov', 1.047)
    clip = params.get('clip', {'near': 0.1, 'far': 100.0})

    xml = f'{indent}<camera>\n'
    xml += f'{indent}  <horizontal_fov>{fov}</horizontal_fov>\n'
    xml += f'{indent}  <image>\n'
    xml += f'{indent}    <width>{width}</width>\n'
    xml += f'{indent}    <height>{height}</height>\n'
    xml += f'{indent}    <format>R8G8B8</format>\n'
    xml += f'{indent}  </image>\n'
    xml += f'{indent}  <clip>\n'
    xml += f'{indent}    <near>{clip["near"]}</near>\n'
    xml += f'{indent}    <far>{clip["far"]}</far>\n'
    xml += f'{indent}  </clip>\n'
    xml += f'{indent}</camera>\n'

    return xml


def generate_depth_camera_sdf_content(params, indent):
    """Generate depth camera-specific SDF content"""
    return generate_camera_sdf_content(params, indent)


def generate_lidar_sdf_content(params, indent):
    """Generate lidar/ray-specific SDF content"""
    samples = params.get('samples', 360)
    resolution = params.get('resolution', 1.0)
    range_config = params.get('range', {'min': 0.1, 'max': 10.0})
    angle_config = params.get('angle', {'min': -3.14159, 'max': 3.14159})

    xml = f'{indent}<ray>\n'
    xml += f'{indent}  <scan>\n'
    xml += f'{indent}    <horizontal>\n'
    xml += f'{indent}      <samples>{samples}</samples>\n'
    xml += f'{indent}      <resolution>{resolution}</resolution>\n'
    xml += f'{indent}      <min_angle>{angle_config["min"]}</min_angle>\n'
    xml += f'{indent}      <max_angle>{angle_config["max"]}</max_angle>\n'
    xml += f'{indent}    </horizontal>\n'
    xml += f'{indent}  </scan>\n'
    xml += f'{indent}  <range>\n'
    xml += f'{indent}    <min>{range_config["min"]}</min>\n'
    xml += f'{indent}    <max>{range_config["max"]}</max>\n'
    xml += f'{indent}    <resolution>0.01</resolution>\n'
    xml += f'{indent}  </range>\n'
    xml += f'{indent}</ray>\n'

    return xml


def generate_imu_sdf_content(params, indent):
    """Generate IMU-specific SDF content"""
    xml = f'{indent}<imu>\n'
    xml += f'{indent}  <orientation_reference_frame>\n'
    xml += f'{indent}    <localization>ENU</localization>\n'
    xml += f'{indent}  </orientation_reference_frame>\n'
    xml += f'{indent}</imu>\n'

    return xml


def generate_sensors_for_link_sdf(sensors, link_name, indent='      '):
    """
    Generate SDF sensors for a specific link

    Parameters
    ----------
    sensors : list
        List of all sensor configurations
    link_name : str
        Name of the parent link
    indent : str
        Indentation string

    Returns
    -------
    str : SDF XML for sensors attached to this link
    """
    link_sensors = [s for s in sensors if s.get('parent_link', 'base_link') == link_name]

    if not link_sensors:
        return ''

    xml = ''
    for sensor in link_sensors:
        xml += generate_sensor_sdf(sensor, indent)

    return xml
