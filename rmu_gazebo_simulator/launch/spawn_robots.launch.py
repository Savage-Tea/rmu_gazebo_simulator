# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from sdformat_tools.urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def generate_launch_description():
    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    pkg_simulator = get_package_share_directory("rmu_gazebo_simulator")
    pkg_pb2025_robot_description = get_package_share_directory(
        "pb2025_robot_description"
    )

    robot_xmacro_path = os.path.join(
        pkg_pb2025_robot_description,
        "resource",
        "xmacro",
        "simulation_robot.sdf.xmacro",
    )
    bridge_config = os.path.join(pkg_simulator, "config", "ros_gz_bridge.yaml")
    robot_config = os.path.join(pkg_simulator, "config", "base_params.yaml")

    # Get spawn robot init pose
    gz_world_path = os.path.join(pkg_simulator, "config", "gz_world.yaml")
    with open(gz_world_path) as file:
        config = yaml.safe_load(file)
        selected_world = config.get("world")
        robots = config["robots"].get(selected_world)

    xmacro = XMLMacro4sdf()
    xmacro.set_xml_file(robot_xmacro_path)

    ld = LaunchDescription()

    for robot in robots:
        # Generate SDF from xmacro
        xmacro.generate({"global_initial_color": robot["color"]})
        robot_xml = xmacro.to_string()

        # Inject Airy 96-line GpuLidar sensor into front_mid360 link
        import xml.etree.ElementTree as ET
        root = ET.fromstring(robot_xml)
        airy_sensor_xml = '''<sensor type="gpu_lidar" name="front_airy_sensor">
            <ignition_frame_id>front_mid360</ignition_frame_id>
            <pose>0 0 0.06 0 0 0</pose>
            <always_on>true</always_on>
            <visualize>true</visualize>
            <update_rate>10</update_rate>
            <ray>
              <scan>
                <horizontal>
                  <samples>900</samples>
                  <resolution>1.0</resolution>
                  <min_angle>-3.14159</min_angle>
                  <max_angle>3.14159</max_angle>
                </horizontal>
                <vertical>
                  <samples>96</samples>
                  <min_angle>-0.785398</min_angle>
                  <max_angle>0.785398</max_angle>
                </vertical>
              </scan>
              <range>
                <min>0.1</min>
                <max>60.0</max>
                <resolution>0.005</resolution>
              </range>
            </ray>
            <noise>
              <type>gaussian</type>
              <mean>0.0</mean>
              <stddev>0.01</stddev>
            </noise>
          </sensor>'''
        for link in root.iter('link'):
            if link.get('name') == 'front_mid360':
                airy_elem = ET.fromstring(airy_sensor_xml)
                link.append(airy_elem)
                break

        # DEBUG: verify sensor injection
        found = False
        for sensor in root.iter('sensor'):
            if sensor.get('name') == 'front_airy_sensor':
                found = True; break
        print(f"[spawn_robots] Injected front_airy_sensor into {robot['name']}: {found}")

        robot_xml = ET.tostring(root, encoding='unicode')

        # Generate URDF from SDF
        urdf_generator = UrdfGenerator()
        urdf_generator.parse_from_sdf_string(robot_xml)
        robot_urdf_xml = urdf_generator.to_string()

        # replace the <robot_name> in the bridge config file
        aft_replace_ros_bridge_params = ReplaceString(
            source_file=bridge_config,
            replacements={"<robot_name>": robot["name"]},
        )

        spawn_robot = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-string",
                robot_xml,
                "-name",
                robot["name"],
                "-allow_renaming",
                "true",
                "-x",
                robot["x_pose"],
                "-y",
                robot["y_pose"],
                "-z",
                robot["z_pose"],
                "-Y",
                robot["yaw"],
            ],
        )

        robot_base = Node(
            package="rmoss_gz_base",
            executable="rmua19_robot_base",
            namespace=robot["name"],
            parameters=[robot_config, {"robot_name": robot["name"]}],
        )

        robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            namespace=robot["name"],
            remappings=remappings,
            parameters=[
                {
                    "use_sim_time": True,
                    "robot_description": robot_urdf_xml,
                }
            ],
        )

        robot_ign_bridge = Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            namespace=robot["name"],
            parameters=[{"config_file": aft_replace_ros_bridge_params}],
        )

        # Execute service call after spawning robots
        # https://gazebosim.org/api/gazebo/6.9/levels.html#Runtime-performers
        set_performer_service = ExecuteProcess(
            cmd=[
                "ign",
                "service",
                "-s",
                "/world/default/level/set_performer",
                "--reqtype",
                "ignition.msgs.StringMsg",
                "--reptype",
                "ignition.msgs.Boolean",
                "--timeout",
                "2000",
                "--req",
                f'data: "{robot["name"]}"',
            ],
            output="screen",
        )

        ld.add_action(spawn_robot)
        ld.add_action(robot_base)
        ld.add_action(robot_state_publisher)
        ld.add_action(robot_ign_bridge)
        ld.add_action(set_performer_service)

    return ld
