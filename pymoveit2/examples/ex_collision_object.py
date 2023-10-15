#!/usr/bin/env python3
"""
Example of adding and removing a collision object with a mesh geometry.
Note: Python module `trimesh` is required for this example (`pip install trimesh`).
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p position:="[0.5, 0.0, 0.5]" -p quat_xyzw:="[0.0, 0.0, -0.707, 0.707]"`
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="add" -p filepath:="./my_favourity_mesh.stl"`
`ros2 run pymoveit2 ex_collision_object.py --ros-args -p action:="remove"`
#our required command:
ros2 run pymoveit2 ex_collision_object.py --ros-args -p filepath1:="/home/giri/eyrc_ws/src/pymoveit2/examples/assets/rack.stl" -p action:="add" -p position1:="[0.55, 0.05, -0.6]" -p quat_xyzw1:="[0.0, 0.0, 0.0, -1.0]" -p filepath2:="/home/giri/eyrc_ws/src/pymoveit2/examples/assets/rack2.stl" -p action:="add" -p position2:="[0.25, -0.65, -0.6]" -p quat_xyzw2:="[0.0, 0.0, 1.0, -1.0]" -p filepath3:="/home/giri/eyrc_ws/src/pymoveit2/examples/assets/rack3.stl" -p action:="add" -p position3:="[0.25, 0.65, -0.6]" -p quat_xyzw3:="[0.0, 0.0, 1.0, -1.0]"
"""


from os import path
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node

from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)


def main():
    rclpy.init()

    # Create node for this example
    node = Node("ex_collision_object")

    # Declare parameter for joint positions
    node.declare_parameter(
        "filepath1",
    )

    node.declare_parameter(
        "filepath2",
    )

    node.declare_parameter(
        "filepath3",
    )
    node.declare_parameter(
        "action",
        "add",
    )
    node.declare_parameter("position1", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw1", [0.0, 0.0, -0.707, 0.707])

    node.declare_parameter("position2", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw2", [0.0, 0.0, -0.707, 0.707])

    node.declare_parameter("position3", [0.5, 0.0, 0.5])
    node.declare_parameter("quat_xyzw3", [0.0, 0.0, -0.707, 0.707])

    # Create callback group that allows execution of callbacks in parallel without restrictions
    callback_group = ReentrantCallbackGroup()

    # Create MoveIt 2 interface
    moveit2 = MoveIt2(
        node=node,
        joint_names=ur5.joint_names(),
        base_link_name=ur5.base_link_name(),
        end_effector_name=ur5.end_effector_name(),
        group_name=ur5.MOVE_GROUP_ARM,
        callback_group=callback_group,
    )

    # Spin the node in background thread(s)
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # Get parameters
    filepath1 = node.get_parameter("filepath1").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position1 = node.get_parameter("position1").get_parameter_value().double_array_value
    quat_xyzw1 = node.get_parameter("quat_xyzw1").get_parameter_value().double_array_value


    filepath2 = node.get_parameter("filepath2").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position2 = node.get_parameter("position2").get_parameter_value().double_array_value
    quat_xyzw2 = node.get_parameter("quat_xyzw2").get_parameter_value().double_array_value

    filepath3 = node.get_parameter("filepath3").get_parameter_value().string_value
    action = node.get_parameter("action").get_parameter_value().string_value
    position3 = node.get_parameter("position3").get_parameter_value().double_array_value
    quat_xyzw3 = node.get_parameter("quat_xyzw3").get_parameter_value().double_array_value

    # Use the default example mesh if invalid
    if not filepath1:
        node.get_logger().info(f"Using the default example mesh file")
        filepath = DEFAULT_EXAMPLE_MESH

    # Make sure the mesh file exists
    if not path.exists(filepath1):
        node.get_logger().error(f"File '{filepath1}' does not exist")
        rclpy.shutdown()
        exit(1)

    if not path.exists(filepath2):
        node.get_logger().error(f"File '{filepath2}' does not exist")
        rclpy.shutdown()
        exit(1)
    if not path.exists(filepath3):
        node.get_logger().error(f"File '{filepath3}' does not exist")
        rclpy.shutdown()
        exit(1)
    # Determine ID of the collision mesh
    mesh_id1 = path.basename(filepath1).split(".")[0]
    mesh_id2 = path.basename(filepath2).split(".")[0]
    mesh_id3 = path.basename(filepath3).split(".")[0]

    if "add" == action:
        # Add collision mesh
        node.get_logger().info(
            f"Adding collision mesh '{filepath1}' {{position1: {list(position1)}, quat_xyzw1: {list(quat_xyzw1)}}}"
        )

        node.get_logger().info(
            f"Adding collision mesh '{filepath2}' {{position2: {list(position2)}, quat_xyzw2: {list(quat_xyzw2)}}}"
        )

        node.get_logger().info(
            f"Adding collision mesh '{filepath3}' {{position3: {list(position3)}, quat_xyzw3: {list(quat_xyzw3)}}}"
        )
        # print(ur5.base_link_name())
        while(True):
            moveit2.add_collision_mesh(
                filepath=filepath1, id=mesh_id1, position=position1, quat_xyzw=quat_xyzw1, frame_id=ur5.base_link_name()
            )
            moveit2.add_collision_mesh(
                filepath=filepath2, id=mesh_id2, position=position2, quat_xyzw=quat_xyzw2, frame_id=ur5.base_link_name()
            )
            moveit2.add_collision_mesh(
                filepath=filepath3, id=mesh_id3, position=position3, quat_xyzw=quat_xyzw3, frame_id=ur5.base_link_name()
           
           
            )
    else:
        # Remove collision mesh
        node.get_logger().info(f"Removing collision mesh with ID '{mesh_id1}'")
        moveit2.remove_collision_mesh(id=mesh_id1)
        

        node.get_logger().info(f"Removing collision mesh with ID '{mesh_id2}'")
        moveit2.remove_collision_mesh(id=mesh_id2)

        node.get_logger().info(f"Removing collision mesh with ID '{mesh_id3}'")
        moveit2.remove_collision_mesh(id=mesh_id3)
    rclpy.shutdown()
    exit(0)


if __name__ == "__main__":
    main()
