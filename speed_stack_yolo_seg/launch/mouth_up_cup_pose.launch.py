from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("speed_stack_yolo_seg")
    default_weights = os.path.join(
        pkg_share, "weights", "best.pt"
    )

    return LaunchDescription([
        DeclareLaunchArgument("weights_path", default_value=default_weights),
        DeclareLaunchArgument(
            "image_topic", default_value="/camera/camera/color/image_raw"),
        DeclareLaunchArgument(
            "depth_topic",
            default_value="/camera/camera/aligned_depth_to_color/image_raw"),
        DeclareLaunchArgument(
            "camera_info_topic",
            default_value="/camera/camera/color/camera_info"),
        DeclareLaunchArgument("imgsz", default_value="640"),
        DeclareLaunchArgument("conf", default_value="0.25"),
        DeclareLaunchArgument("iou", default_value="0.45"),
        DeclareLaunchArgument("device", default_value="cuda"),
        DeclareLaunchArgument("half", default_value="false"),
        DeclareLaunchArgument(
            "target_class_name", default_value="mouth-up-cup",
            description="검출할 YOLO 클래스 이름 (넓은 입구가 위를 향한 컵)."),
        DeclareLaunchArgument("depth_window", default_value="9"),

        Node(
            package="speed_stack_yolo_seg",
            executable="mouth_up_cup_pose_node",
            name="mouth_up_cup_pose_node",
            output="screen",
            parameters=[{
                "weights_path": LaunchConfiguration("weights_path"),
                "image_topic": LaunchConfiguration("image_topic"),
                "depth_topic": LaunchConfiguration("depth_topic"),
                "camera_info_topic": LaunchConfiguration("camera_info_topic"),

                "grasp_pose_topic": "/mouth_up_cup/grasp_pose",
                "debug_image_topic": "/mouth_up_cup/debug_image",

                "imgsz": LaunchConfiguration("imgsz"),
                "conf": LaunchConfiguration("conf"),
                "iou": LaunchConfiguration("iou"),
                "device": LaunchConfiguration("device"),
                "half": LaunchConfiguration("half"),

                "target_class_name": LaunchConfiguration("target_class_name"),
                "min_mask_area": 300.0,
                "depth_window": LaunchConfiguration("depth_window"),
            }],
        ),
    ])
