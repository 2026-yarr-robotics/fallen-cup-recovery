from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Doosan M0609 MoveIt 기본 설정 (URDF, SRDF, kinematics, controllers 등)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    # MoveItPy 전용 YAML
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("dsr_practice"), "config", "moveit_py.yaml"]
    )

    dry_run = LaunchConfiguration("dry_run")
    cup_yaw_override_deg = LaunchConfiguration("cup_yaw_override_deg")
    mode = LaunchConfiguration("mode")
    sim = LaunchConfiguration("sim")
    sim_cup_x = LaunchConfiguration("sim_cup_x")
    sim_cup_y = LaunchConfiguration("sim_cup_y")
    sim_cup_z = LaunchConfiguration("sim_cup_z")
    sim_cup_yaw_deg = LaunchConfiguration("sim_cup_yaw_deg")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dry_run",
                default_value="false",
                description="True면 approach 자세까지만, gripper/descend/lift 스킵",
            ),
            DeclareLaunchArgument(
                "cup_yaw_override_deg",
                default_value="nan",
                description="NaN이 아니면 인식 yaw 무시하고 강제 값 사용",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="drop",
                description="lift 후 동작: 'drop' (3s 대기 후 release) / "
                            "'place' (작업공간으로 옮겨 세우기)",
            ),
            DeclareLaunchArgument(
                "sim",
                default_value="false",
                description="True면 카메라/그리퍼 HW 우회 (MoveIt virtual용)",
            ),
            DeclareLaunchArgument(
                "sim_cup_x",
                default_value="0.40",
                description="sim 모드 가상 컵 base x (m)",
            ),
            DeclareLaunchArgument(
                "sim_cup_y",
                default_value="0.0",
                description="sim 모드 가상 컵 base y (m)",
            ),
            DeclareLaunchArgument(
                "sim_cup_z",
                default_value="0.10",
                description="sim 모드 가상 컵 base z (m)",
            ),
            DeclareLaunchArgument(
                "sim_cup_yaw_deg",
                default_value="0.0",
                description="sim 모드 가상 컵 yaw (deg)",
            ),
            Node(
                package="dsr_practice",
                executable="stand_fallen_cup",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    moveit_py_params,
                    {
                        "dry_run": ParameterValue(dry_run, value_type=bool),
                        "cup_yaw_override_deg": ParameterValue(
                            cup_yaw_override_deg, value_type=float
                        ),
                        "mode": ParameterValue(mode, value_type=str),
                        "sim": ParameterValue(sim, value_type=bool),
                        "sim_cup_x": ParameterValue(sim_cup_x, value_type=float),
                        "sim_cup_y": ParameterValue(sim_cup_y, value_type=float),
                        "sim_cup_z": ParameterValue(sim_cup_z, value_type=float),
                        "sim_cup_yaw_deg": ParameterValue(
                            sim_cup_yaw_deg, value_type=float
                        ),
                    },
                ],
            )
        ]
    )
