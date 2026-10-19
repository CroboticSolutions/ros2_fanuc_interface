
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription,ExecuteProcess 
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import xacro
from launch_ros.actions import SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


# GUI calibration store (hand_eye_calibration/calibration_store.py): one directory per
# robot model, Apply writes a version and repoints current.yaml.
_CRX_CAMERA_MOUNTS = {"femto_bolt": "femto_bolt_handeye", "realsense_d435": "realsense_d435_handeye"}


def _crx_calibration_file(model, mount, default_name):
    """Active calibration of this CRX for a mount, else the crx_description default."""
    store = os.path.join(
        os.path.expanduser(os.environ.get("ARMS_CALIBRATION_DIR", "~/.ros/calibration")),
        model, mount, "current.yaml")
    if os.path.exists(store):
        return store
    return os.path.join(get_package_share_directory("crx_description"),
                        "config", "calibration", default_name)


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_type",
            default_value="crx20ia_l",
            description="model of the fanuc robot. ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="False to use the standard mock of ros2",
            choices=["true", "false"],
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="controllers.yaml"
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="10.11.31.111",
            description="the IP of the controlled robot",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "read_only",
            default_value="false",
            description="if the robot is read only . ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rmi",
            default_value="false",
            description="if the robot is read only . ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "gz",
            default_value="false",
            description="If mock hardware, simulate using Gazebo ",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "wrist_camera",
            default_value="none",
            description="Wrist camera in the URDF: none | femto_bolt | realsense_d435",
            choices=["none", "femto_bolt", "realsense_d435"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "camera_calibration",
            default_value="",
            description="Camera mount YAML ('' = active store version or crx_description default)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tcp_calibration",
            default_value="",
            description="Welding TCP YAML ('' = active store version or crx_description default)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gz_headless",
            default_value="true",
            description="Use Gazebo in headless mode",
        )
    )
    
    return LaunchDescription( declared_arguments + [OpaqueFunction(function=launch_setup)] )

def launch_setup(context, *args, **kwargs):

    description_package = "crx_description"    
    robot_type = LaunchConfiguration("robot_type")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    robot_ip = LaunchConfiguration("robot_ip")
    controllers_file = LaunchConfiguration("controllers_file")
    read_only = LaunchConfiguration("read_only")
    use_rmi = LaunchConfiguration("use_rmi")
    gz = LaunchConfiguration("gz")
    gz_headless = LaunchConfiguration("gz_headless")

    robot_type_str = robot_type.perform(context)
    # Calibrated wrist camera and TCP: explicit files, else the robot's active store version.
    wrist_camera = LaunchConfiguration("wrist_camera").perform(context)
    camera_calibration = LaunchConfiguration("camera_calibration").perform(context).strip()
    if not camera_calibration and wrist_camera in _CRX_CAMERA_MOUNTS:
        camera_calibration = _crx_calibration_file(
            robot_type_str, _CRX_CAMERA_MOUNTS[wrist_camera], f"{wrist_camera}_handeye_default.yaml")
    tcp_calibration = LaunchConfiguration("tcp_calibration").perform(context).strip() or \
        _crx_calibration_file(robot_type_str, "welding_tcp", "welding_tcp_default.yaml")
    print("calibration", camera_calibration or "-", tcp_calibration)
    print("robot_type", robot_type_str)



    # use_sim_time
    set_use_sim_time = SetParameter(name='use_sim_time', value=LaunchConfiguration('gz'))

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",PathJoinSubstitution([FindPackageShare(description_package), "urdf", robot_type_str, robot_type_str+".xacro"]),
            " ", "robot_type:=", robot_type,
            " ", "use_mock_hardware:=", use_mock_hardware,
            " ", "robot_ip:=", robot_ip,
            " ", "read_only:=", read_only,
            " ", "use_rmi:=", use_rmi,
            " ", "gz:=", gz,
            " ", "wrist_camera:=", wrist_camera,
            " ", "camera_calibration:=", camera_calibration,
            " ", "tcp_calibration:=", tcp_calibration,
        ]
    )
    
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("fanuc_control"),
            "config",
            controllers_file,
        ]
    )
    
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        remappings=[
            (
                "/forward_position_controller/commands",
                "/position_commands",
            ),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator_controller", "-c", "/controller_manager"],
    )
    controller_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["--inactive","forward_position_controller", "-c", "/controller_manager"],
    )

    # move_group reads robot_description from robot_state_publisher's topic (no
    # parameter) and respawns, so GUI calibration Apply can push a new URDF and
    # restart move_group alone (launch_server calibration.py _restart_moveit).
    moveit_params = MoveItConfigsBuilder(
        robot_type_str, package_name=robot_type_str + "_moveit_config"
    ).to_moveit_configs().to_dict()
    moveit_params.pop("robot_description", None)
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        respawn=True,
        respawn_delay=1.0,
        parameters=[
            moveit_params,
            {
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )
    
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory(robot_type_str+'_moveit_config'),'launch', 'moveit_rviz.launch.py')]))
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('crx_description'),'launch', 'gazebo.launch.py')]),
        launch_arguments={"headless" : gz_headless}.items(), 
        condition=IfCondition(gz))

    nodes_to_start = [
        set_use_sim_time,
        control_node,
        joint_state_broadcaster_spawner,
        controller_spawner_started,
        controller_spawner_inactive,
        robot_state_publisher_node,
        move_group,
        moveit_rviz,
        gazebo_launch
    ]

    return nodes_to_start
