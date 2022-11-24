import launch_ros.actions
import xacro
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
def get_package_file(package, file_path):
    """Get the location of a file installed in an ament package"""
    package_path = get_package_share_directory(package)
    absolute_file_path = os.path.join(package_path, file_path)
    return absolute_file_path

def load_file(file_path):
    """Load the contents of a file into a string"""
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def run_xacro(xacro_file):
    """Run xacro and output a file in the same directory with the same name, w/o a .xacro suffix"""
    urdf_file, ext = os.path.splitext(xacro_file)
    if ext != '.xacro':
        raise RuntimeError(f'Input file to xacro must have a .xacro extension, got {xacro_file}')
    os.system(f'xacro {xacro_file} -o {urdf_file}')
    return urdf_file

def generate_launch_description():
    # args that can be set from the command line or a default will be used
    sim_robot = DeclareLaunchArgument(
        "sim_robot", default_value=False, description = "Bool, enables ur_hardware")
    robot_ip = DeclareLaunchArgument(
        "robot_ip", default_value='172.31.1.137', description = "Format: XXX.XX.X.XXX: found in info on teach pendant")
    algorithm_selected = DeclareLaunchArgument(
        "algorithm_selected", default_value=TextSubstitution(text="spiral_search"), description = "Select the algorithm to be used: spiral_search or corner_search"
    )
    ros_controllers_file = get_package_file('spiral_search_example_ros2', 'config/ur10e_controllers.yaml')
    xacro_file = get_package_file('spiral_search_example_ros2', 'urdf/workcell.urdf.xacro')
    urdf_file = run_xacro(xacro_file)
    urdf = xacro.process_file(xacro_file).toprettyxml(indent='  ')
    robot_description = load_file(urdf_file)
    rsp = Node(
            name='robot_state_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf}])

    rviz = Node(
            name='rviz2',
            package='rviz2',
            executable='rviz2',
            parameters=[{'robot_description': urdf}])

# include basic ur_robot_driver launch file
    launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('ur_bringup'),'launch'),
                '/ur_control.launch.py']), launch_arguments={'ur_type':'ur10e',
                          'robot_ip':robot_ip,
                          'prefix':'ur_',
                          'runtime_config_package':'spiral_search_example_ros2',
                          'controllers_file':'ur10e_controllers.yaml',
                          'robot_controller':'joint_state_broadcaster',
                          'launch_rviz':'false',
                          'use_fake_hardware':sim_robot,
                          # {'kinematics_config','$(find-pkg-share spiral_search_example_ros2)/config/my_robot_calibration.yaml'},
                          # {'headless_mode','true'},
                          'stopped_controllers':'cartesian_compliance_controller',
                          'controllers':'joint_state_controller',
                          'controller_config_file':'$(find-pkg-share spiral_search_example_ros2)/config/ur10e_controllers.yaml'}.items(),
    )

    # spiral search algorithm
    spiral_search_algorithms = Node(
        package='spiral_search_example_ros2',
        executable='spiral_search_node',
        name='algorithm_node',
        output='screen',
        remappings=[
            ('/wrench','/cartesian_compliance_controller/ft_sensor_wrench'),
        ]
    )

    # Controller manager for realtime interactions
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters= [
            {'robot_description': robot_description},
            ros_controllers_file
        ],
        output="screen",
    )
    # Startup up ROS2 controllers (will exit immediately)
    controller_names = ['joint_state_controller','cartesian_compliance_controller','cartesian_force_controller']
    spawn_controllers = [
        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=[controller],
            output="screen")
        for controller in controller_names
    ]

    # # spiral search plotters
    # spiral_search_plotter = Node(
    #     package='spiral_search_example_ros2',
    #     executable='plotting_node',
    #     name='assembly_plotter',
    #     output="screen",
    # )

    return LaunchDescription([
        algorithm_selected,
        launch_include,
        # spiral_search_plotter,
        spiral_search_algorithms,
        rsp,
        rviz,
        ros2_control_node
    ] + spawn_controllers)


