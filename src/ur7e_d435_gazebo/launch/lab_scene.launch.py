"""Lab scene shortcut; all camera/mount arguments remain available."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(Path(get_package_share_directory('ur7e_d435_gazebo'))/'launch/scene.launch.py')),
        launch_arguments={'scene_variant': 'lab'}.items())])
