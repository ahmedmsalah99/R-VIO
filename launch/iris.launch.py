import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    # Get the package share directory
    rvio_dir = get_package_share_directory('rvio')

    # Default config file
    config_file = os.path.join(rvio_dir, 'config', 'rvio_iris.yaml')

    return launch.LaunchDescription([
        SetEnvironmentVariable(
            name='CPUPROFILE',
            value='prof.out'
        ),
        launch_ros.actions.Node(
            package='rvio',
            executable='rvio_mono',
            name='rvio',
            arguments=[config_file],
            output='screen'
            # ,prefix=['valgrind --tool=callgrind --dump-instr=yes -v --instr-atstart=no']
            # ,prefix=['CPUPROFILE=prof.out']
        )
    ])
