import launch
import launch_ros.actions
def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='flying_ndogman',
            executable='camera_publisher',
            name='opencam'),
        launch_ros.actions.Node(
            package='flying_ndogman',
            executable='impros',
            name='impros'),
            ])