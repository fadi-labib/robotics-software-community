from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo

# This launch file needs to be run in non interactive mode '-n' also stop_signal = SIGINT in docker compose
# As without this workaround containers will not close cleanly
# see https://github.com/ros2/launch/issues/666 for more info

# However this workaround still does not display nodes shutting down messages for some reason 

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='Starting Temperature Monitor containers.'),
        ExecuteProcess(
            cmd=['bash', '-c', 'docker compose -f docker-compose.yaml up --build'],
            name='Temp-Monitor',
            output='both',
            emulate_tty=True,
        ),

        
    ])