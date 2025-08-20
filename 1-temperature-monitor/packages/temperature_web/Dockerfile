FROM ros:humble


COPY temperature_web docker_ws/temperature_web
COPY temperature_interfaces docker_ws/src/temperature_interfaces 

# Install rosbridge-server
RUN apt-get update && apt-get install -y \
    ros-humble-rosbridge-server\
    && rm -rf /var/lib/apt/lists/*

WORKDIR /docker_ws


RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Order of commands affects the behavior weirdly 
# (i.e if we run the http server before the rosbridge everything will seem fine but no data reaches the website)
# Launched the rosbridge with those arguments to avoid Warnings that would otherwise clog the logs
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    echo 'Open your browser at: http://localhost:8080' && \
    ros2 launch rosbridge_server rosbridge_websocket_launch.xml 'default_call_service_timeout:=5.0' 'call_services_in_new_thread:=true' 'send_action_goals_in_new_thread:=true' & \
    python3 -m http.server 8080 --directory temperature_web "]