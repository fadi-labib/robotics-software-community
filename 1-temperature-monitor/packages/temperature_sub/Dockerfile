FROM ros:humble


COPY temperature_sub  docker_ws/src/temperature_sub 
COPY temperature_interfaces docker_ws/src/temperature_interfaces 


WORKDIR /docker_ws/

RUN . /opt/ros/humble/setup.sh && \
    colcon build

# run the subscriber directly instead of using "ros2 run"
# as "ros2 run" appears to not propagate SIGTERM to its children inside the container
# see https://github.com/ros2/ros2cli/issues/895 for more info
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    install/temperature_sub/lib/temperature_sub/temp_sub"]