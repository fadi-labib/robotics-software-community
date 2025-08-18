FROM ros:humble


COPY temperature_pub  docker_ws/src/temperature_pub 
COPY temperature_interfaces docker_ws/src/temperature_interfaces 


WORKDIR /docker_ws/

RUN . /opt/ros/humble/setup.sh && \
    colcon build

# run the publisher directly instead of using "ros2 run"
# as "ros2 run" appears to not propagate SIGTERM to its children inside the container
# see https://github.com/ros2/ros2cli/issues/895 for more info
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source install/setup.bash && \
      ros2 interface show temperature_interfaces/msg/Temperature && \
      python3 install/temperature_pub/lib/temperature_pub/temp_pub"  ]