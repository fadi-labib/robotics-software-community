FROM ros:humble


COPY temperature_pub  docker_ws/src/temperature_pub 
COPY temperature_interfaces docker_ws/src/temperature_interfaces 


WORKDIR /docker_ws/

RUN . /opt/ros/humble/setup.sh && \
    colcon build

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source install/setup.bash && \
      ros2 interface show temperature_interfaces/msg/Temperature"  ]