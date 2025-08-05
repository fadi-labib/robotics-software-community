FROM ros:humble


COPY temperature_sub  docker_ws/src/temperature_sub 
COPY temperature_interfaces docker_ws/src/temperature_interfaces 


WORKDIR /docker_ws/

RUN . /opt/ros/humble/setup.sh && \
    colcon build

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source install/setup.bash && \
      ros2 topic echo /temperature"  ]