FROM ros:humble

COPY temperature_interfaces docker_ws/src/temperature_interfaces 
COPY temperature_pub  docker_ws/src/temperature_pub 


WORKDIR /docker_ws/

RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Create a non-root user 
#(From Allison Thackston's Dockerfiles repo https://github.com/athackst/dockerfiles)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Run container as non-root for security 
USER ros

# run the publisher directly instead of using "ros2 run"
# as "ros2 run" appears to not propagate SIGTERM to its children inside the container
# see https://github.com/ros2/ros2cli/issues/895 for more info
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
      source install/setup.bash && \
      ros2 interface show temperature_interfaces/msg/Temperature && \
      python3 install/temperature_pub/lib/temperature_pub/temp_pub"  ]