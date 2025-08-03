FROM ros-base:latest

WORKDIR /home/rosuser/broadcaster_ws

COPY . .

RUN . /opt/ros/humble/setup.sh && \
    colcon build

# Source ROS & entrypoint
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 run broadcaster_ws temperature_broadcaster"]
