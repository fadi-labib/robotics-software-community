FROM ros-base:dev

WORKDIR /home/rosuser/listener_ws

COPY . .

# Build custom messages first
RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select custom_temperature_msgs && \
    . install/setup.sh && \
    colcon build --packages-select temperature_listener

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && \
    source install/setup.bash && \
    ros2 run temperature_listener temperature_listener_node"]
