FROM ros-base:dev

WORKDIR /home/rosuser/listener_ws

COPY . .

RUN . /opt/ros/humble/setup.sh && \
    colcon build

CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 topic echo /temperature_reading"]