FROM ros:jazzy-ros-base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-joy \
    ros-jazzy-teleop-twist-joy \
    ros-jazzy-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/teleop
COPY pipeline/teleop_twist_joy.yaml /opt/teleop/teleop_twist_joy.yaml

ENV TELEOP_CONFIG=/opt/teleop/teleop_twist_joy.yaml

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && ros2 run joy joy_node & ros2 run teleop_twist_joy teleop_node --ros-args --params-file ${TELEOP_CONFIG}"]
