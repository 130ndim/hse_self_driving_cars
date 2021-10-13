FROM ros:noetic-robot

SHELL ["/bin/bash", "-c"]

RUN apt-get update -yqq \
    && apt-get install -yqq ros-noetic-turtlesim

RUN mkdir -p /workspace/src \
    && cd /workspace/src \
    && source /opt/ros/noetic/setup.bash \
    && catkin_init_workspace \
    && catkin_create_pkg turtle_chaser rospy

COPY *.py /workspace/src/turtle_chaser/src/
COPY docker-entrypoint.sh /workspace/src/
COPY chase.launch /workspace/src/turtle_chaser/src
RUN chmod +x /workspace/src/docker-entrypoint.sh \
    && chmod +x /workspace/src/turtle_chaser/src/chase.py
WORKDIR /workspace

RUN source /opt/ros/noetic/setup.bash && catkin_make

ENTRYPOINT ["/workspace/src/docker-entrypoint.sh"]

