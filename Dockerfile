FROM ros:melodic

# Install apt dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-melodic-vision-msgs \
    ros-melodic-robot-localization \
    ros-melodic-tf \
    ros-melodic-rviz \
    git \
    && rm -rf /var/lib/apt/lists/*

# Shallow-clone osrf/car_demo and extract only prius_msgs
# (--filter/--sparse require git 2.19+; ros:melodic ships git 2.17)
RUN git clone --depth 1 https://github.com/osrf/car_demo.git /tmp/car_demo \
    && mkdir -p /catkin_ws/src \
    && cp -r /tmp/car_demo/prius_msgs /catkin_ws/src/ \
    && rm -rf /tmp/car_demo

# Copy source
COPY . /catkin_ws/src/cl-rrt/

# Build in Release mode
WORKDIR /catkin_ws
RUN /bin/bash -c \
    "source /opt/ros/melodic/setup.bash && \
     catkin_make -DCMAKE_BUILD_TYPE=Release"

# Source the workspace for every interactive shell
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Custom entrypoint that sources both ROS and the workspace.
# Written inline to avoid Windows CRLF line-ending issues with copied scripts.
RUN printf '#!/bin/bash\nset -e\nsource /opt/ros/melodic/setup.bash\nsource /catkin_ws/devel/setup.bash\nexec "$@"\n' \
    > /docker-entrypoint.sh && chmod +x /docker-entrypoint.sh
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]
