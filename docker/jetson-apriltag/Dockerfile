ARG IMAGE_NAME=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_b7e1ed6c02a6fa3c1c7392479291c035
# ARG IMAGE_NAME=dustynv/ros:humble-ros-base-l4t-r35.1.0
ARG ROS2_DIST=humble

FROM ${IMAGE_NAME}

WORKDIR /root/isaac_ros_ws

# Add apt repositories
RUN echo 'ADDING APT REPOS'
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
RUN sudo apt update && sudo apt install curl -y \
  && sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN sudo apt-get update -y
RUN sudo apt-get install ros-humble-isaac-ros-apriltag -y
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && \
  colcon build --parallel-workers $(nproc) --symlink-install \
  --event-handlers console_direct+ --base-paths src \
  --cmake-args ' -DCMAKE_BUILD_TYPE=Release' \
  ' -DCMAKE_LIBRARY_PATH=/usr/local/cuda/lib64/stubs' \
  ' -DCMAKE_CXX_FLAGS="-Wl,--allow-shlib-undefined"' \
  ' --no-warn-unused-cli' "

# Setup environment variables 
COPY ros_entrypoint_jetson.sh /root/isaac_ros_ws/ros_entrypoint.sh
RUN sudo chmod 755 /root/isaac_ros_ws/ros_entrypoint.sh

ENTRYPOINT ["/root/isaac_ros_ws/ros_entrypoint.sh"]
CMD ["bash"]