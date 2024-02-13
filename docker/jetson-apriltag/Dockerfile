ARG IMAGE_NAME=dustynv/ros:humble-ros-base-l4t-r35.1.0
ARG ROS2_DIST=humble

FROM ${IMAGE_NAME}

# Install the Isaac ROS April Tag
RUN mkdir -p /root/isaac_ros_ws/src
WORKDIR /root/isaac_ros_ws/src
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
RUN git clone -b ros2 https://github.com/ros-drivers/usb_cam.git
WORKDIR /root/isaac_ros_ws
RUN wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
RUN echo 'deb https://isaac.download.nvidia.com/isaac-ros/ubuntu/main focal main' | sudo tee -a /etc/apt/sources.list

# Install negotiated from source
# https://forums.developer.nvidia.com/t/apt-install-ros-humble-negotiated-error/274317

RUN apt-get update -y && rosdep update && rosdep install --from-paths src --ignore-src -r -y

# DOnt uncomment this its here for reference
# RUN apt install ros-humble-isaac-ros-image-proc ros-humble-isaac-ros-nitros ros-humble-isaac-ros-nitros-april-tag-detection-array-type ros-humble-isaac-ros-nitros-camera-info-type ros-humble-isaac-ros-nitros-image-type ros-humble-isaac-ros-apriltag

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/install/setup.bash && \
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