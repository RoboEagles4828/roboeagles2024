ARG IMAGE_NAME=dustynv/ros:humble-ros-base-l4t-r35.1.0

FROM ${IMAGE_NAME}



# Install the Isaac ROS April Tag
RUN mkdir -p /root/${ISAAC_ROS_WS}/src
WORKDIR /root/${ISAAC_ROS_WS}/src
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
RUN git clone -b ros2 https://github.com/ros-drivers/usb_cam
RUN sudo apt-get install -y ros-humble-isaac-ros-apriltag
RUN colcon build --symlink-install
RUN sourcw /root/${ISAAC_ROS_WS}/install/setup.bash



# Setup environment variables 
COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

