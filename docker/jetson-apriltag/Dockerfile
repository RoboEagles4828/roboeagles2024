ARG IMAGE_NAME=dustynv/ros:humble-ros-base-l4t-r35.1.0

FROM ${IMAGE_NAME}



# Install the Isaac ROS April Tag
RUN mkdir -p /root/isaac_ros_ws/src
WORKDIR /root/isaac_ros_ws/src
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git
RUN git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_apriltag.git
RUN git clone -b ros2 https://github.com/ros-drivers/usb_cam
RUN source /opt/ros/humble/setup.bash
RUN rosdep install --from-paths ./ --ignore-src -r -y
RUN colcon build --symlink-install



# Setup environment variables 
COPY ros_entrypoint_jetson.sh /sbin/ros_entrypoint.sh
RUN sudo chmod 755 /sbin/ros_entrypoint.sh

ENTRYPOINT ["/sbin/ros_entrypoint.sh"]
CMD ["bash"]

