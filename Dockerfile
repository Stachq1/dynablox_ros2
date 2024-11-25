FROM ros:humble

# Set the working directory
WORKDIR /root

# Copy the repository into the container
COPY . /root/dynablox

# Update the package list and install dependencies
RUN apt-get update && apt-get install -y \
    libgoogle-glog-dev \
    libogre-1.9-dev \
    libgflags-dev \
    libyaml-cpp-dev \
    libboost-system-dev \
    libboost-thread-dev \
    libxmlrpcpp-dev \
    protobuf-compiler \
    libpcl-dev \
    vim \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-rviz2

# Allow rviz to find the Ogre libraries
RUN export LD_LIBRARY_PATH=/opt/ros/humble/opt/rviz_ogre_vendor/lib:$LD_LIBRARY_PATH

# Remove apt installed Eigen 3.4.0 and install 3.3.7 instead
RUN rm -rf /usr/include/eigen3 && mv /root/dynablox/eigen3 /usr/include/

# Install the Livox SDK for data inspection
# RUN git clone https://github.com/Livox-SDK/Livox-SDK2.git && \
#     cd ./Livox-SDK2/ && \
#     mkdir build && \
#     cd build && \
#     cmake .. && make -j 2 && \
#     sudo make install

# RUN git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2 && \
#     /bin/bash -c "source /opt/ros/humble/setup.bash" && \
#     cd ws_livox/src/livox_ros_driver2 && \
#     ./build.sh humble

# Source the ROS environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Set the entrypoint to bash
ENTRYPOINT ["/bin/bash"]


############################################
# To run this properly with RViz use:
# xhost +local:root
# docker run -it --rm \
# --net=host \
# -e DISPLAY=$DISPLAY \
# -v /tmp/.X11-unix:/tmp/.X11-unix \
# -v /home/spiasecki/Desktop/rosbags:/root/rosbags \
# dynablox
############################################
