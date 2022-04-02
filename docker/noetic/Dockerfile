# Start with just Ubuntu and ROS Noetic!
FROM ubuntu:focal
FROM ros:noetic
SHELL ["/bin/bash", "-c"]

# Download dependencies
RUN sudo apt-get update -y && sudo apt-get upgrade -y
RUN sudo apt-get install git -y

# Download repository
RUN mkdir -p /usr/catkin_ws/src
WORKDIR /root/catkin_ws/src
RUN git clone --recurse-submodules -j8 https://github.com/uf-mil/mil.git

# Run setup scripts
WORKDIR /root/catkin_ws/src/mil
RUN git checkout noetic-migration
RUN git submodule update --init --recursive
RUN ./scripts/system_install
RUN ./scripts/user_install; exit 0 # Always exits with zero because of darknet_ros error

# Compile with catkin_make
WORKDIR /root/catkin_ws

# Here comes bash!
ENTRYPOINT ["/bin/bash", "-c"]

# Default to just opening a shell if no command is specified
RUN source /opt/ros/noetic/setup.bash

# Compile
RUN sudo apt-get install python doxygen -y
RUN pip3 install breathe pyserial
# ENV PATH="/opt/ros/noetic/bin:${PATH}"
RUN source src/mil/scripts/setup.bash
RUN source /opt/ros/noetic/setup.bash
# ENV PYTHONPATH="/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"
RUN source /opt/ros/noetic/setup.bash && source src/mil/scripts/setup.bash; catkin_make -DCATKIN_BLACKLIST_PACKAGES="darknet_ros"
RUN echo "source /catkin_ws/devel/setup.bash" >> ~/.bashrc

# Build docs
WORKDIR /root/catkin_ws/src/mil
ENV PYTHONPATH="/root/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages:${PYTHONPATH}"
RUN source /opt/ros/noetic/setup.bash && source src/mil/scripts/setup.bash; ./scripts/build_docs
