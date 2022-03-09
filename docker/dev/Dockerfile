ARG MIL_DOCKER_TAG_ROOT=uf-mil
FROM ${MIL_DOCKER_TAG_ROOT}:base

# Create a mil-dev user and make them a sudoer
RUN useradd --uid 1000 --create-home --shell /bin/bash mil-dev \
  && echo "" >> /etc/sudoers \
  && echo "mil-dev ALL=(ALL) NOPASSWD: ALL" >> /etc/sudoers

# Switch to the new user
USER mil-dev
WORKDIR /home/mil-dev

ADD --chown=mil-dev:mil-dev user_install /tmp/user_install
RUN chmod +x /tmp/user_install && /tmp/user_install
RUN mkdir -p /home/mil-dev/catkin_ws/src/mil

ADD dynparam /tmp/dynparam
RUN sudo cp /tmp/dynparam /opt/ros/melodic/lib/dynamic_reconfigure/dynparam

# Use an interactive bash terminal as the shell
# This allows the user to start the container running a ROS command, etc
ENTRYPOINT ["/bin/bash", "-ic"]

# Default to just opening a shell if no command is specified
CMD ["bash"]
